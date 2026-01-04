

#include "ch32fun.h"

#define MATRIX_NROW        11
#define MATRIX_NCOL        44
#define FB_NCOL            (MATRIX_NCOL /2) // number of columns in framebuffer

#define PIN_CHARGE_STT     PA0
#define PIN_CHARGE_STT_INT PIN_CHARGE_STT
#define PIN_KEY1           PA1
#define PIN_KEY1_INT       PIN_KEY1
#define PIN_KEY2           PB22
#define PIN_KEY2_INT       PB8 // ch582 is weird, PB22 interrupt comes in on PB8 bit in irq flag
#define PIN_VBAT_ADC       PA5

// For now copied from ch32fun.LowPowerSleep, since we don't want the RTCTrigger() call in there.
RV_STATIC_INLINE void BigSleep(uint16_t power_plan) 
{
#if defined(CH570_CH572) && (FUNCONF_POWERED_BY_V5PIN == 1)
	power_plan |= RB_PWR_LDO5V_EN;
#endif

#ifdef CH570_CH572
#if ( CLK_SOURCE_CH5XX == CLK_SOURCE_PLL_75MHz ) || ( CLK_SOURCE_CH5XX == CLK_SOURCE_PLL_100MHz )
	//if system clock is higher than 60Mhz, it need to be reduced before sleep.
	SYS_SAFE_ACCESS
	(
		R8_CLK_SYS_CFG = CLK_SOURCE_PLL_60MHz;
	);
#endif
#else
	SYS_SAFE_ACCESS
	(
		R8_BAT_DET_CTRL = 0;
		R8_XT32K_TUNE = (R16_RTC_CNT_32K > 0x3fff) ? (R8_XT32K_TUNE & 0xfc) | 0x01 : R8_XT32K_TUNE;
		R8_XT32M_TUNE = (R8_XT32M_TUNE & 0xfc) | 0x03;
	);
#endif

	NVIC->SCTLR |= (1 << 2); //deep sleep
	SYS_SAFE_ACCESS
	(
		R8_SLP_POWER_CTRL |= RB_RAM_RET_LV;
		R16_POWER_PLAN = RB_PWR_PLAN_EN | RB_PWR_CORE | power_plan;
		R8_PLL_CONFIG |= (1 << 5);
	);
	
	NVIC->SCTLR &= ~(1 << 3); // wfi
	asm volatile ("wfi\nnop\nnop" );

#ifdef CH570_CH572
#if ( CLK_SOURCE_CH5XX == CLK_SOURCE_PLL_75MHz ) || ( CLK_SOURCE_CH5XX == CLK_SOURCE_PLL_100MHz )
	//machine delay for a while.
	uint16_t i = 400;
	do {
		asm volatile("nop");
	} while(i--);

	//get system clock back to normal
	SYS_SAFE_ACCESS
	(
		R8_CLK_SYS_CFG = CLK_SOURCE_CH5XX;
	);
#endif
#else
	SYS_SAFE_ACCESS
	(
		R16_POWER_PLAN &= ~RB_XT_PRE_EN;
		R8_PLL_CONFIG &= ~(1 << 5);
		R8_XT32M_TUNE = (R8_XT32M_TUNE & 0xfc) | 0x01;
	);
#endif
}

#pragma region Layers

typedef enum {
    EVENTS_NONE = 0,
    EVENTS_KEY1 = 1 << 0,
    EVENTS_KEY2 = 1 << 1,
    EVENTS_RTC = 1 << 2,
    EVENTS_CHARGE = 1 << 3,
    EVENTS_RESET = 1 << 4,
} events_t;

static events_t g_events = EVENTS_NONE;

typedef enum {
    ACTIVE_NONE = 0,
    ACTIVE_SCREEN = 1 << 1,
    ACTIVE_BLE = 1 << 2,
    ACTIVE_USB = 1 << 3,
} active_t;

typedef struct {
    uint32_t rtc_now;
    uint32_t rtc_alarm;
    events_t events;
    active_t active;
} layer_result_t;
#define WAIT_IDLE 0

typedef void (*layer_t)(layer_result_t *state, void *self, int next);

#define MAX_LAYERS 10
static layer_t g_layers[MAX_LAYERS] = {NULL};

static void push_layer(layer_t layer) {
    for (int i = 0; i < MAX_LAYERS; i++) {
        if (!g_layers[i]) {
            g_layers[i] = layer;
            break;
        }
    }
}

static void pop_layer() {
    for (int i = MAX_LAYERS-1; i>=0; --i) {
        if (g_layers[i]) {
            g_layers[i] = NULL;
            break;
        }
    }
}

static inline uint32_t timediff(uint32_t a, uint32_t b) {
    uint32_t diff = a - b;
    if (diff > RTC_MAX_COUNT) {
        diff -= RTC_MAX_COUNT;
    }
    return diff;
}

static int rtc_rate(layer_result_t *state, uint32_t cyc, uint32_t *prev_time) {
    if (!(state->events & (EVENTS_RTC | EVENTS_RESET))) {
        return 0;
    }
    uint32_t rtc_alarm = state->rtc_now + cyc;
    if (rtc_alarm > RTC_MAX_COUNT) {
        rtc_alarm -= RTC_MAX_COUNT;
    }
    if (rtc_alarm > 0 && (!state->rtc_alarm || timediff(rtc_alarm, state->rtc_now) < timediff(state->rtc_alarm, state->rtc_now))) {
        state->rtc_alarm = rtc_alarm;
    }
    if (timediff(state->rtc_now, *prev_time) >= cyc) {
        *prev_time = state->rtc_now;
        return 1;
    }
    return 0;
}

static void layer_next(layer_result_t *state, int next) {
    if (next < MAX_LAYERS && g_layers[next]) {
        g_layers[next](state, &g_layers[next], next + 1);
    }
}

static layer_result_t run_layers(events_t events) {
    layer_result_t state = { R32_RTC_CNT_32K, 0, events, 0 };
    layer_next(&state, 0);
    return state;
}

#pragma endregion

#pragma region RTC

__INTERRUPT
__HIGH_CODE
void RTC_IRQHandler() {
	// clear trigger flag
	R8_RTC_FLAG_CTRL = RB_RTC_TRIG_CLR;
    g_events |= EVENTS_RTC;
}

static void RtcLayer(layer_result_t *state, void *self, int next) {
    if (state->events & EVENTS_RESET) {
        LSIEnable(); // Disable LSE, enable LSI
        RTCInit(); // Set the RTC counter to 0 and enable RTC Trigger
        SleepInit();
    }
    layer_next(state, next);
    uint32_t alarm = state->rtc_alarm;
    if (alarm > 0) {
        SYS_SAFE_ACCESS
        (
            R32_RTC_TRIG = alarm;
        );
    }
}

#pragma endregion

#pragma region Screen

#define LINE_A PA15
#define LINE_B PB18
#define LINE_C PB0
#define LINE_D PB7
#define LINE_E PA12
#define LINE_F PA10
#define LINE_G PA11
#define LINE_H PB9
#define LINE_I PB8
#define LINE_J PB15
#define LINE_K PB14
#define LINE_L PB13
#define LINE_M PB12
#define LINE_N PB5
#define LINE_O PA4
#define LINE_P PB3
#define LINE_Q PB4
#define LINE_R PB2
#define LINE_S PB1
#define LINE_T PB6 // or PB23
#define LINE_U PB21
#define LINE_V PB20
#define LINE_W PB19

static u32 led_lines[] = {
	LINE_A, LINE_B, LINE_C, LINE_D, LINE_E,
	LINE_F, LINE_G, LINE_H, LINE_I, LINE_J,
	LINE_K, LINE_L, LINE_M, LINE_N, LINE_O,
	LINE_P, LINE_Q, LINE_R, LINE_S, LINE_T,
	LINE_U, LINE_V, LINE_W
};
#define NLINES (sizeof(led_lines) / sizeof(u32))

typedef uint16_t framebuffer_t[MATRIX_NCOL];

typedef struct {
	uint32_t pa_mode;
	uint32_t pa_val;
	uint32_t pb_mode;
	uint32_t pb_val;
	int on_count;
} chunk_t;

__HIGH_CODE
static chunk_t pack_fb_row(framebuffer_t fb, int chunk) {
    uint16_t a = fb[chunk*2];
    uint16_t b = fb[chunk*2 + 1];
    if (chunk == 0) {
        // Swap first 2 pixels of the first row
        uint16_t a0 = a;
        a = (a & 0xfffe) | (b & 1);
        b = (b & 0xfffe) | (a0 & 1);
    }
    
    uint32_t pa = 0;
    uint32_t pb = 0;
	uint32_t pa_val = 0;
	uint32_t pb_val = 0;
    int on_count = 0;
    for (int i = 0, j = 0; i < NLINES; i++, j++) {
        uint32_t line = led_lines[i];
        int value;
        if (i == chunk) {
            // Row line; always lit
            value = 1;
			// Also push 5v
			if (line & PB) {
				pb_val |= line;
			} else {
				pa_val |= line;
			}
            // The row can't also be a column, so we skip this index
            j--;
        } else {
            value = ((j % 2 ? b : a) >> (j / 2)) & 1;
            on_count += value;
        }
        if (value) {
            if (line & PB) {
                pb |= line;
            } else {
                pa |= line;
            }
        }
    }
    chunk_t ret = { pa, pa_val, pb & ~PB, pb_val & ~PB, on_count };
	return ret;
}

#define RR_BASE 1500
#define RR_DIV 8

static framebuffer_t active_fb = {};
static int active_fb_chunk = 0;
static chunk_t prev_chunk = {};
static int chunk_wait = 0;

static int g_dim = RR_DIV - 1;

__INTERRUPT
__HIGH_CODE
void TMR0_IRQHandler() {
	int status = R8_TMR0_INT_FLAG;

    // Suspend interrupts to avoid glitches during button presses
    uint32_t irqs = NVIC_get_enabled_IRQs();
    NVIC_clear_all_IRQs_except(0);
    if (chunk_wait == g_dim) {
        R32_PA_DIR &= ~prev_chunk.pa_mode;
        R32_PA_OUT &= ~prev_chunk.pa_val;
        R32_PB_DIR &= ~prev_chunk.pb_mode;
        R32_PB_OUT &= ~prev_chunk.pb_val;
    }

    if (chunk_wait == 0) {
        chunk_t chunk = pack_fb_row(active_fb, active_fb_chunk);
        R32_PA_DIR |= chunk.pa_mode;
        R32_PA_OUT |= chunk.pa_val;
        R32_PB_DIR |= chunk.pb_mode;
        R32_PB_OUT |= chunk.pb_val;
        prev_chunk = chunk;
        active_fb_chunk = (active_fb_chunk + 1) % FB_NCOL;
        chunk_wait = RR_DIV - 1;
    } else {
        chunk_wait--;
    }
	R8_TMR0_INT_FLAG = status; // acknowledge
    NVIC_restore_IRQs(irqs);
}

static int screen_irq_active = 0;

static void ScreenLayer(layer_result_t *state, void *self, int next) {
    if (state->events & EVENTS_RESET) {
        R32_TMR0_CNT_END = FUNCONF_SYSTEM_CORE_CLOCK / (RR_BASE * RR_DIV);
        R8_TMR0_CTRL_MOD = RB_TMR_COUNT_EN;
        R8_TMR0_INTER_EN = RB_TMR_IE_CYC_END;
    }
    layer_next(state, next);
    if ((state->active & ACTIVE_SCREEN) ^ screen_irq_active) {
        if (screen_irq_active) {
            NVIC_DisableIRQ(TMR0_IRQn);
        } else {
            NVIC_EnableIRQ(TMR0_IRQn);
        }
        screen_irq_active ^= ACTIVE_SCREEN;
    }
}

#pragma endregion

#pragma region Buttons

void GPIOSetup() {
	funPinMode( PIN_KEY1, GPIO_CFGLR_IN_PD ); // Set PIN_KEY1 to input, pulldown as keypress is to vcc
	funPinMode( PIN_KEY2, GPIO_CFGLR_IN_PU ); // Set PIN_KEY2 to input, pullup as keypress is to gnd
	funPinMode( PIN_CHARGE_STT, GPIO_CFGLR_IN_PU ); // Set PIN_CHARGE_STT to input, pullup as connecting charger makes this go to gnd
	funPinMode( PIN_VBAT_ADC, GPIO_CFGLR_IN_FLOAT ); // Set PIN_VBAT_ADC to floating input for ADC

	// key1 and charge stt interrupts
	R16_PA_INT_MODE |= (PIN_KEY1_INT | PIN_CHARGE_STT_INT); // edge mode, should go to ch32fun.h
	funDigitalWrite(PIN_KEY1, FUN_HIGH); // rising edge
	funDigitalWrite(PIN_CHARGE_STT, FUN_LOW); // falling edge

	NVIC_EnableIRQ(GPIOA_IRQn);
	R16_PA_INT_IF = (PIN_KEY1_INT | PIN_CHARGE_STT_INT); // reset key1 and charge stt flags
	R16_PA_INT_EN |= (PIN_KEY1_INT | PIN_CHARGE_STT_INT); // enable key1 and charge stt interrupts

	// key2 interrupt
	R16_PIN_ALTERNATE |= RB_PIN_INTX; // set PB8 interrupt to PB22 (PIN_KEY2_INT points to PB8)
	R16_PB_INT_MODE |= (PIN_KEY2_INT & ~PB); // edge mode, should go to ch32fun.h
	funDigitalWrite(PIN_KEY2, FUN_LOW); // falling edge

	NVIC_EnableIRQ(GPIOB_IRQn);
	R16_PB_INT_IF = (PIN_KEY2_INT & ~PB); // reset key2 flag
	R16_PB_INT_EN |= (PIN_KEY2_INT & ~PB); // enable key2 interrupt

	// enable wakeup on either key
	SYS_SAFE_ACCESS(
		R8_SLP_WAKE_CTRL |= (RB_WAKE_EV_MODE | RB_SLP_GPIO_WAKE);
		R8_SLP_POWER_CTRL &= ~(RB_WAKE_DLY_MOD);
		R8_SLP_POWER_CTRL |= 0x00; // 0x00: long delay, 0x01: short delay (short doesn't work here)
	);
}

__INTERRUPT
void GPIOA_IRQHandler() {
	int status = R16_PA_INT_IF;
	R16_PA_INT_IF = status; // acknowledge

	if(status & PIN_KEY1_INT) {
        g_events |= EVENTS_KEY1;
	}
	if(status & PIN_CHARGE_STT_INT) {
        g_events |= EVENTS_CHARGE;
	}
}

__INTERRUPT
void GPIOB_IRQHandler() {
	int status = R16_PB_INT_IF;
	R16_PB_INT_IF = status; // acknowledge

	if(status & PIN_KEY2_INT) {
        g_events |= EVENTS_KEY2;
	}
}

static void ButtonsLayer(layer_result_t *state, void *self, int next) {
    if (state->events & EVENTS_RESET) {
        GPIOSetup();
    } else {
        state->events |= g_events;
        g_events = 0;
    }
    layer_next(state, next);
}

#pragma endregion

#pragma region Game-of-life

static void gol() {
    framebuffer_t fb2 = {0};
    for (int x = 0; x < MATRIX_NCOL; x++) {
        for (int y = 0; y < MATRIX_NROW; y++) {
            int ncount = 0;
            for (int xa=-1; xa<=1; xa++) {
                for (int ya=-1; ya<=1; ya++) {
                    if (active_fb[(x + xa + MATRIX_NCOL) % MATRIX_NCOL] & (1 << ((y + ya + MATRIX_NROW) % MATRIX_NROW))) {
                        ncount++;
                    }
                }
            }
            if (ncount == 3) {
                fb2[x] |= 1<<y;
            } else if (ncount == 4) {
                fb2[x] |= active_fb[x] & (1<<y);
            }
        }
    }
    memcpy(active_fb, fb2, sizeof(fb2));
}

/* White Noise Generator State */
#define NOISE_BITS 8
#define NOISE_MASK ((1<<NOISE_BITS)-1)
#define NOISE_POLY_TAP0 31
#define NOISE_POLY_TAP1 21
#define NOISE_POLY_TAP2 1
#define NOISE_POLY_TAP3 0
uint32_t lfsr = 1;

/*
 * random byte generator
 */
uint8_t rand8(void)
{
    uint8_t bit;
    uint32_t new_data;

    for(bit=0;bit<NOISE_BITS;bit++)
    {
        new_data = ((lfsr>>NOISE_POLY_TAP0) ^
                    (lfsr>>NOISE_POLY_TAP1) ^
                    (lfsr>>NOISE_POLY_TAP2) ^
                    (lfsr>>NOISE_POLY_TAP3));
        lfsr = (lfsr<<1) | (new_data&1);
    }

    return lfsr&NOISE_MASK;
}

static void gol_rand() {
    lfsr = R32_RTC_CNT_32K;
	for (int i = 0; i < MATRIX_NCOL; i++) {
		active_fb[i] = rand8() | (rand8()<<8);
	}
}

#define RTC_RATE(hz)  (RTC_FREQ/(hz))

static uint32_t gol_time = 0;

static void GolLayer(layer_result_t *state, void *self, int next) {
    // Request to the ScreenLayer to turn on the LED screen
    state->active |= ACTIVE_SCREEN;
    // If KEY1 is pressed this loop...
    if (state->events & EVENTS_RESET) {
        // Initialize the board
        gol_rand();
    }
    // If `gol_time` was at least 1/50th of a second ago...
    // (this will also set state->rtc_alarm to wake up at the right time)
    if (rtc_rate(state, RTC_RATE(50), &gol_time)) {
        // Run an iteration of the game
        gol();
    }
    // Allow layers later in the stack to make their own changes
    layer_next(state, next);
}

#pragma endregion

#pragma region Bounce

static uint32_t bounce_time = 0;
static int bounce_x = MATRIX_NCOL/2;
static int bounce_y = MATRIX_NROW/2;
static int bounce_dir = 0;

#define BALL_H 3
#define BALL_W 3

static void BounceLayer(layer_result_t *state, void *self, int next) {
    state->active |= ACTIVE_SCREEN;
    if (rtc_rate(state, RTC_RATE(20), &bounce_time)) {
        bounce_x += (bounce_dir & 1) ? 1 : -1;
        bounce_y += (bounce_dir & 2) ? 1 : -1;
        if (bounce_x <= BALL_W/2 || bounce_x >= MATRIX_NCOL-(BALL_W/2)-1) {
            bounce_dir ^= 1;
        }
        if (bounce_y <= BALL_H/2 || bounce_y >= MATRIX_NROW-(BALL_H/2)-1) {
            bounce_dir ^= 2;
        }
        memset(active_fb, 0, sizeof(active_fb));
        uint16_t v = ((1 << BALL_H)-1) << (bounce_y - (BALL_H/2));
        for (int i = -BALL_W/2; i < (BALL_W/2)+1; i++) {
            int x = bounce_x + i;
            if (x < 0 || x >= MATRIX_NCOL) {
                continue;
            }
            active_fb[x] = v;
        }
    }
    layer_next(state, next);
}

#pragma endregion

static void BtlLayer(layer_result_t *state, void *self, int next) {
    if (state->events & EVENTS_KEY2 && funDigitalRead(PIN_KEY1)) {
        DCDCEnable(); // this seems to be needed!?
        jump_isprom();
    }
    if (state->events & EVENTS_KEY2) {
        g_dim = (g_dim + 1) % RR_DIV;
    }
    layer_next(state, next);
}

static layer_t ctrl_layers[] = {
    &GolLayer,
    &BounceLayer,
    NULL,
};
#define CTRL_LAYERS_COUNT  (sizeof(ctrl_layers)/sizeof(layer_t));
static int ctrl_layer_idx = 0;

static void CtrlLayer(layer_result_t *state, void *self, int next) {
    if (state->events & EVENTS_KEY1) {
        ctrl_layer_idx = (ctrl_layer_idx + 1) % CTRL_LAYERS_COUNT;
        state->events |= EVENTS_RESET;
    }
    g_layers[next] = ctrl_layers[ctrl_layer_idx];
    layer_next(state, next);
}

int main() {
    SystemInit();
    push_layer(&RtcLayer);
    push_layer(&ButtonsLayer);
    push_layer(&ScreenLayer);
    push_layer(&BtlLayer);
    push_layer(&CtrlLayer);
    // push_layer(&GolLayer);
    // push_layer(&BounceLayer);

    run_layers(EVENTS_RESET);
    while (1) {
        layer_result_t state = run_layers(EVENTS_NONE);
        if (state.active) {
            __WFI();
        } else {
            BigSleep(0); // TODO: determine a good power plan here - what RAM do we keep, if any?
            __WFI();
            DCDCEnable();
        }
    }
}
