#pragma once

#include "ch32fun.h"
#include "scheduler.h"

static scheduler_t g_scheduler;

#define MSTATUS_MIE 0x8
#define MSTATUS_MPIE 0x80

typedef enum {
    CSR_MSTATUS = 0x300,
    CSR_MEPC    = 0x341,
} csr_t;

#ifdef __GNUC__
#define ALWAYS_INLINE __attribute__((always_inline)) inline
#else
#define ALWAYS_INLINE inline
#endif

ALWAYS_INLINE uint32_t csr_set_bits(csr_t csr_num, uint32_t bits) {
	register uint32_t result;
	asm volatile(ADD_ARCH_ZICSR "csrrs %0, %1, %2" : "=r"(result) : "I"(csr_num), "r"(bits));
	return result;
}

ALWAYS_INLINE uint32_t csr_clear_bits(csr_t csr_num, uint32_t bits) {
	register uint32_t result;
	asm volatile(ADD_ARCH_ZICSR "csrrc %0, %1, %2" : "=r"(result) : "I"(csr_num), "r"(bits));
	return result;
}

ALWAYS_INLINE uint32_t csr_swap(csr_t csr_num, uint32_t value) {
	register uint32_t result;
	asm volatile(ADD_ARCH_ZICSR "csrrw %0, %1, %2" : "=r"(result) : "I"(csr_num), "r"(value));
	return result;
}

ALWAYS_INLINE uint32_t csr_read(csr_t csr_num) {
    register uint32_t result;
    asm(ADD_ARCH_ZICSR "csrr %0, %1" : "=r"(result) : "I"(csr_num));
    return result;
}

ALWAYS_INLINE uint32_t exitInterruptMode() {
	// Disable MPIE so that mret will not re-enable interrupts.
	uint32_t prev_mstatus = csr_clear_bits(CSR_MSTATUS, MSTATUS_MPIE);
	// Execute mret (return from interrupt), but with mepc (interrupt return address) set to 1 instruction after this one.
	// This clears PFIC->IACTR and allows nested interrupts to be handled correctly.
	// This hack is specific to CH32; normally in RISCV you would just set mstatus.mie=1, but that's not enough here.
	register uint32_t prev_mepc;
	asm goto (ADD_ARCH_ZICSR "csrrw %0, mepc, %1; mret" : "=r"(prev_mepc) : "r"((uint32_t)&&after_mret) : : after_mret);
after_mret:
    csr_swap(CSR_MSTATUS, prev_mstatus);
	return prev_mepc;
}

irq_state_t scheduler_irq_set(irq_state_t state) {
    uint32_t prev_mstatus = state ? csr_set_bits(CSR_MSTATUS, MSTATUS_MIE) : csr_clear_bits(CSR_MSTATUS, MSTATUS_MIE);
    irq_state_t prev_state = (prev_mstatus & MSTATUS_MIE) ? IRQ_ENABLED : IRQ_DISABLED;
    // printf("IRQ=%d->%d\n", prev_state, state);
    return prev_state;
}

void scheduler_irq_schedule(sysclk_t alarm) {
    // printf("%u %u\n", (unsigned int)alarm, (unsigned int)(alarm - SysTick->CNT));
    if (sizeof(SysTick->CMP) == sizeof(sysclk_t)) {
        SysTick->CMP = alarm;
    } else {
        uint64_t current_cmp = SysTick->CMP & ((uint64_t)UINT32_MAX << 32);
        if (alarm <= (sysclk_t)SysTick->CNT) {
            SysTick->CMP = (current_cmp + ((uint64_t)1 << 32)) | alarm;
        } else {
            SysTick->CMP = current_cmp | alarm;
        }
    }
    SysTick->CNTL |= SYSTICK_CTLR_STIE; // Re-enable systick interrupts
}

ALWAYS_INLINE sysclk_t scheduler_now(void) {
    return SysTick->CNT;
}

__INTERRUPT
void SysTick_Handler() {
    SysTick->SR = 0; // Clear interrupt flag
    // Disable systick interrupts until the next scheduled alarm
    SysTick->CNTL &= ~SYSTICK_CTLR_STIE;
    // printf("Q\n");
	uint32_t prev_mepc = exitInterruptMode();

    scheduler_result_t result = scheduler_tick_all(&g_scheduler);
    if (result != SCHEDULER_OK) {
        NVIC_SystemReset();
    }

    csr_swap(CSR_MEPC, prev_mepc);
}

void post_task(handler_t handler, void *context, sysclk_t delay, priority_t priority) {
    // printf("Posting task with delay %u at time %u\n", (unsigned int)delay, (unsigned int)g_scheduler.now);
    g_scheduler.now = SysTick->CNT;
    task_t task = {
        .handler = handler,
        .context = context,
        .scheduled_time = g_scheduler.now + delay,
    };
    scheduler_result_t result = scheduler_task(&g_scheduler, priority, &task);
    // printf("Posting task result: %d\n", result);
    if (result != SCHEDULER_OK) {
        NVIC_SystemReset();
    }
}

void scheduler_configure(void) {
    scheduler_init(&g_scheduler);

    // Configure SysTick
    SysTick->CNT = 0;
    SysTick->CTLR = SYSTICK_CTLR_STE;

    NVIC_EnableIRQ(SysTick_IRQn);
}
