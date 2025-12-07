#include <stdio.h>
#include "ch32fun.h"
#include "fsusb.h"

#define LED            PA8 // DEBUG led on board, not on badge
#define PIN_CHARGE_STT PA0
#define PIN_KEY1       PA1
#define PIN_KEY1_INT   PIN_KEY1
#define PIN_KEY2       PB22
#define PIN_KEY2_INT   PB8 // ch582 is weird, PB22 interrupt comes in on PB8 bit in irq flag


void blink(int n) {
	for(int i = n-1; i >= 0; i--) {
		funDigitalWrite( LED, FUN_LOW ); // Turn on LED
		Delay_Ms(33);
		funDigitalWrite( LED, FUN_HIGH ); // Turn off LED
		if(i) Delay_Ms(33);
	}
}


__INTERRUPT
void GPIOA_IRQHandler() {
	int status = R16_PA_INT_IF;
	R16_PA_INT_IF = status; // acknowledge

	if(status & PIN_KEY1) {
		blink(1);
	}
}

__INTERRUPT
void GPIOB_IRQHandler() {
	int status = R16_PB_INT_IF;
	R16_PB_INT_IF = status; // acknowledge

	if(status & PIN_KEY2_INT) {
		blink(1);
		USBFSReset();
		jump_isprom();
	}
}

void GPIOSetup() {
	funPinMode( PIN_KEY1, GPIO_CFGLR_IN_PD ); // Set PIN_KEY1 to input, pulldown as keypress is to vcc
	funPinMode( PIN_KEY2, GPIO_CFGLR_IN_PU ); // Set PIN_KEY2 to input, pullup as keypress is to gnd

	// key1 interrupt
	R16_PA_INT_MODE |= (PIN_KEY1_INT); // edge mode, should go to ch32fun.h
	funDigitalWrite(PIN_KEY1, FUN_HIGH); // rising edge

	NVIC_EnableIRQ(GPIOA_IRQn);
	R16_PA_INT_IF = (PIN_KEY1_INT); // reset key1 flag
	R16_PA_INT_EN |= (PIN_KEY1_INT); // enable key1 interrupt

	// key2 interrupt
	R16_PIN_ALTERNATE |= RB_PIN_INTX; // set PB8 interrupt to PB22 (PIN_KEY2_INT points to PB8)
	R16_PB_INT_MODE |= (PIN_KEY2_INT & ~PB); // edge mode, should go to ch32fun.h
	funDigitalWrite(PIN_KEY2, FUN_LOW); // falling edge

	NVIC_EnableIRQ(GPIOB_IRQn);
	R16_PB_INT_IF = (PIN_KEY2_INT & ~PB); // reset key2 flag
	R16_PB_INT_EN |= (PIN_KEY2_INT & ~PB); // enable key2 interrupt
}


int main() {
	SystemInit();

	funGpioInitAll(); // no-op on ch5xx

	GPIOSetup();
	funPinMode( LED, GPIO_CFGLR_OUT_2Mhz_PP ); // DEBUG led on board, not on badge

	USBFSSetup();

	printf("ledbadge\r\n");
	blink(5);

	while(1);
}
