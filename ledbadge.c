#include <stdio.h>
#include "ch32fun.h"
#include "fsusb.h"

#define PIN_CHARGE_STT PA0
#define PIN_KEY1 PA1
#define PIN_KEY2 PB22
#define KEY1_PRESSED funDigitalRead( PIN_KEY1 )
#define KEY2_PRESSED !funDigitalRead( PIN_KEY2 )

int main() {
	SystemInit();

	funGpioInitAll(); // no-op on ch5xx

	funPinMode( PIN_KEY1 | PIN_KEY2, GPIO_CFGLR_IN_PUPD ); // Set PIN_KEY1,2 to input

	USBFSSetup();

	printf("ledbadge\r\n");

	while(1) {
		if( KEY2_PRESSED ) {
			// this should go to a wakeup irq handler on this button
			USBFSReset();
			jump_isprom();
		}
	}
}
