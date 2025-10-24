/*
 * 001LedToggle.c
 *
 *  Created on: Oct 7, 2025
 *      Author: NIKHIL
 */

/*
 * 	Program to toggle on board LED with some delay and following configuration
 * 		case 1: Use Push Pull configuration for the output pin
 * 		Case 2: Use Open drain configuration for the output pin
 * 	On board LED connected to pin no => PA5
 * 	Check User Manual and Schematics
 */

#include "stm32c031xx.h"

void delay(void){
	for(uint32_t i = 0; i <= 500000; i++);
}

int main(void){
	GPIO_Handle_t GpioLed;
	GpioLed.pGpiox = GPIOA;
	GpioLed.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_5;
	GpioLed.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OT;
// togleling led with (Pull up config and PushPull) and (NOPP & Open Drain)

//	GpioLed.Gpio_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioLed.Gpio_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;

//	GpioLed.Gpio_PinConfig.GPIO_PinPuPd = GPIO_PP_NOPP;	// with nopp led doesnot toggle it can only go to gnd but not to high to make it high should enable pull up
	GpioLed.Gpio_PinConfig.GPIO_PinPuPd = GPIO_PP_PU;

	GPIO_PeriClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	while(1){
		GPIO_ToggleOpPin(GPIOA, GPIO_PIN_NUMBER_5);
		delay();
	}
}


