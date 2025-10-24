/*
 * 002Led_button.c
 *
 *  Created on: Oct 8, 2025
 *      Author: NIKHIL
 */

/*
 * 	Program to toggle the on board LED Whenever the on board button is pressed
 * 	Nucleo C031C6 => On board led => PA5 , On board user button => PC13
 * 	Check User Manual and Schematics
 */

#include "stm32c031xx.h"

#define LOW		DISABLE
#define BTNPRESSED	LOW

void delay(void){
	for(uint32_t i = 0; i <= 500000; i++);
}

int main(void){

	GPIO_Handle_t GpioLed,GpioBtn;

	GpioLed.pGpiox = GPIOA;

	GpioLed.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_5;
	GpioLed.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OT;

	GpioLed.Gpio_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;

	GpioLed.Gpio_PinConfig.GPIO_PinPuPd = GPIO_PP_PU;

	GPIO_PeriClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	//

	GpioBtn.pGpiox = GPIOC;

	GpioBtn.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_13;
	GpioBtn.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.Gpio_PinConfig.GPIO_PinPuPd = GPIO_PP_NOPP;

	GPIO_PeriClkCtrl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1){
		if(GPIO_ReadFromIpPin(GPIOC, GPIO_PIN_NUMBER_13) == BTNPRESSED ){
			delay();
			GPIO_WriteToOpPin(GPIOC, GPIO_PIN_NUMBER_13, GPIO_PIN_SET);
		}
	}
}

