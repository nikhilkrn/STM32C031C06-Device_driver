/*
 * 003LedExternalBtn.c
 *
 *  Created on: Oct 9, 2025
 *      Author: NIKHIL
 */



/*
 * 	Program to toggle the external LED connected to PB12 with external button connected to pin number PA8
 * 	Nucleo C031C6 => led => PB12 , button => PA8
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

	GpioLed.pGpiox = GPIOB;

	GpioLed.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_12;
	GpioLed.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OT;

	GpioLed.Gpio_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;

	GpioLed.Gpio_PinConfig.GPIO_PinPuPd = GPIO_PP_PU;

	GPIO_PeriClkCtrl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);

	//

	GpioBtn.pGpiox = GPIOA;

	GpioBtn.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_8;
	GpioBtn.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.Gpio_PinConfig.GPIO_PinPuPd = GPIO_PP_PU;

	GPIO_PeriClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1){
		if(GPIO_ReadFromIpPin(GPIOA, GPIO_PIN_NUMBER_8) == BTNPRESSED ){
			delay();
			GPIO_WriteToOpPin(GPIOB, GPIO_PIN_NUMBER_12, GPIO_PIN_SET);
		}
	}
}
