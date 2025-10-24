/*
 * 004InterruptBtn.c
 *
 *  Created on: Oct 17, 2025
 *      Author: NIKHIL
 */

/*
 * 	Program to toggle the external LED connected to PB12 with external button connected to
 * 	pin number PA2 whenever interrupt is triggered by button pressed.
 * 	Interrupt should be triggered during the falling edge of button pressed
 *
 * 	Nucleo C031C6 => led => PB12 , button => PA2
 * 	Check User Manual and Schematics
 */


#include "stm32c031xx.h"
#include <string.h>

#define ENABLE 1

void EXTI2_3_IRQHandler(void);

int main(){
	GPIO_Handle_t GpioBtn, GpioLed;

	//making all the fields of structure to 0 so that it won't take some garbage value.
	memset(&GpioBtn,0,sizeof(GpioBtn));
	memset(&GpioLed,0,sizeof(GpioLed));

	GpioLed.pGpiox = GPIOB;
	GpioLed.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OT;
	GpioLed.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_12;
	GpioLed.Gpio_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	GpioLed.Gpio_PinConfig.GPIO_PinPuPd = GPIO_PP_PU;

	GPIO_PeriClkCtrl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);


	GpioBtn.pGpiox = GPIOA;
	GpioBtn.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_2;
	GpioBtn.Gpio_PinConfig.GPIO_PinPuPd = GPIO_PP_PU;

	GPIO_PeriClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	GPIO_IRQPriorityHandling(IRQ_NO_EXTI2_3, NVIC_IRQ_PRI1);
	GPIO_IRQConfig(IRQ_NO_EXTI2_3, ENABLE);


}

void EXTI2_3_IRQHandler(void){
	// need to prevent debouncing apply delay here
	GPIO_IRQHandling(GPIO_PIN_NUMBER_2);
	GPIO_ToggleOpPin(GPIOB, GPIO_PIN_NUMBER_12);
}
