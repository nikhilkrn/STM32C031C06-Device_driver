/*
 * stm32c031xx_gpio_driver.h
 *
 *  Created on: Oct 6, 2025
 *      Author: NIKHIL
 */

#ifndef INC_STM32C031XX_GPIO_DRIVER_H_
#define INC_STM32C031XX_GPIO_DRIVER_H_

#include "stm32c031xx.h"		// Main Header file where all address and peri are initialized


typedef struct{
	uint8_t GPIO_PinNumber;		/*<-	Possible Values from @GPIO_PIN_Numbers	->*/
	uint8_t GPIO_PinMode;		/*<-	Possible Values from @GPIO_PIN_MODES	->*/
	uint8_t GPIO_PinOpType;		/*<-	Possible Values from @GPIO_PIN_OPTYPES	->*/
	uint8_t GPIO_PinOpSpeed;	/*<-	Possible Values from @GPIO_PIN_OPSPEED	->*/
	uint8_t GPIO_PinPuPd;		/*<-	Possible Values from @GPIO_PIN_PuPd		->*/
	uint8_t GPIO_PinAltrFunc;
}GPIO_PinConfig_t;// Configuration Registers as per manual (See GPIO section to know about these)

typedef struct{
	GPIO_RegDef_t *pGpiox;					// base addr. of the GPIO port to which the port belongs to
	GPIO_PinConfig_t Gpio_PinConfig;		// Gpio Pin Config
}GPIO_Handle_t;



/*
 * ====================================
 *
 * ====================================
 */

/*
 * @GPIO_PIN_Numbers
 * GPIO POSSIBLE PIN NUMBERS
 */
#define	GPIO_PIN_NUMBER_0		0
#define	GPIO_PIN_NUMBER_1		1
#define	GPIO_PIN_NUMBER_2		2
#define	GPIO_PIN_NUMBER_3		3
#define	GPIO_PIN_NUMBER_4		4
#define	GPIO_PIN_NUMBER_5		5
#define	GPIO_PIN_NUMBER_6		6
#define	GPIO_PIN_NUMBER_7		7
#define	GPIO_PIN_NUMBER_8		8
#define	GPIO_PIN_NUMBER_9		9
#define	GPIO_PIN_NUMBER_10		10
#define	GPIO_PIN_NUMBER_11		11
#define	GPIO_PIN_NUMBER_12		12
#define	GPIO_PIN_NUMBER_13		13
#define	GPIO_PIN_NUMBER_14		14
#define	GPIO_PIN_NUMBER_15		15

/*
 * @GPIO_PIN_MODES
 * GPIO POSSIBLE PIN MODES
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4			// Interrupt Falling trigger
#define GPIO_MODE_IT_RT		5			// Interrupt Raising trigger
#define GPIO_MODE_IT_FRT	6			// Interrupt falling raising trigger


/*
 * @GPIO_PIN_OPTYPES
 * GPIO POSSIBLE OUTPUT TYPE
 */
#define GPIO_OP_TYPE_PP		0			// Push Pull Configuration
#define GPIO_OP_TYPE_OD		1			// OpenDrain Configuration


/*
 * @GPIO_PIN_OPSPEED
 * GPIO POSSIBLE SPEED
 */
#define GPIO_SPEED_VL		0			// Very Low Speed
#define GPIO_SPEED_L		1			// Low Speed
#define GPIO_SPEED_H		2			// High Speed
#define GPIO_SPEED_VH		3			// Very High Speed

/*
 * @GPIO_PIN_PuPd
 * GPIO POSSIBLE PULL-UP/PULL-DOWN CONFIGURATION
 */
#define GPIO_PP_NOPP		0			// No PullUp/PullDown
#define GPIO_PP_PU			1			// Pull Up
#define GPIO_PP_PD			2			// Pull Down



/*
 * ==================================================
 *	APIs Supported By This Driver
 * ==================================================
 */

/*	PERIPHERAL CLOCK SETUP	*/
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*	INITIALIZING AND DEINITIALIZING PINS	*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*	READING & WRITING FROM/TO GPIO PIN/PORT	*/
uint8_t GPIO_ReadFromIpPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromIpPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOpPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOpPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOpPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* 	IRQ CONFIGURE AND ISR HANDLING	*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityHandling(uint8_t IRQNumber,uint8_t IRQPriority);


#endif /* INC_STM32C031XX_GPIO_DRIVER_H_ */
