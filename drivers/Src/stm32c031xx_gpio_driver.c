/*
 * stm32c031xx_gpio_driver.c
 *
 *  Created on: Oct 6, 2025
 *      Author: NIKHIL
 */

#include<stdint.h>

#include"stm32c031xx_gpio_driver.h"


/*	PERIPHERAL CLOCK SETUP	*/

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
			if(pGPIOx == GPIOA){
				GPIOA_PCLK_EN();
			}
			else if(pGPIOx == GPIOB){
				GPIOB_PCLK_EN();
			}
			else if(pGPIOx == GPIOC){
				GPIOC_PCLK_EN();
			}
			else if(pGPIOx == GPIOD){
				GPIOD_PCLK_EN();
			}
			else if(pGPIOx == GPIOF){
				GPIOF_PCLK_EN();
			}
		}else{
			if(pGPIOx == GPIOA){
				GPIOA_PCLK_DI();
			}
			else if (pGPIOx == GPIOB) {
				GPIOB_PCLK_DI();
			}
			else if(pGPIOx == GPIOC){
				GPIOC_PCLK_DI();
			}
			else if(pGPIOx == GPIOD){
				GPIOD_PCLK_DI();
			}
			else if(pGPIOx == GPIOF){
				GPIOF_PCLK_DI();
			}
		}
}


/*	INITIALIZING AND DEINITIALIZING PINS	*/
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initialize a GPIO Port for a intended role be it input, output,
 * 					 	alternate mode or analog mode with speed
 *
 * @param[in]         -	tells func. to which gpio port and which pin wants to configure with what settings
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp;
	// 1. Configure Pin Mode
	if(pGPIOHandle->Gpio_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		// 1.1 Configure the non-interrupt pin mode
		temp = pGPIOHandle->Gpio_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);  // 2 is * coz there is 2 bit for pin mode reg for each pins
		pGPIOHandle->pGpiox->MODER &= ~(0x03 << 2 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGpiox->MODER |= temp;

	}else{
		// 1.2 Configure the interrupt pin mode
		if(pGPIOHandle->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){

			//1.2.1 configure Falling Trigger reg (FTSR).
			EXTI->FTSR |= (0x01 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);

			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~(0x01 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){

			//1.2.2 configure the RTSR.
			EXTI->RTSR |= (0x01 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);

			// clear the corresponding FTSR bit
			EXTI->FTSR &= ~(0x01 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FRT){

			// 1.2.3 Configure both FTSR and RTSR
			EXTI->RTSR |= (0x01 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (0x01 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);

		}
		// 1.3. Configure the  GPIO Port selection in EXTI_EXTICR
		uint8_t reg_index = pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber / 4;
		uint8_t bit_shift = (pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber % 4) * 8;
		EXTI->EXTICR[reg_index] &= ~(0xFF << bit_shift);
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGpiox);
		EXTI->EXTICR[reg_index] |= (portcode << bit_shift);

		// 1.4. Enable the EXTI Interrupt delivery using IMR
		EXTI->IMR |= (0x01 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	// 2. Configure Speed
	temp = pGPIOHandle->Gpio_PinConfig.GPIO_PinOpSpeed << (2 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGpiox->OSPEEDR &= ~(0x03 << 2 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGpiox->OSPEEDR |= temp;

	temp = 0;
	//3. Configure Pull up Pull Down Setting
	temp = pGPIOHandle->Gpio_PinConfig.GPIO_PinPuPd << (2 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGpiox->PUPDR &= ~(0x03 << 2 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGpiox->PUPDR |= temp;

	temp = 0;
	//4. Configure Output Type
	temp = pGPIOHandle->Gpio_PinConfig.GPIO_PinOpType << (pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGpiox->OTYPER &= ~(0x01 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGpiox->OTYPER |= temp;

	temp = 0;
	//5. Configure Alternate Function
	if(pGPIOHandle->Gpio_PinConfig.GPIO_PinAltrFunc == GPIO_MODE_ALTFN){

		uint8_t temp1 = pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber % 8;

		if(pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber <=7){
			temp = pGPIOHandle->Gpio_PinConfig.GPIO_PinAltrFunc <<(4 * temp1);
			pGPIOHandle->pGpiox->AFRL &= ~(0x0F << 4 * temp1);
			pGPIOHandle->pGpiox->AFRL |= temp;
		}
		else{
			temp = pGPIOHandle->Gpio_PinConfig.GPIO_PinAltrFunc <<(4 * temp1);
			pGPIOHandle->pGpiox->AFRH &= ~(0x0F << 4 * temp1);
			pGPIOHandle->pGpiox->AFRH |= temp;
		}
	}
}


/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function reset peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA){
		GPIOA_REG_RST();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RST();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RST();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RST();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_REG_RST();
	}
}

/*	READING & WRITING FROM/TO GPIO PIN/PORT	*/

/*********************************************************************
 * @fn      		  - GPIO_ReadFromIpPin
 *
 * @brief             - This function Read data from a given PIN
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - PinNumber to read
 *
 * @return            - (uint8_t)1Byte data i.e read value from pin
 *
 * @Note              - (Used in input mode)

 */

uint8_t GPIO_ReadFromIpPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromIpPort
 *
 * @brief             - This function Read the data from the given Port
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @return            -  (uint16_t) values from whole port
 *
 * @Note              -  none

 */
uint16_t GPIO_ReadFromIpPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);

	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOpPin
 *
 * @brief             - This function writes to a given pin
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - Pin number to which data is to be written
 * @param[in]         -	Value or data to write on pin
 *
 * @return            -  none
 *
 * @Note              -  Used in Output mode

 */
void GPIO_WriteToOpPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (0x01 << PinNumber);
	}else{
		pGPIOx->ODR &= ~(0x01 << PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOpPort
 *
 * @brief             - This function write or set value to entire port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - value or data to write to the port
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteToOpPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR |= Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOpPin
 *
 * @brief             - This function toggle between on and off
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - Pin number to toggle
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_ToggleOpPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (0x01 << PinNumber);
}


/* 	IRQ CONFIGURE AND ISR HANDLING	*/
/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - IRQNumber     : The interrupt request number.
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQConfig(uint8_t IRQNumber,  uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		// program 	ISER reg , in M0+ there are only 31 interrupt are possible unlike M4 there are 81 IRQ no so there is IRQ0-2
		*NVIC_ISER |= (1 << IRQNumber);
	}else{
		*NVIC_ICER |= (1 << IRQNumber);
	}

}

/*********************************************************************
 * @fn         - GPIO_IRQPriorityHandling
 *
 * @brief      - This function configures the priority of a given interrupt (IRQ)
 *               in the NVIC. On STM32C0xx (Cortex-M0+), only the upper 2 bits
 *               of each priority field are implemented, providing 4 priority levels (0–3).
 *
 * @param[in]  - IRQNumber     : The interrupt request number.
 * @param[in]  - IRQPriority   : The desired priority level (0 = highest).
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQPriorityHandling(uint8_t IRQNumber,uint8_t IRQPriority){
	uint8_t iprx = IRQNumber /4 ;
	uint8_t iprx_bit = IRQNumber % 4 ;
	uint8_t shifted_bits_numbers = (8 * iprx_bit) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR + iprx) |= (IRQPriority << (shifted_bits_numbers));
}


/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function clear the interrupt flag after an EXTI interrupt occurs on a specific GPIO pin
 *
 * @param[in]         - Pin Number
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQHandling(uint8_t PinNumber){
	uint8_t PinMasking = (1 << PinNumber);
	if (EXTI->FPR & PinMasking){
		EXTI->FPR |= PinMasking;
	}
	else if(EXTI->RPR & PinMasking){
		EXTI->RPR |= PinMasking;
	}
}
