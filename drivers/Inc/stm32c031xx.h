/*
 * stm32c031xx.h
 *
 *  Created on: Oct 5, 2025
 *      Author: NIKHIL
 */

#ifndef INC_STM32C031XX_H_
#define INC_STM32C031XX_H_

#include<stdint.h>


#define __vo volatile

/*
 * ======================================================
 *		PROCESSOR SPECIFIC BASE ADDR NVIC ISER,ICER
 *
 *these info are available in programming manual of M0+
 * ======================================================
 */
#define NVIC_ISER			((__vo uint32_t*) 0xE000E100)
#define NVIC_ICER			((__vo uint32_t*) 0xE000E180)
#define NVIC_IPR			((__vo uint32_t*) 0xE000E400)


/*
 * ==================================================
 *		BASE ADDRESSES OF  BUS,PERIPHERALS
 * ==================================================
 */


#define FLASH_BASE_ADDR				0x08000000UL		// Flash memory base addr
#define SRAM1_BASE_ADDR				0x20000000UL		// in c031 there is only 1 SRAM
#define SRAM						SRAM1_BASEADDR		// SRAM base addr
#define SYSMEM_BASE_ADDR			0x1FFF0000UL		// ROM base addr

// Base ADDR of Buses
#define IOPORTBUS_BASE_ADDR			0x50000000UL		// IO port base addr
#define AHBBUS_BASE_ADDR			0x40020000UL		// AHB bus base addr
#define APBBUS1_BASE_ADDR			0x40000000UL		// base addr of Peripheral
#define APBBUS2_BASE_ADDR			0x40010000UL		// base addr of Peripheral

/*
 * ==================================================
 *		BASE ADDR OF PERIPHERALS ON DIFFERENT BUSES
 * ==================================================
 */

// Peripheral on IOPORT bus
#define GPIOA_BASE_ADDR				(IOPORTBUS_BASE_ADDR + 0x0000UL)
#define GPIOB_BASE_ADDR				(IOPORTBUS_BASE_ADDR + 0x0400UL)
#define GPIOC_BASE_ADDR				(IOPORTBUS_BASE_ADDR + 0x0800UL)
#define GPIOD_BASE_ADDR				(IOPORTBUS_BASE_ADDR + 0x0C00UL)
#define GPIOF_BASE_ADDR				(IOPORTBUS_BASE_ADDR + 0x1400UL)

// Peripheral on AHB bus
#define DMA1_BASE_ADDR				(AHBBUS_BASE_ADDR + 0x0000)
#define DMAMUX_BASE_ADDR			(AHBBUS_BASE_ADDR + 0x0800)
#define RCC_BASE_ADDR				(AHBBUS_BASE_ADDR + 0x1000)
#define EXTI_BASE_ADDR				(AHBBUS_BASE_ADDR + 0x1800)
#define FLASH_REG_BASE_ADDR			(AHBBUS_BASE_ADDR + 0x2000)
#define CRC_BASE_ADDR				(AHBBUS_BASE_ADDR + 0x3000)

// Peripheral on APB bus
#define TIM2_BASE_ADDR				(APBBUS1_BASE_ADDR + 0x0000)		//1KB
#define TIM3_BASE_ADDR				(APBBUS1_BASE_ADDR + 0x0400)		//1KB
#define TIM14_BASE_ADDR				(APBBUS1_BASE_ADDR + 0x2000)		//1KB
#define RTC_BASE_ADDR				(APBBUS1_BASE_ADDR + 0x2800)		//1KB
#define WWDG_BASE_ADDR				(APBBUS1_BASE_ADDR + 0x2C00)		//1KB
#define IWDG_BASE_ADDR				(APBBUS1_BASE_ADDR + 0x3000)		//1KB
#define SPI2_BASE_ADDR				(APBBUS1_BASE_ADDR + 0x3800)		//1KB
#define USART2_BASE_ADDR			(APBBUS1_BASE_ADDR + 0x4400)		//1KB
#define USART3_BASE_ADDR			(APBBUS1_BASE_ADDR + 0x4800)		//1KB
#define USART4_BASE_ADDR			(APBBUS1_BASE_ADDR + 0x4C00)		//1KB
#define I2C1_BASE_ADDR				(APBBUS1_BASE_ADDR + 0x5400)		//1KB
#define I2C2_BASE_ADDR				(APBBUS1_BASE_ADDR + 0x5800)		//1KB
#define USB_BASE_ADDR				(APBBUS1_BASE_ADDR + 0x5C00)		//1KB
#define FDCAN1_BASE_ADDR			(APBBUS1_BASE_ADDR + 0x6400)		//1KB
#define CRS_BASE_ADDR				(APBBUS1_BASE_ADDR + 0x6C00)		//1KB
#define PWR_BASE_ADDR				(APBBUS1_BASE_ADDR + 0x7000)		//1KB
#define USBRAM_BASE_ADDR			(APBBUS1_BASE_ADDR + 0x9800)		//2KB
#define FDCAN_MSG_RAM_BASE_ADDR		(APBBUS1_BASE_ADDR + 0xB400)		//1KB
#define FDCAN_SCRATCH_RAM_BASE_ADDR	(APBBUS1_BASE_ADDR + 0xB800)		//5KB
#define SYSCFG_BASE_ADDR			(APBBUS2_BASE_ADDR + 0x0000)		//28Bytes
#define SYSCFG_ITLINE_BASE_ADDR		(APBBUS2_BASE_ADDR + 0x0080)		//895Bytes
#define ADC_BASE_ADDR				(APBBUS2_BASE_ADDR + 0x2400)		//1kb
#define TIM1_BASE_ADDR				(APBBUS2_BASE_ADDR + 0x2C00)		//1Kb
#define SPI1_BASE_ADDR				(APBBUS2_BASE_ADDR + 0x3000)		//1kb
#define USART1_BASE_ADDR			(APBBUS2_BASE_ADDR + 0x3800)		//1Kb
#define TIM15_BASE_ADDR				(APBBUS2_BASE_ADDR + 0x4000)		//1Kb
#define TIM16_BASE_ADDR				(APBBUS2_BASE_ADDR + 0x4400)		//1KB TIM16 base addr
#define TIM17_BASE_ADDR				(APBBUS2_BASE_ADDR + 0x4800)		//1KB TIM17 base addr
#define DBG_BASE_ADDR				(APBBUS2_BASE_ADDR + 0x5800)		//1KB DBG base addr


/* ================================================
 *
 *	STRUCTURE DEFINATION OF PERIPHERALS REGS
 *
 * ================================================
 * */

/*	Peripheral reg definition structure for GPIO  */

typedef struct{
	__vo uint32_t MODER;	// Offset: 0x00 => GPIO port mode register
	__vo uint32_t OTYPER;	// Offset: 0x04	=> GPIO port output type register => PushPull, Open drain
	__vo uint32_t OSPEEDR;	// Offset: 0x08	=> GPIO port output speed register
	__vo uint32_t PUPDR;	// Offset: 0x0C	=> GPIO port pull-up/pull-down register
	__vo uint32_t IDR;		// Offset: 0x10	=> GPIO port input data register	=> Readonly
	__vo uint32_t ODR;		// Offset: 0x14	=> GPIO port output data register
	__vo uint32_t BSRR;		// Offset: 0x18 => GPIO port bit set/reset register
	__vo uint32_t LCKR;		// Offset: 0x1C => GPIO port configuration lock register
	__vo uint32_t AFRL;		// Offset: 0x20 => GPIO alternate function low register
	__vo uint32_t AFRH;		// Offset: 0x24 => GPIO alternate function high register
	__vo uint32_t BRR;		// Offset: 0x28 => GPIO port bit reset register
}GPIO_RegDef_t;

/*	Peripheral reg definition structure for RCC  */

typedef struct{
	__vo uint32_t CR;		// Offset: 0x00 => RCC clock control register
	__vo uint32_t ICSCR;	// Offset: 0x04 => RCC internal clock source calibration register
	__vo uint32_t CFGR;		// Offset: 0x08 =>RCC clock configuration register
	__vo uint32_t Reserved1;// Offset: 0x0C => RESERVED
	__vo uint32_t Reserved2;// Offset: 0x10	=> RESERVED
	__vo uint32_t CRRCR;	// Offset: 0x14 => RCC clock recovery RC register
	__vo uint32_t CIER;		// Offset: 0x18 => RCC clock interrupt enable register
	__vo uint32_t CIFR;		// Offset: 0x1C => RCC clock interrupt flag register
	__vo uint32_t CICR;		// Offset: 0x20 => RCC clock interrupt clear register
	__vo uint32_t IOPRSTR;	// Offset: 0x24 => RCC I/O port reset register
	__vo uint32_t AHBRSTR;	// Offset: 0x28 => RCC AHB peripheral reset register
	__vo uint32_t APBRSTR1;	// Offset: 0x2C => RCC APB peripheral reset register 1
	__vo uint32_t APBRSTR2;	// Offset: 0x30 => RCC APB peripheral reset register 2
	__vo uint32_t IOPENR;	// Offset: 0x34 => RCC I/O port clock enable register
	__vo uint32_t AHBENR;	// Offset: 0x38 => RCC AHB peripheral clock enable register
	__vo uint32_t APBENR1;	// Offset: 0x3C => RCC APB peripheral clock enable register 1
	__vo uint32_t APBENR2;	// Offset: 0x40 => RCC APB peripheral clock enable register 2
	__vo uint32_t IOPSMENR;	// Offset: 0x44 => RCC I/O port in Sleep mode clock enable register
	__vo uint32_t AHBSMENR;	// Offset: 0x48 => RCC AHB peripheral clock enable in Sleep/Stop mode register
	__vo uint32_t APBSMENR1;// Offset: 0x4C => RCC APB peripheral clock enable in Sleep/Stop mode register 1
	__vo uint32_t APBSMENR2;// Offset: 0x50 => RCC APB peripheral clock enable in Sleep/Stop mode register 2
	__vo uint32_t CCIPR;	// Offset: 0x54 => RCC peripherals independent clock configuration register 1
	__vo uint32_t CCIPR2;	// Offset: 0x58 => RCC peripherals independent clock configuration register 2
	__vo uint32_t CSR1;		// Offset: 0x5C => RCC control/status register 1
	__vo uint32_t CSR2;		// Offset: 0x60 => RCC control/status register 2

}RCC_RegDef_t;

/*	Peripheral reg definition structure for EXTI  */

typedef struct{
	__vo uint32_t RTSR;			// offset: 0x00 =>  Rising trigger selection register
	__vo uint32_t FTSR;			// offset: 0x04 =>  Falling trigger selection register
	__vo uint32_t SWIER;		// offset: 0x08 =>  Software Interrupt event register
	__vo uint32_t RPR;			// offset: 0x0C =>  Rising edge pending register
	__vo uint32_t FPR;			// offset: 0x10 =>  Falling edge pending register
	__vo uint32_t Reserved0[17];// Offset: 0x14-5C => Reserved and STM32C071xx only available registers
	__vo uint32_t EXTICR[4];	// Offset: 0x60 - 0x6C => External interrupt selection register
	__vo uint32_t Reserved1[4];	// Offset: 0x70 - 0x7C => Reserved and STM32C071xx only available registers
	__vo uint32_t IMR;			// Offset: 0x80 => CPU wake-up with interrupt mask register
	__vo uint32_t EMR;			// Offset: 0x84 => CPU wake-up with event mask register
	__vo uint32_t Reserved2[2];	// Offset: 0x88- 0x94 => Only STM32C071xx only available registers

}EXTI_RegDef_t;

typedef struct{
	__vo uint32_t CFGR1; 		// offset: 0x00 SYSCFG configuration register 1
	__vo uint32_t Reserved0[5]; // offset: 0x04 - 0x17
	__vo uint32_t CFGR2;		// offset: 0x18 Bit 0 LOCKUP_LOCK: Cortex®-M0+ LOCKUP enable
	__vo uint32_t Reserved1[8]; // offset: 1C-3B
	__vo uint32_t CFGR3;		// offset: 0x3C SYSCFG configuration register 3
	__vo uint32_t Reserved2[16];// offset: 0x40-0x7F

}SysCfg_regDef_t;

typedef struct{
	__vo uint32_t ITLINE0;		// offset: 0x80 Bit 0WWDG: Window watchdog interrupt pending flag
	__vo uint32_t RESERVED3;	// offset: 0x84 only available on STM32C071xx.
	__vo uint32_t ITLINE2;		// offset: 0x88 Bit 1RTC: RTC interrupt request pending (EXTI line 19)
	__vo uint32_t ITLINE3;		// offset: 0x8C Bit 1 FLASH_ITF: Flash interface interrupt request pending
	__vo uint32_t ITLINE4;		// offset: 0x90 Bit 0RCC: Reset and clock control interrupt request pending
	__vo uint32_t ITLINE5;		// offset: 0x94 Bit 0-1 EXTI0-1: EXTI line 0-1 interrupt request pending
	__vo uint32_t ITLINE6;		// offset: 0x98 Bit 0-1: EXTI 2-3 line interrupt request pending
	__vo uint32_t ITLINE7;		// offset: 0x9C Bit 0-11: EXTI line 4-15 interrupt request pending
	__vo uint32_t RESERVED4;	// offset: 0xA0 only available on STM32C071xx.
	__vo uint32_t ITLINE9;		// offset: 0xA4 Bit 0 DMA1_CH1: DMA1 channel 1interrupt request pending
	__vo uint32_t ITLINE10;		// offset: 0xA8 Bit 0-1 DMA1_CH2-3: DMA1 channel 2-3 interrupt request pending
	__vo uint32_t ITLINE11;		// offset: 0xAC Bit 0DMAMUX: DMAMUX interrupt request pending
	__vo uint32_t ITLINE12;		// offset: 0xB0 Bit 0 ADC: ADC interrupt request pending
	__vo uint32_t ITLINE13;		// offset: 0xB4 Bit 0-3 TIM1_
	__vo uint32_t ITLINE14;		// offset: 0xB8 Bit 0 TIM1_CC: Timer 1 capture compare interrupt request pending
	__vo uint32_t RESERVED5;	// offset: 0xBC only available on STM32C071xx.
	__vo uint32_t ITLINE16;		// offset: 0xC0 Bit 0 TIM3: Timer 3 interrupt request pending
	__vo uint32_t RESERVED6[2];	// offset: 0xC4-0xC8 only available on STM32C071xx.
	__vo uint32_t ITLINE19;		// offset: 0xCC Bit 0 TIM14: Timer 14 interrupt request pending
	__vo uint32_t RESERVED7; 	// offset: 0xD0 only available on STM32C071xx.
	__vo uint32_t ITLINE21;		// offset: 0xD4 Bit 0 TIM16: Timer 16 interrupt request pending
	__vo uint32_t ITLINE22;		// offset: 0xD8 Bit 0 TIM17: Timer 17 interrupt request pending
	__vo uint32_t ITLINE23;		// offset: 0xDC Bit 0 I2C1: I2C1 interrupt request pending, combined with EXTI line 23
	__vo uint32_t RESERVED8;	// offset: 0xE0 only available on STM32C071xx.
	__vo uint32_t ITLINE25;		// offset: 0xE4 Bit 0 SPI1: SPI1 interrupt request pending
	__vo uint32_t RESERVED9;	// offset: 0xE8 only available on STM32C071xx.
	__vo uint32_t ITLINE27;		// offset: 0xEC Bit:0 USART1: USART1 interrupt request pending, combined with EXTI line 25
	__vo uint32_t ITLINE28;		// offset: 0xF0 Bit:0 USART2: USART2 interrupt request pending (EXTI line 26)
	__vo uint32_t RESERVED10[3];// offset: 0xF4-FC only available on STM32C071xx.

}SysCfg_ITLine_regDef_t;


/*
 * ==================================================
 *
 * ==================================================
 */

#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASE_ADDR)


#define RCC 		((RCC_RegDef_t*)RCC_BASE_ADDR)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASE_ADDR)

#define SYSCFG		((SysCfg_regDef_t*)SYSCFG_BASE_ADDR)
#define SYSCFGIT	((SysCfg_ITLine_regDef_t*)SYSCFG_ITLINE_BASE_ADDR)

/*
 * ==================================================
 *		Peripheral Clock Enable Macros
 * ==================================================
 */

/* Clock enable Macros for GPIO peripheral   */
#define GPIOA_PCLK_EN()		(RCC->IOPENR |= (0x01 << 0)	)
#define GPIOB_PCLK_EN()		(RCC->IOPENR |= (0x01 << 1)	)
#define GPIOC_PCLK_EN()		(RCC->IOPENR |= (0x01 << 2)	)
#define GPIOD_PCLK_EN()		(RCC->IOPENR |= (0x01 << 3)	)
#define GPIOF_PCLK_EN()		(RCC->IOPENR |= (0x01 << 5)	)

/* Clock Enable Macros for I2Cx peripherals  */

#define I2C1_PCLK_EN()		(RCC->APBENR1 |= (0x01 << 21))

/* Clock Enable Macros for SPIx peripherals  */

#define SPI1_PCLK_EN()		(RCC->APBENR2 |=(0x01 << 12))


/* Clock Enable Macros for USARTx peripherals  */

#define USART1_PCLK_EN()	(RCC->APBENR2 |= (0x01 << 14))
#define USART2_PCLK_EN()	(RCC->APBENR1 |= (0x01 << 17))

/* Clock Enable Macros for SYSCFG peripherals  */

#define SYSCFG_PCLK_EN()	(RCC->APBENR2 |=(0x01 << 0))


/*
 * ==================================================
 *		Peripheral Clock Disable Macros
 * ==================================================
 */


/* Clock Disable Macros for GPIO peripheral   */
#define GPIOA_PCLK_DI()		(RCC->IOPENR &= ~(0x01 << 0))
#define GPIOB_PCLK_DI()		(RCC->IOPENR &= ~(0x01 << 1))
#define GPIOC_PCLK_DI()		(RCC->IOPENR &= ~(0x01 << 2))
#define GPIOD_PCLK_DI()		(RCC->IOPENR &= ~(0x01 << 3))
#define GPIOF_PCLK_DI()		(RCC->IOPENR &= ~(0x01 << 5))

/* Clock Disable Macros for I2Cx peripherals  */

#define I2C1_PCLK_DI()		(RCC->APBENR1 &= ~(0x01 << 21))

/* Clock Disable Macros for SPIx peripherals  */

#define SPI1_PCLK_DI()		(RCC->APBENR2 &= ~(0x01 << 12))


/* Clock Disable Macros for USARTx peripherals  */

#define USART1_PCLK_DI()	(RCC->APBENR2 &= ~(0x01 << 14))
#define USART2_PCLK_DI()	(RCC->APBENR1 &= ~(0x01 << 17))

/* Clock Disable Macros for SYSCFG peripherals  */

#define SYSCFG_PCLK_DI()	(RCC->APBENR2 &= ~(0x01 << 0))


/*
 * ===================================================
 * 		GPIO PORT RESET MACROS
 * ===================================================
 */
#define GPIOA_REG_RST() 			do{(RCC->IOPRSTR |= (0x01 << 0)); (RCC->IOPRSTR &= ~(0x01 << 0));}while(0)
#define GPIOB_REG_RST() 			do{(RCC->IOPRSTR |= (0x01 << 1)); (RCC->IOPRSTR &= ~(0x01 << 1));}while(0)
#define GPIOC_REG_RST() 			do{(RCC->IOPRSTR |= (0x01 << 2)); (RCC->IOPRSTR &= ~(0x01 << 2));}while(0)
#define GPIOD_REG_RST() 			do{(RCC->IOPRSTR |= (0x01 << 3)); (RCC->IOPRSTR &= ~(0x01 << 3));}while(0)
#define GPIOF_REG_RST() 			do{(RCC->IOPRSTR |= (0x01 << 5)); (RCC->IOPRSTR &= ~(0x01 << 5));}while(0)




/*
 * ========================================================
 *		IRQ number as per RM0490(pg no: 258, table no:55)
 * ========================================================
 */

#define IRQ_NO_EXTI0_1		5
#define IRQ_NO_EXTI2_3		6
#define IRQ_NO_EXTI4_15		7



/*
 * ========================================================
 *		IRQ Priority
 *	Since stm32c031xx only has 2 priority bits so only 4
 *	priority are possible 2^2 =4
 *	as per RM0490(pg no: 257)
 * ========================================================
 */

#define NVIC_IRQ_PRI0       0
#define NVIC_IRQ_PRI1       1
#define NVIC_IRQ_PRI2       2
#define NVIC_IRQ_PRI3       3

/*
 * ==================================================
 *		MISCELLANEOUS Macros
 * ==================================================
 */

#define GPIO_BASEADDR_TO_CODE(baseAddr)			((baseAddr == GPIOA)? 0: \
												 (baseAddr == GPIOB)? 1: \
												 (baseAddr == GPIOC)? 2: \
												 (baseAddr == GPIOD)? 3: \
												 (baseAddr == GPIOF)? 4: 0)




/*
 * ==================================================
 *		GENERIC Macros
 * ==================================================
 */

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define	GPIO_PIN_SET 	SET
#define	GPIO_PIN_RESET	RESET
#define NO_PR_BITS_IMPLEMENTED 4


/*
 *	DRIVERS SPECIFIC HEADER FILES
 */

#include "stm32c031xx_gpio_driver.h"




#endif /* INC_STM32C031XX_H_ */
