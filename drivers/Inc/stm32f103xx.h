/*
 * stm32f103xx.h
 *
 *  Created on: Oct 3, 2024
 *      Author: Waseem Irfan
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stddef.h>
#include <stdint.h>
// some generic macros
#define ENABLE	1
#define DISABLE	0

#define SET		ENABLE
#define RESET 	DISABLE

#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#define FLAG_SET 		SET
#define FLAG_RESET		RESET

#define __vo volatile
// ******************************* START: Processor Specific Details **********************************/
/*
*  ARM Cortex M3 Processor NVIC ISER register addresses
*/

#define NVIC_ISER0			((__vo uint32_t *)	0xE000E100)
#define NVIC_ISER1			((__vo uint32_t *)	0xE000E104)
#define NVIC_ISER2			((__vo uint32_t *)	0xE000E108)
#define NVIC_ISER3			((__vo uint32_t *)	0xE000E10C)

/*
 *  ARM Cortex M3 Processor NVIC ISER register addresses
 */
#define NVIC_ICER0			((__vo uint32_t *)	0XE000E180)
#define NVIC_ICER1			((__vo uint32_t *)	0XE000E184)
#define NVIC_ICER2			((__vo uint32_t *)	0XE000E188)
#define NVIC_ICER3			((__vo uint32_t *)	0XE000E18C)

/*
 *  Interrupt Priority Register Base address
 */

#define NVIC_PR_BASEADDR		((__vo uint32_t *)	0xE000E400)
#define NO_PR_BITS_IMPLEMENTED			4

/*
 * macros for all possible priority levels
 */
#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI1			1
#define NVIC_IRQ_PRI2			2
#define NVIC_IRQ_PRI3			3
#define NVIC_IRQ_PRI4			4
#define NVIC_IRQ_PRI5			5
#define NVIC_IRQ_PRI6			6
#define NVIC_IRQ_PRI7			7
#define NVIC_IRQ_PRI8			8
#define NVIC_IRQ_PRI9			9
#define NVIC_IRQ_PRI10			10
#define NVIC_IRQ_PRI11			11
#define NVIC_IRQ_PRI12			12
#define NVIC_IRQ_PRI13			13
#define NVIC_IRQ_PRI14			14
#define NVIC_IRQ_PRI15			15

/*
 * Base Addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR		0x08000000U 	/*Base Address of Flash memory*/
#define SRAM_BASEADDR		0x20000000U 	/*Base Address of SRAM*/
#define ROM_BASEADDR		0x1FFFF000U 	/*Base Address of ROM (System memory)*/

/*
 * Base Addresses of peripherals of different bus domains of MCU stm32f103xx
 */
#define PERIPH_BASEADDR				0x40000000U     /*Base Address of Peripherals*/
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHBPERIPH_BASEADDR			0x40018000U

/*
 * Base Addresses of all peripherals which are hanging on APB2 Bus
 */
#define GPIOA_BASEADDR				(APB2PERIPH_BASEADDR + 0x0800)
#define GPIOB_BASEADDR				(APB2PERIPH_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define GPIOD_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)
#define GPIOE_BASEADDR				(APB2PERIPH_BASEADDR + 0x1800)
#define GPIOF_BASEADDR				(APB2PERIPH_BASEADDR + 0x1C00)
#define GPIOG_BASEADDR				(APB2PERIPH_BASEADDR + 0x2000)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x0400)
#define AFIO_BASEADDR				 APB2PERIPH_BASEADDR
/*
 * Base Addresses of all peripherals which are hanging on APB1 Bus
 */
#define I2C1_BASEADDR 				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR 				(APB1PERIPH_BASEADDR + 0x5800)
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base Addresses of all peripherals which are hanging on AHB Bus
 */
#define RCC_BASEADDR				0x40021000U
/*
 * peripheral register definition for GPIO
 */
typedef struct{
	__vo uint32_t CRL;			/*Port configuration register low , 	Address Offset = 0x00*/
	__vo uint32_t CRH;			/*Port configuration register high , 	Address Offset = 0x04*/
	__vo uint32_t IDR;			/*Port input data register , 			Address Offset = 0x04*/
	__vo uint32_t ODR;			/*Port output data register , 			Address Offset = 0x0C*/
	__vo uint32_t BSRR;			/*Port bit set/reset register , 		Address Offset = 0x10*/
	__vo uint32_t BRR;			/*Port bit reset register , 			Address Offset = 0x14*/
	__vo uint32_t LCKR;			/*Port configuration lock register,		Address Offset = 0x18*/
}GPIO_RegDef_t;

/*
 * peripheral register definition for GPIO
 */
typedef struct{
	__vo uint32_t SPI_CR1;		/*SPI control register 1 (not used in I2S mode), Address offset: 0x00 */
	__vo uint32_t SPI_CR2;		/*SPI control register 2,						 Address offset: 0x04 */
	__vo uint32_t SPI_SR;		/*SPI status register, 							 Address offset: 0x08 */
	__vo uint32_t SPI_DR;		/*SPI data register,							 Address offset: 0x0C */
	__vo uint32_t SPI_CRCPR;	/*SPI CRC polynomial register,					 Address offset: 0x10 */
	__vo uint32_t SPI_RXCRCR;	/*SPI RX CRC register,							 Address offset: 0x14 */
	__vo uint32_t SPI_TXCRCR;	/*SPI TX CRC register  (not used in I2S mode), 	 Address offset: 0x18 */
	__vo uint32_t SPI_I2SCFGR;	/*SPI_I2S configuration register,				 Address offset: 0x1C */
	__vo uint32_t SPI_I2SPR;	/*SPI_I2S prescaler register, 					 Address offset: 0x20 */

}SPI_RegDef_t;

/*
* peripheral register definition for AFIO
*/

typedef struct{
	__vo uint32_t EVCR;														/*  Address Offset = 0x00 */
	/*MAPR[0] for low-, medium- high- and XL-density devices,					Address Offset = 0x04
	  MAPR[1] for connectivity line devices, 									Address Offset = */
	__vo uint32_t MAPR[2];
//	__vo uint32_t EXTICR1; /*External interrupt configuration register 1,		Address Offset = 0x08 */
//	__vo uint32_t EXTICR2; /*External interrupt configuration register 2,		Address Offset = 0x0C */
//	__vo uint32_t EXTICR3; /*External interrupt configuration register 3,		Address Offset = 0x10 */
//	__vo uint32_t EXTICR4; /*External interrupt configuration register 4,		Address Offset = 0x14 */
	__vo uint32_t EXTICR[4];
	__vo uint32_t MAPR2;   /*AF remap and debug I/O configuration register2,	Address Offset = */
}AFIO_RegDef_t;

/*
 * peripheral register definition for RCC
 */
typedef struct{
	__vo uint32_t CR;			/*Clock control register,			 			Address Offset = 0x00*/
	__vo uint32_t CFGR;			/*Clock configuration register,					Address Offset = 0x04*/
	__vo uint32_t CIR;			/*Clock interrupt register,						Address Offset = 0x08*/
	__vo uint32_t APB2RSTR;		/*APB2 peripheral reset register,				Address Offset = 0x0C*/
	__vo uint32_t APB1RSTR;		/*APB1 peripheral reset register,				Address Offset = 0x10*/
	__vo uint32_t AHBENR;		/*AHB Peripheral Clock enable register,			Address Offset = 0x14*/
	__vo uint32_t APB2ENR;		/*APB2 peripheral clock enable register,		Address Offset = 0x18*/
	__vo uint32_t APB1ENR;		/*APB1 peripheral clock enable register,		Address Offset = 0x1C*/
	__vo uint32_t BDCR;			/*Backup domain control register,				Address Offset = 0x20*/
	__vo uint32_t CSR;			/*Control/status register,						Address Offset = 0x24*/
	__vo uint32_t AHBSTR;		/*AHB peripheral clock reset register,			Address Offset = 0x28*/
	__vo uint32_t CFGR2;		/*Clock configuration register2,				Address Offset = 0x2C*/
}RCC_RegDef_t;

/*
 * peripheral register definition for RCC
 */
typedef struct{
	__vo uint32_t IMR;			/*Interrupt mask register, 					Address Offset = 0x00 */
	__vo uint32_t EMR;			/*Event mask register , 					Address Offset = 0x04 */
	__vo uint32_t RTSR;			/*Rising trigger selection register,		Address Offset = 0x08 */
	__vo uint32_t FTSR;			/*Falling trigger selection register,		Address Offset = 0x0C */
	__vo uint32_t SWIER;		/*Software interrupt event register,		Address Offset = 0x10 */
	__vo uint32_t PR;			/*Pending register, 	 					Address Offset = 0x14 */

}EXTI_RegDef_t;

/*
 *  peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASEADDR)

#define RCC			((RCC_RegDef_t*)  RCC_BASEADDR)
#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)
#define AFIO		((AFIO_RegDef_t*)AFIO_BASEADDR)

#define SPI1		((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*) SPI3_BASEADDR)

/*
 * Reset Macros for GPIOx peripherals
 */
#define GPIOA_REG_RESET()			do{(RCC->APB2RSTR |= (1<<2));	(RCC->APB2RSTR &= ~(1<<2));}while(0)
#define GPIOB_REG_RESET()			do{(RCC->APB2RSTR |= (1<<3));	(RCC->APB2RSTR &= ~(1<<3));}while(0)
#define GPIOC_REG_RESET()			do{(RCC->APB2RSTR |= (1<<4));	(RCC->APB2RSTR &= ~(1<<4));}while(0)
#define GPIOD_REG_RESET()			do{(RCC->APB2RSTR |= (1<<5));	(RCC->APB2RSTR &= ~(1<<5));}while(0)
#define GPIOE_REG_RESET()			do{(RCC->APB2RSTR |= (1<<6));	(RCC->APB2RSTR &= ~(1<<6));}while(0)

/*
 * Reset Macros for SPIx peripherals
 */
#define SPI1_REG_RESET()			do{(RCC->APB2ENR |= (1 << 12));	(RCC->APB2ENR  &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()			do{(RCC->APB1ENR |= (1 << 14));	(RCC->APB1ENR  &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()			do{(RCC->APB1ENR |= (1 << 15));	(RCC->APB1ENR  &= ~(1 << 15));}while(0)
/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()			(RCC->APB2ENR |= (1 << 2)) 	/* IO port A clock enable*/
#define GPIOB_PCLK_EN()			(RCC->APB2ENR |= (1 << 3)) 	/* IO port B clock enable*/
#define GPIOC_PCLK_EN()			(RCC->APB2ENR |= (1 << 4)) 	/* IO port C clock enable*/
#define GPIOD_PCLK_EN()			(RCC->APB2ENR |= (1 << 5))	/* IO port D clock enable*/
#define GPIOE_PCLK_EN()			(RCC->APB2ENR |= (1 << 6)) 	/* IO port E clock enable*/
#define GPIOF_PCLK_EN()			(RCC->APB2ENR |= (1 << 7))	/* IO port F clock enable*/
#define GPIOG_PCLK_EN()			(RCC->APB2ENR |= (1 << 8)) 	/* IO port G clock enable*/
/*
 * Clock Enable Macros for AFIO peripherals
 */
#define AFIO_PCLK_EN()			(RCC->APB2ENR |= (1 << 0))
/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21))  /*I2C1 clock enable*/
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22))  /*I2C2 clock enable*/

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12))  /*SPI1 clock enable*/
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14))  /*SPI2 clock enable*/
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15))  /*SPI3 clock enable*/

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))  /*USART1 clock enable*/
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17))  /*USART2 clock enable*/
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1 << 18))  /*USART3 clock enable*/
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1 << 19))  /*UART4 clock enable*/
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1 << 20))  /*UART5 clock enable*/

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 2)) 	/* IO port A clock disable*/
#define GPIOB_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 3)) 	/* IO port B clock disable*/
#define GPIOC_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 4)) 	/* IO port C clock disable*/
#define GPIOD_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 5))		/* IO port D clock disable*/
#define GPIOE_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 6)) 	/* IO port E clock disable*/
#define GPIOF_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 7))		/* IO port F clock disable*/
#define GPIOG_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 8)) 	/* IO port G clock disable*/

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 21))  /*I2C1 clock disable*/
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 22))  /*I2C2 clock disable*/

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12))  /*SPI1 clock disable*/
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14))  /*SPI2 clock disable*/
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15))  /*SPI3 clock disable*/

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))  /*USART1 clock disable*/
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17))  /*USART2 clock disable*/
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 18))  /*USART3 clock disable*/
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 19))  /*UART4 clock disable*/
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 20))  /*UART5 clock disable*/

/*
 * Return port code for GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA)? 0 :\
										 (x == GPIOB)? 1 :\
										 (x == GPIOC)? 2 :\
										 (x == GPIOD)? 3 :\
										 (x == GPIOE)? 4 :\
										 (x == GPIOF)? 5 :\
										 (x == GPIOG)? 6 :0)
/*
 * Creating macros for IRQ Number
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51

#endif /* INC_STM32F103XX_H_ */
