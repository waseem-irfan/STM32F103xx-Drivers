/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Oct 4, 2024
 *      Author: Waseem Irfan
 */

#ifndef STM32F103XX_GPIO_DRIVER_H_
#define STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"

typedef struct{
	uint8_t	GPIO_PinNumber;			/*<possible values from @GPIO_PIN_NUMBER macros>*/
	uint8_t GPIO_PinMode;			/*<possible values from @GPIO_PIN_MODE macros>*/
	uint8_t	GPIO_PinSpeed;			/*<possible values from @GPIO_PIN_MODE macros>*/
	uint8_t GPIO_PinPuPdControl;	/*<possible values from @GPIO_PIN_MODE macros>*/
	uint8_t GPIO_OPType;			/*<possible values from @GPIO_PIN_MODE macros>*/
	uint8_t	GPIO_PinAltFunMode;		/*<possible values from @GPIO_PIN_MODE macros>*/
}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx;		/* This holds the base address of GPIO port to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

//@GPIO_PIN_NUMBER macros
#define GPIO_PIN_0					0
#define GPIO_PIN_1					1
#define GPIO_PIN_2					2
#define GPIO_PIN_3					3
#define GPIO_PIN_4					4
#define GPIO_PIN_5					5
#define GPIO_PIN_6					6
#define GPIO_PIN_7					7
#define GPIO_PIN_8					8
#define GPIO_PIN_9					9
#define GPIO_PIN_10					10
#define GPIO_PIN_11					11
#define GPIO_PIN_12					12
#define GPIO_PIN_13					13
#define GPIO_PIN_14					14
#define GPIO_PIN_15					15

// @GPIO_PIN_MODE macros
// Input mode macros (MODEy = 00)
#define GPIO_MODE_INPUT_ANALOG     0x00 // CNF = 00, MODE = 00
#define GPIO_MODE_INPUT_FLOATING   0x04 // CNF = 01, MODE = 00
#define GPIO_MODE_INPUT_PUPD       0x08 // CNF = 10, MODE = 00

// Output mode macros (MODEy > 00)
#define GPIO_MODE_OUTPUT_PP_10MHZ  0x01 // CNF = 00, MODE = 01
#define GPIO_MODE_OUTPUT_PP_2MHZ   0x02 // CNF = 00, MODE = 10
#define GPIO_MODE_OUTPUT_PP_50MHZ  0x03 // CNF = 00, MODE = 11

#define GPIO_MODE_OUTPUT_OD_10MHZ  0x05 // CNF = 01, MODE = 01
#define GPIO_MODE_OUTPUT_OD_2MHZ   0x06 // CNF = 01, MODE = 10
#define GPIO_MODE_OUTPUT_OD_50MHZ  0x07 // CNF = 01, MODE = 11

// Alternate function output mode macros
#define GPIO_MODE_AF_PP_10MHZ      0x09 // CNF = 10, MODE = 01
#define GPIO_MODE_AF_PP_2MHZ       0x0A // CNF = 10, MODE = 10
#define GPIO_MODE_AF_PP_50MHZ      0x0B // CNF = 10, MODE = 11

#define GPIO_MODE_AF_OD_10MHZ      0x0D // CNF = 11, MODE = 01
#define GPIO_MODE_AF_OD_2MHZ       0x0E // CNF = 11, MODE = 10
#define GPIO_MODE_AF_OD_50MHZ      0x0F // CNF = 11, MODE = 11

// GPIO Interrupts macros
#define GPIO_MODE_IT_FT			   0x10
#define GPIO_MODE_IT_RT			   0x11
#define GPIO_MODE_IT_RFT		   0x12
/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_periClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
/*
 * Init and Deinit
 */
void GPIO_init(GPIO_Handle_t *pGPIOHandle);
void GPIO_deinit(GPIO_RegDef_t *pGPIOx);
/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/*
 *  IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t PinNumber);
void GPIO_IRQHandler(uint8_t PinNumber);


#endif /* STM32F103XX_GPIO_DRIVER_H_ */
