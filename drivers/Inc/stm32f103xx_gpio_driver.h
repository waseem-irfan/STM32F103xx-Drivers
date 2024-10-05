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
	uint8_t	GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t	GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_OPType;
	uint8_t	GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx;		/* This holds the base address of GPIO port to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

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
