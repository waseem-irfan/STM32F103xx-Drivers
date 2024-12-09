/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Oct 30, 2024
 *      Author: Waseem Irfan
 */
#include "stm32f103xx_spi_driver.h"



/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_periClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
			if(pSPIx == SPI1){
				SPI1_PCLK_EN();
			}
			else if(pSPIx == SPI2){
				SPI2_PCLK_EN();
			}
			else if(pSPIx == SPI3){
				SPI3_PCLK_EN();
			}
		}
		else{
			if(pSPIx == SPI1){
				SPI1_PCLK_DI();
			}
			else if(pSPIx == SPI2){
				SPI2_PCLK_DI();
			}
			else if(pSPIx == SPI3){
				SPI3_PCLK_DI();
			}
		}
}


/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             -
 *
 * @param[in]         - Address of configured SPI
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_init(SPI_Handle_t *pSPIHandle){
	// peripheral clock enable
	SPI_periClockControl(pSPIHandle->pSPIx, ENABLE);
	// configure the SPI_CR1 register
	uint32_t tempreg=0;

	//1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//2. Configure the BusConfig
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// BIDIMODE will 0
		tempreg &= ~(1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// BIDIMODE will be 1
		tempreg |= (1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//1. BIDIMODE will be 0
		tempreg &= ~(1 << 15);
		//2. RXONLY will be 1
		tempreg |= (1 << 10);
	}

	//3. Configure the speed of serial clock (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SClkSpeed << SPI_CR1_BR;
	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
	//5. Configure the clock polarity
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	//6. Configure the clock phase
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	//7. Configuring Slave Select Management
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->SPI_CR1 = tempreg;
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void SPI_deinit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0){
		//1. wait for TXE flag to SET
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
		//2. Check DFF bit in CR1
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
			// 16bit frame
			pSPIx->SPI_DR = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}
		else{
			// 8bit frame
			pSPIx->SPI_DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	while(Len > 0){
			//1. wait for RXNE flag to SET
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
			//2. Check DFF bit in CR1
			if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
				// 16bit frame
				// Load the data from DR to Rxbuffer,
				*((uint16_t *)pRxBuffer) = pSPIx->SPI_DR;
				Len--;
				Len--;
				(uint16_t *)pRxBuffer++;
			}
			else{
				// 8bit frame
				*pRxBuffer = pSPIx->SPI_DR;
				Len--;
				pRxBuffer++;
			}
		}
}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t PinNumber){

}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */


void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void SPI_IRQHandler(SPI_Handle_t *pHandle){

}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SPI_SR & FlagName){
		return FLAG_SET;
	}
	else{
		return FLAG_RESET;
	}
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->SPI_CR1 |= (1<< SPI_CR1_SPE);
	}
	else{
		pSPIx->SPI_CR1 &= ~(1<< SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
	}
	else{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
	}
	else{
		pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

