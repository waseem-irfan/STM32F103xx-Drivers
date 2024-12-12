/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Oct 30, 2024
 *      Author: Waseem Irfan
 */
#include "stm32f103xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle);
static void SPI_ApplicationEventCallBack(SPI_Handle_t *pHandle);
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

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t PinNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			// program ISER0 register
			*NVIC_ISER0 = (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <64){
			// program ISER1 register
			*NVIC_ISER1 = (1 << IRQNumber%32);
		}
		else if(IRQNumber >= 64 && IRQNumber <96){
			// program ISER2 register
			*NVIC_ISER2 = (1 << IRQNumber%64);
		}
	}
	else{
		if(IRQNumber <= 31){
			// program ICER0 register
			*NVIC_ICER0 = (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <64){
			// program ICER1 register
			*NVIC_ICER1 = (1 << IRQNumber%32);
		}
		else if(IRQNumber >= 64 && IRQNumber <96){
			// program ICER2 register
			*NVIC_ICER2 = (1 << IRQNumber%64);
		}
	}
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
	// 1. First lets find out the ipr register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount =  (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx ) |= (IRQPriority << shift_amount);
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
	uint8_t temp1, temp2;

	//first check for TXE
	temp1 = pHandle->pSPIx->SPI_SR & (1<<SPI_SR_TXE);
	temp2 = pHandle->pSPIx->SPI_CR2 & (1<<SPI_CR2_TXEIE);
	if(temp1 && temp2){
		//handle txe
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SPI_SR & (1<<SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->SPI_CR2 & (1<<SPI_CR2_RXNEIE);
	if(temp1 & temp2){
		// handle rxne
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for over flag
	temp1 = pHandle->pSPIx->SPI_SR & (1<<SPI_SR_OVR);
	temp2 = pHandle->pSPIx->SPI_CR2 & (1<<SPI_CR2_ERRIE);
	if(temp1 & temp2){
		// handle overrun flag
		spi_ovr_err_interrupt_handle(pHandle);
	}

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

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
	/*
	 * API to Send data with Interrupt Mode
	 * 1. Save the Tx buffer length and address in some global variable.
	 * 2. Mark the SPI state as busy in transmission so that no other
	 * code can take same SPI peripheral until transmission is over.
	 * 3. Enable TXEIE control bit to get interrupt whenever TXE Flag is set in SR
	 *
	 */
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		pSPIHandle->pSPIx->SPI_CR2 |= (1<<SPI_CR2_TXEIE);
	}
	return state;
	//4. Data Transmission will be handled by ISR code. (will implement later)


}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){
	/*
	 * API to Send data with Interrupt Mode
	 * 1. Save the Tx buffer length and address in some global variable.
	 * 2. Mark the SPI state as busy in transmission so that no other
	 * code can take same SPI peripheral until transmission is over.
	 * 3. Enable TXEIE control bit to get interrupt whenever TXE Flag is set in SR
	 *
	 */
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX){
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		pSPIHandle->pSPIx->SPI_CR2 |= (1<<SPI_CR2_RXNEIE);
	}
	return state;
	//4. Data Transmission will be handled by ISR code. (will implement later)


}

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle){
	//2. Check DFF bit in CR1
	if(pHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
		// 16bit frame
		pHandle->pSPIx->SPI_DR = *((uint16_t *)pHandle->pTxBuffer);
		pHandle->TxLen--;
		pHandle->TxLen--;
		(uint16_t *)pHandle->pTxBuffer++;
			}
	else{
		// 8bit frame
		pHandle->pSPIx->SPI_DR = *pHandle->pTxBuffer;
		pHandle->TxLen--;
		pHandle->pTxBuffer++;
	}
	if(!pHandle->TxLen){
		//TxLen is zero, so close the spi transmission and inform the application
		// that TXE is over.

		// this prevents interrupts from setting up of TXE flag
		pHandle->pSPIx->SPI_CR2 &= ~(1<<SPI_CR2_TXEIE);
		pHandle->pTxBuffer = NULL;
		pHandle->TxLen = 0;
		pHandle->TxState = SPI_READY;
		SPI_ApplicationEventCallBack(pHandle,SPI_EVENT_TX_CMPLT);

	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle){
	// do rx as per dff
	//2. Check DFF bit in CR1
	if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
	// 16bit frame
	// Load the data from DR to Rxbuffer,
		*((uint16_t *)pHandle->pRxBuffer) = pHandle->pSPIx->SPI_DR;
		pHandle->RxLen-=2;
		pHandle->pRxBuffer--;
		pHandle->pRxBuffer--;
	}
	else{
		// 8bit frame
		*(pHandle->pRxBuffer) =(uint8_t) pHandle->pSPIx->SPI_DR;
		pHandle->RxLen--;
		pRxBuffer--;
	}

	if(!pHandle->RxLen){
		//reception is complete
		//lets turn off the rxne interrupt
		pHandle->pSPIx->SPI_CR2 &= ~(1<<SPI_CR2_RXNEIE);
		pHandle->pRxBuffer = NULL;
		pHandle->RxLen = 0;
		pHandle->RxState = SPI_READY;
		SPI_ApplicationEventCallBack(pHandle,SPI_EVENT_RX_CMPLT);
	}

}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle){

}
static void SPI_ApplicationEventCallBack(SPI_Handle_t *pHandle){

}
