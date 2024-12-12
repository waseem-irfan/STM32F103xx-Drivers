/*
 * stm32f103xx_spi_driver.h
 *
 *  Created on: Oct 30, 2024
 *      Author: Waseem Irfan
 */

#ifndef INC_STM32F103XX_SPI_DRIVER_H_
#define INC_STM32F103XX_SPI_DRIVER_H_

#include "stm32f103xx.h"
/*
 * Configuration Structure for SPIx peripheral
 */

typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t	SPI_SClkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */

typedef struct{
	SPI_RegDef_t *pSPIx;	/*<This holds the base address of SPIx(x=1,2,3) peripheral>*/
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;		/*<To store the Tx Buffer Address>*/
	uint8_t *pRxBuffer;		/*<To store the Rx Buffer Address>*/
	uint32_t RxLen;			/*<To store the Rx Buffer Length>*/
	uint32_t TxLen;			/*<To store the Tx Buffer Length>*/
	uint8_t RxState;		/*<To store the Rx Buffer State>*/
	uint8_t TxState;		/*<To store the Tx Buffer State>*/
}SPI_Handle_t;

/*
 *	 @ SPI_DeviceMode
 */

#define SPI_DEVICE_MODE_MASTER 	1
#define SPI_DEVICE_MODE_SLAVE	0

/*
 * 	@ SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * 	@ SPI_SClkSpeed
 */
#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6
#define SPI_SCLK_SPEED_DIV256	7

/*
 * 	@SPI_DFF
 */
#define SPI_DFF_8BITS			0
#define SPI_DFF_16BITS			1

/*
 *	@SPI_CPOL
 */

#define SPI_CPOL_HIGH 			1
#define SPI_CPOL_LOW			0

/*
 *	@SPI_CPHA
 */
#define SPI_CPHA_HIGH 			1
#define SPI_CPHA_LOW			0

/*
 * 	@ SPI_SSM
 */
#define SPI_SSM_EN				1
#define SPI_SSM_DI				0


// CR1 BitFields macros
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL 			1
#define SPI_CR1_MSTR 			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define	SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

// CR2 BitFields macros
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

// SR BitFields macros
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF 			5
#define SPI_SR_OVR  			6
#define SPI_SR_BSY  			7

/*
 * SPI related flags
 */
#define SPI_TXE_FLAG			(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG			(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG			(1 << SPI_SR_BSY)

/*
 * SPI Application states
 */

#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 * possible SPI Application Events
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_periClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and Deinit
 */
void SPI_init(SPI_Handle_t *pSPIHandle);
void SPI_deinit(SPI_RegDef_t *pSPIx);

/*
 * Data Send & Data Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t PinNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandler(SPI_Handle_t *pHandle);

/*
 * Other peripheral Control APIs
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);


#endif /* INC_STM32F103XX_SPI_DRIVER_H_ */
