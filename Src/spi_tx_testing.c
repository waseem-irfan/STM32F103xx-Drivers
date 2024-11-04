/*
 * spi_tx_testing.c
 *
 *  Created on: Oct 31, 2024
 *      Author: Waseem Irfan
 */

/********************** Exercise **********************/
/*
 * 	Test the SPI_SendData API to send the string "Hello World" and use the below Configurations:
 * 	1. SPI2 Master Mode
 * 	2. SCLK Max possible
 * 	3. DFF 0 and DFF 1
 */
#include <string.h>
#include <stm32f103xx_gpio_driver.h>
#include <stm32f103xx_spi_driver.h>

/*	From Alternate functionality Table
 *  PB12 --> SPI2_NSS // No need to configure it also
 *  PB13 --> SPI2_SCK
 *  PB14 --> SPI2_MISO // Don't need to configure for this task
 *  PB15 --> SPI2_MOSI
 *  Alternate function mode : Default
 */

int main(void){
	SPI_Handle_t spi2_tx;
	GPIO_Handle_t spi2_gpio;
	memset(&spi2_gpio, 0, sizeof(spi2_gpio));
	memset(&spi2_tx, 0, sizeof(spi2_tx));
	char data[] = "Hello World";

	/*Configuring SPI2*/

	// Configuring the port and pins & Enabling Clock for PORTB
	spi2_gpio.pGPIOx = GPIOB;
	spi2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_PP_10MHZ;
	// For MOSI2
	spi2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_init(&spi2_gpio);
	// For SCLK2
	spi2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_init(&spi2_gpio);
	// For MISO2
//	spi2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
//	GPIO_init(&spi2_gpio);
	// For NSS
//	spi2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
//	GPIO_init(&spi2_gpio);

	/*NOTE: first of all configure the base address of SPI2*/
	spi2_tx.pSPIx = SPI2;
	// 1. Configure the Master & Bus COnfiguration
	spi2_tx.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	spi2_tx.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	// 2. Configure SCLK2 maximum
	spi2_tx.SPIConfig.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV2;
	// 3. Configure the Data frame
	spi2_tx.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	// 4. Configure the clock polarity
	spi2_tx.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	// 5. Configure the clock phase
	spi2_tx.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	// 6. Configure the SSM
	spi2_tx.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_init(&spi2_tx);
	// NOTE: Remember to enable SPI Peripheral
	SPI_SSIConfig(SPI2, ENABLE); // this makes NSS to high and avoid MODF error

	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2,(uint8_t *)data,strlen(data));
	// before disabling spi confirm that spi is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);
}
