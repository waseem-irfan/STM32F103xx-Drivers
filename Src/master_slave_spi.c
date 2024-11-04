/*
 * master_slave_spi.c
 *
 *  Created on: Nov 2, 2024
 *      Author: Waseem Irfan
 */

/**************************** Exercise ***************************/

/*
 *  ======= SPI Master(STM32) Slave(Arduino) Communication =======
 *  When the button on the Master is pressed, Master should send string of data to the Arduino slave connected
 *  The data received by the Arduino will be displayed on the arduino serial port monitor
 *
 *  1. Use SPI Full Duplex mode
 *  2. ST board will be in SPI Master and Arduino will be configured as SPI slave mode
 *  3. Use DFF = 0
 *  4. Use Hardware slave management (SSM = 0)
 *  5. clock speed = 2MHz , fclk = 16MHz
 *
 *  In this exercise master is not going to receive anything from the slave. So you may not need to configure MISO pin
 *
 *  NOTE: Slave doesn't know how many bytes of data master is going to send. SO first master will send the number of bytes Info
 *  which slave is going to receive next.
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
void SPI_BtnConfig(void){
	GPIO_Handle_t gpio_btn;
	gpio_btn.pGPIOx = GPIOA;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_periClockControl(GPIOA, ENABLE);
	GPIO_init(&gpio_btn);
}
void delay(void){
	for(int i=0 ; i<=600000; i++){

	}
}
int main(void){
	SPI_Handle_t spi2_tx;
	GPIO_Handle_t spi2_gpio;
	memset(&spi2_gpio, 0, sizeof(spi2_gpio));
	memset(&spi2_tx, 0, sizeof(spi2_tx));
	char data[] = "Hello World";
	/*Configuring the button*/
	SPI_BtnConfig();
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
	spi2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_init(&spi2_gpio);

	/*NOTE: first of all configure the base address of SPI2*/
	spi2_tx.pSPIx = SPI2;
	// 1. Configure the Master & Bus COnfiguration
	spi2_tx.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	spi2_tx.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	// 2. Configure SCLK2 maximum
	spi2_tx.SPIConfig.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV8;
	// 3. Configure the Data frame
	spi2_tx.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	// 4. Configure the clock polarity
	spi2_tx.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	// 5. Configure the clock phase
	spi2_tx.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	// 6. Configure the SSM
	spi2_tx.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_init(&spi2_tx);
	// NOTE: Remember to enable SPI Peripheral
	/*
	 * Making SSOE 1 does NSS output ENABLE
	 * The NSS pin is automatically managed by the hardware
	 * i.e. when SPE = 1, NSS will pulled to low
	 * and NSS will pulled to high when SPE = 0`
	 */
	SPI_SSOEConfig(SPI2, ENABLE);
	while(1){
		// wait to press the button
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_5));
		delay();

		SPI_PeripheralControl(SPI2, ENABLE);
		//first send length of data
		uint8_t DataLen = strlen(data);
		SPI_SendData(SPI2,&DataLen,1);
		SPI_SendData(SPI2,(uint8_t *)data,strlen(data));
		// before disabling spi confirm that spi is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
		SPI_PeripheralControl(SPI2, DISABLE);
	}
	return 0;
}
