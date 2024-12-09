/*
 * SPI_comnd&Response_Com.c
 *
 *  Created on: Dec 6, 2024
 *      Author: Waseem Irfan
 */

/**************************** Exercise ***************************/

/*
 *  ======= SPI Master(STM32) Slave(Arduino)command & Response base Communication =======
 * When the button on master is pressed, master sends a command to the slave and slave respond
 * as per command implementation
 *
 * 1. Use SPI full duplex mode
 * 2. ST board will be in SPI master mode and Arduino will be configured as SPI slave mode
 * 3. Use DFF=0
 * 4. Use hardware slave management (SSM = 0)
 * 5. SCLK speed = 2MHz, fclk = 16MHz
 *
 * => Arduino Sketch : 002SPISlaveCmdHandling.ino
 *
 */
#include <stdio.h>
#include <string.h>
#include <stm32f103xx_gpio_driver.h>
#include <stm32f103xx_spi_driver.h>

//command codes --> slave recognizes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON		1
#define LED_OFF		0

// arduino analog pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

// arduino led

#define LED_PIN 9

void delay(void){
	for(int i=0 ; i<=500000; i++){

	}
}
 /*	From Alternate functionality Table
  *  PB12 --> SPI2_NSS // No need to configure it also
  *  PB13 --> SPI2_SCK
  *  PB14 --> SPI2_MISO // Don't need to configure for this task
  *  PB15 --> SPI2_MOSI
  *  Alternate function mode : Default
  */

void SPI_GPIOInit(void){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_PP_10MHZ;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_init(&SPIPins);
}

void SPI2_Init(void){
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SClkSpeed =SPI_SCLK_SPEED_DIV32;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI_init(&SPI2handle);
}

void GPIO_ButtonInit(void){
	GPIO_Handle_t gpio_btn, gpio_led;
	gpio_btn.pGPIOx = GPIOA;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_periClockControl(GPIOA, ENABLE);
	GPIO_init(&gpio_btn);

	gpio_led.pGPIOx = GPIOC;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP_10MHZ;
	GPIO_periClockControl(GPIOC, ENABLE);
	GPIO_init(&gpio_led);
}

uint8_t SPI_verifyResponse(uint8_t ack){
	if(ack==0xF5){
		//ack
		return 1;
	}
	else{
		//nack
		return 0;
	}
}

int main(void){
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;
	uint8_t ackbyte;
	uint8_t arg[2];
	printf("Application is running\n");

	GPIO_ButtonInit();

	SPI2_Init();
	while(1){
		// wait till the button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));
		//To Avoid de-bouncing issue
		delay();
		//enable spi2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		//1. CMD_LED_CTRL		<pin no(1)>			<value(1)>

		// send led command to arduino to check if it supports control command or not.
		uint8_t comndcode = COMMAND_LED_CTRL;
		SPI_SendData(SPI2, &comndcode, 1);
		/*
		 * NOTE: This transmission of 1 byte resulted 1 garbage byte collection in Rx buffer
		 * of the master and RXNE flag is set. So do the dummy read and clear the flag.
		 */
		/* Remember: In SPI communication when master or slave sends 1 byte,
		 * it also receives 1 byte in return.
		 */
		// so dummy read to clear the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send some dummy bits(1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// receive the ack from the slave
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_verifyResponse(ackbyte)){
			// send args.
			arg[0] = LED_PIN;
			arg[1] = LED_ON;

			SPI_SendData(SPI2, arg, 2);
		}

		// end of COMMAND_LED_CTRL

		//2. CMD_SENSOR_READ		<analog pin no(1)>
		// wait till the button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));
		delay();
		comndcode = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &comndcode, 1);
		// so dummy read to clear the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send some dummy bits(1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// receive the ack from the slave
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_verifyResponse(ackbyte)){
			// send args.
			arg[0] = ANALOG_PIN0;

			SPI_SendData(SPI2, arg, 1);

			// so dummy read to clear the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);
			//insert some delay so that slave will be ready with data
			delay();
			// send some dummy bits(1 byte) to fetch the analog read value
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}


		//lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//Disable the SPI2 peripheral
		SPI_periClockControl(SPI2, DISABLE);
	}
	return 0;
}
