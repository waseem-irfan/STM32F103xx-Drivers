/*
 * ledToggle.c
 *
 *  Created on: Oct 9, 2024
 *      Author: Waseem Irfan
 */

#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"

void delay(void){
	for(int i=0 ; i<=600000; i++){

	}
}
int main(void)
{
    /* Loop forever */
	GPIO_Handle_t gpio_led;
	gpio_led.pGPIOx = GPIOC;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP_10MHZ;
	GPIO_periClockControl(GPIOC, ENABLE);
	GPIO_init(&gpio_led);
	while(1){
		GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		delay();
	}

	return 0;
}
