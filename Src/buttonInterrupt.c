/*
 * buttonInterrupt.c
 *
 *  Created on: Oct 10, 2024
 *      Author: Waseem Irfan
 */
#include <string.h>
#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"

void delay(void){
	for(uint32_t i=0; i<=500000/2 ; i++);
}

int main(void){
	GPIO_Handle_t gpio_led, gpio_btn;
	memset(&gpio_led,0,sizeof(gpio_led));
	memset(&gpio_btn,0,sizeof(gpio_btn));	// Important tip: Initialize structures with 0 before using them
	// Configure the led
	gpio_led.pGPIOx = GPIOC;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP_50MHZ;
	GPIO_periClockControl(GPIOC, ENABLE);
	GPIO_init(&gpio_led);

	// Configure the button
	gpio_btn.pGPIOx = GPIOA;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_periClockControl(GPIOA, ENABLE);
	GPIO_init(&gpio_btn);

	// IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	while(1);

}

void EXTI9_5_IRQHandler(void){
	delay();
	GPIO_IRQHandler(GPIO_PIN_5);
	GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
