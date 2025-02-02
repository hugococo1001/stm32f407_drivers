/*
 * 005button_interrupt.c
 *
 *  Created on: Dec 14, 2024
 *      Author: ASUS
 */

#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"

void delay() {
	for(uint32_t i = 0; i < 500000/2; i++);
}

void EXTI9_5_IRQHandler(void) {
	delay();
	GPIO_IRQHandling(5);
	GPIO_ToggleOutputPin(GPIOD, 12);
}

int main(void) {
	// set up handle for led
	GPIO_Handle_t GPIO_led;
	memset(&GPIO_led, 0, sizeof(GPIO_led));

	GPIO_led.pGPIOx = GPIOD;
	GPIO_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// set up handle for button
	GPIO_Handle_t GPIO_button;
	memset(&GPIO_button, 0, sizeof(GPIO_button));

	GPIO_button.pGPIOx = GPIOD;
	GPIO_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // external PD resistor is used

	// enable clocks for GPIO_led and GPIO_button
	GPIO_PeriClockControl(GPIO_led.pGPIOx, ENABLE);
	GPIO_PeriClockControl(GPIO_button.pGPIOx, ENABLE);

	// initialize GPIO_led and GPIO_button
	GPIO_Init(&GPIO_led);
	GPIO_Init(&GPIO_button);

	// IRQ config
	GPIO_IRQ_Number_Config(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQ_Priority_Config(IRQ_NO_EXTI9_5, 15);

	while(1);

	return 0;
}
