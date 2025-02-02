/*
 * 003led_button_ext.c
 *
 * This program is pretty the same as 002led_button.c but it lighs up
 * an external LED and uses an external PU/PD resistor.
 *
 *  Created on: Dec 12, 2024
 *      Author: ASUS
 */


#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"

#define BTN_PRESSED 1

void delay() {
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void) {
	// set up handle for led
	GPIO_Handle_t GPIO_led;

	GPIO_led.pGPIOx = GPIOA;
	GPIO_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// set up handle for button
	GPIO_Handle_t GPIO_button;

	GPIO_button.pGPIOx = GPIOB;
	GPIO_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // external PD resistor is used

	// enable clocks for GPIO_led and GPIO_button
	GPIO_PeriClockControl(GPIO_led.pGPIOx, ENABLE);
	GPIO_PeriClockControl(GPIO_button.pGPIOx, ENABLE);

	// initialize GPIO_led and GPIO_button
	GPIO_Init(&GPIO_led);
	GPIO_Init(&GPIO_button);

	while(1) {
		if (GPIO_ReadFromInputPin(GPIO_button.pGPIOx, GPIO_button.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED) {
			// turn LED on
			delay();
			GPIO_ToggleOutputPin(GPIO_led.pGPIOx, GPIO_led.GPIO_PinConfig.GPIO_PinNumber);
		}
	}
}

