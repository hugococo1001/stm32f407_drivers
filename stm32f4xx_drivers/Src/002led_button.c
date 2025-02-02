/*
 * 002led_butoon.c
 *
 * This program toogle LED4 and LED5 when B1 is pressed
 *
 *  Created on: Dec 9, 2024
 *      Author: ASUS
 */

#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"

#define BTN_PRESSED 1

void delay() {
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void) {

	// set up handle for LED5
	GPIO_Handle_t GPIO_led5;

	GPIO_led5.pGPIOx = GPIOD;
	GPIO_led5.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_led5.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_led5.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_led5.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_led5.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// set up handle for LED4
	GPIO_Handle_t GPIO_led4;

	GPIO_led4.pGPIOx = GPIOD;
	GPIO_led4.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_led4.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_led4.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_led4.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_led4.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// set up handle for button
	GPIO_Handle_t GPIO_button;

	GPIO_button.pGPIOx = GPIOA;
	GPIO_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// enable clocks for GPIO_led4, GPIO_led5 and GPIO_button
	GPIO_PeriClockControl(GPIO_led4.pGPIOx, ENABLE);
	GPIO_PeriClockControl(GPIO_led5.pGPIOx, ENABLE);
	GPIO_PeriClockControl(GPIO_button.pGPIOx, ENABLE);

	// initialize GPIO_led4, GPIO_led5 and GPIO_button
	GPIO_Init(&GPIO_led4);
	GPIO_Init(&GPIO_led5);
	GPIO_Init(&GPIO_button);

	while(1) {
		if (GPIO_ReadFromInputPin(GPIO_button.pGPIOx, GPIO_button.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED) {
			// turn LED on
			delay();
			GPIO_ToggleOutputPin(GPIO_led4.pGPIOx, GPIO_led4.GPIO_PinConfig.GPIO_PinNumber);
			GPIO_ToggleOutputPin(GPIO_led5.pGPIOx, GPIO_led5.GPIO_PinConfig.GPIO_PinNumber);
		}
	}

// use functionality but using state-logic to debounce button
//	while(1) {
//		int current_state = 0;
//		int last_state = 0;
//		uint32_t last_time;
//		uint32_t current_time;
//		int DELAY 20			// milliseconds
//
//		current_time = millis();
//
//		if ((current_state = GPIO_ReadFromInputPin(GPIO_button.pGPIOx, GPIO_button.GPIO_PinConfig.GPIO_PinNumber)) != last_state) {
//			last_time = current_time;
//		}
//
//		if ((current_time - last_time) > DELAY) {
//			if (current_state == 1) {
//				GPIO_WriteToOutputPin(GPIO_led4.pGPIOx,
//									  GPIO_led4.GPIO_PinConfig.GPIO_PinNumber,
//									  GPIO_PIN_SET);
//			} else {
//				GPIO_WriteToOutputPin(GPIO_led4.pGPIOx,
//									  GPIO_led4.GPIO_PinConfig.GPIO_PinNumber,
//									  GPIO_PIN_SET);
//			}
//
//		}
//
//		last_state = current_state;
//	}
}
