/*
 * 0001led_toggle.c
 *
 *  Created on: Dec 9, 2024
 *      Author: ASUS
 */
#include <stdint.h>

#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"

void delay() {
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void) {
	// Create a GPIOD handle so we can use it to initialize GPIOD
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	while(1) {
		GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
		delay();
	}

	return 0;
}


