/*
 * 009spi_message_receive_interrupt.c
 *
 *  Created on: Jan 14, 2025
 *      Author: ASUS
 */

/*
 * This application receives and prints the user message received from the Arduino peripheral in SPI interrupt mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SWV ITM data console
 */
/*
 * Note : Follow the instructions to test this code
 * 1. Download this code on to STM32 board , acts as Master
 * 2. Download Slave code (003SPISlaveUartReadOverSPI.ino) on to Arduino board (Slave)
 * 3. Reset both the boards
 * 4. Enable SWV ITM data console to see the message
 * 5. Open Arduino IDE serial monitor tool
 * 6. Type anything and send the message (Make sure that in the serial monitor tool line ending set to carriage return)
 */

#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"

#define MAX_LEN		500

// some necessary variables
SPI_Handle_t SPI2_Handle;
char RcvBuff[MAX_LEN];
volatile char ReadByte;
volatile uint8_t rcvStop = 0;
volatile uint8_t dataAvailable = 0;

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */
void SPI_GPIOs_Init(void) {
	GPIO_Handle_t SPI_pins;

	SPI_pins.pGPIOx = GPIOB;
	SPI_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	SPI_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_pins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;

	// SCLK
	SPI_pins.GPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_Init(&SPI_pins);

	// NSS
	SPI_pins.GPIO_PinConfig.GPIO_PinNumber = 12;
	GPIO_Init(&SPI_pins);

	// MOSI
	SPI_pins.GPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&SPI_pins);

	// MISO
	SPI_pins.GPIO_PinConfig.GPIO_PinNumber = 14;
	GPIO_Init(&SPI_pins);
}

void SPI2_Init(void) {
	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_Handle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2_Handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2_Handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Handle.SPI_PinConfig.SPI_SSM = SPI_SSM_DI;		// Hardware slave management
	// SS output is enabled in master mode and when the cell is enabled.
	// The cell cannot work in a multi-master environment.
	SPI2_Handle.SPI_PinConfig.SPI_SSOE = SPI_SSOE_EN;

	SPI_Init(&SPI2_Handle);
}

void GPIO_interrupt_pin_Init(uint8_t EnorDi) {
	GPIO_Handle_t GPIO_interrupt_pin;

	// Setting "GPIO_interrupt_pin" to 0 by default to prevent
	// garbage values in untouched registers i.e. GPIOx_OTYPER & GPIOx_AFRL
	memset(&GPIO_interrupt_pin, 0, sizeof(GPIO_interrupt_pin));

	GPIO_interrupt_pin.pGPIOx = GPIOD;
	GPIO_interrupt_pin.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_NO_6;
	GPIO_interrupt_pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIO_interrupt_pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&GPIO_interrupt_pin);

	if (EnorDi == ENABLE) {
		GPIO_IRQ_Number_Config(IRQ_NO_EXTI9_5, ENABLE);
	} else {
		GPIO_IRQ_Number_Config(IRQ_NO_EXTI9_5, DISABLE);
	}

	GPIO_IRQ_Priority_Config(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15); // why 15?
}

int main(void) {
	uint8_t dummy = 0xff;

	// Enable GPIO pins for SPI communication
	SPI_GPIOs_Init();

	// Enable SPI2 parameters
	SPI2_Init();

	// Enable GPIO interrupt pin PD6
	GPIO_interrupt_pin_Init(ENABLE);

	while(1) {
		rcvStop = 0;

		// wait until data is available
		while(!dataAvailable);

		// Now data is available
		// Stop interrupts from PD6 to allow for interrupts for SPI2
		GPIO_interrupt_pin_Init(DISABLE);

		SPI_PeriControl(SPI2_Handle.pSPIx, ENABLE);

		// all communication is in this loop
		// continue sending/receiving until all data are received,
		// at which point rcvStop is 1;
		while(!rcvStop) {
			while(SPI_SendDataIT(&SPI2_Handle, &dummy, 1) == SPI_BUSY_IN_TX);
			while(SPI_ReceiveDataIT(&SPI2_Handle, (uint8_t*)&ReadByte, 1) == SPI_BUSY_IN_RX);	// would say safe to discard 'volatile'...
		}

		/************CLOSING COMMUNICATION************/
		// wait until communication is over
		while(SPI_GetFlagStatus(SPI2_Handle.pSPIx, SPI_SR_BSY));

		SPI_PeriControl(SPI2_Handle.pSPIx, DISABLE);

		printf("Received data = %s\n", RcvBuff);

		dataAvailable = 0;

		GPIO_interrupt_pin_Init(ENABLE);
	}

	return 0;
}

void SPI2_IRQHandler() {
	SPI2_IRQHandling(&SPI2_Handle);
}


void EXTI9_5_IRQHandler() {
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t SPI_event) {
	static uint8_t i = 0;

	if (SPI_event == SPI_EVENT_RX_CMPLT) {
		RcvBuff[i++] = ReadByte;

		if (ReadByte == '\0' || i == MAX_LEN) {
			RcvBuff[i - 1] = '\0';
			rcvStop = 1;	// '\0' indicated end of message (rcvStop == 1)
			i = 0;			// reset i for later communication
		}
	}
}

