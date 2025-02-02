/*
 * 008spi_cmd_handling.c
 *
 *  Created on: Dec 24, 2024
 *      Author: ASUS
 */

/*PB12: SPI2_NSS
 *PB13: SPI2_SCK
 *PB14: SPI2_MISO
 *PB15: SPI2_MOSI
 *Alt Func Mode: 5 */

#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"

extern void initialise_monitor_handles(void);

//command codes
#define COMMAND_LED_CTRL_CODE      		0x50
#define COMMAND_SENSOR_READ_CODE      	0x51
#define COMMAND_LED_READ_CODE      		0x52
#define COMMAND_PRINT_CODE      		0x53
#define COMMAND_ID_READ_CODE      		0x54

#define ACKKNOWLEDGE_CODE				0xF5

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  9

// Initialize the GPIO pins to behave as SPI2 pins (NSS, SCLK, MOSI, MISO)
/******************************************************************8
  * @fn			- SPI2_GPIOInits
  *
  * @brief		- Initialize the GPIO pins used for SPI2 pins
  *
  * @return		- void
  *
  * @Note		- Create a GPIO_Handle_t, initialize all its members except GPIO_PinNumber,
  * 				which is specific to each SPI pin so we do that and call GPIO_Init() for
  * 				each individual GPIO_PinNumber.
*/
void SPI2_GPIOInits(void) {
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;			// SPI alt func
	SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//NSS pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 12;
	GPIO_Init(&SPIPins);

	// SCK pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_Init(&SPIPins);

	// MISO pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 14;
	GPIO_Init(&SPIPins);

	// MOSI pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&SPIPins);
}

// Initialize SPI2 peripheral parameters
/******************************************************************8
  * @fn			- SPI2_Inits
  *
  * @brief		- Initialize SPI2 peripheral parameters
  *
  * @return		- void
  *
  * @Note		- Create a SPI_Handle_t, initialize all its members then call SPI_Init()
*/
void SPI2_Inits(void) {
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;  		// 16MHz/2 = 8MHz
	SPI2handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_PinConfig.SPI_SSM = SPI_SSM_DI;						// hardware mode
	// SPI2handle.SPI_PinConfig.SPI_SSI = SPI_SSI_DI;					// no need to configure SSI in hardware slave management
	SPI2handle.SPI_PinConfig.SPI_SSOE = SPI_SSOE_EN;					// master mode: NSS is kept low until SPE is reset

	SPI_Init(&SPI2handle);
}

// Initialize a GPIOA pin 0 to receive input from a button
/******************************************************************8
  * @fn			- GPIO_ButtonInit
  *
  * @brief		- Initialize a GPIOA pin 0 to receive input from the onboard button B1
  *
  * @return		- void
  *
  * @Note		- void
*/
void GPIO_ButtonInit() {
	// set up handle for button
	GPIO_Handle_t GPIO_button;

	GPIO_button.pGPIOx = GPIOA;
	GPIO_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIO_button);
}

void delay() {
	for(uint32_t i = 0; i < 500000/2; i++);
}


int SPI_VerifyResponse(uint8_t input) {
	if (input == (uint8_t)ACKKNOWLEDGE_CODE) {
		return 1;
	} else {
		return 0;
	}
}

//
int main(void) {
	initialise_monitor_handles();

	printf("Application is running\n");

	// Initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// Initialize the SPI2 peripheral parameters
	SPI2_Inits();

	// Initialize button
	GPIO_ButtonInit();

	printf("SPI Init. done\n");

	// some variables we'll need
	uint8_t dummy_write = 0xff;
	// used to store dummy input when using SPI_ReceiveData() to clear Rx buffer
	uint8_t dummy_read;
	// acknowledge byte
	uint8_t ackbyte;
	uint8_t args[2];

	while (1) {
		// wait for button to be pressed
		while ( ! GPIO_ReadFromInputPin(GPIOA, 0));

		// button debouncing (200ms)
		delay();

		// Enable the SPI peripheral (setting SPB bit in SPI_CR1)
		SPI_PeriControl(SPI2, ENABLE);

		//1. CMD_LED_CTRL  	<pin no(9)>     <value(1)>
		uint8_t command_led_ctrl_code = COMMAND_LED_CTRL_CODE;

		// send command code
		SPI_SendData(SPI2, &command_led_ctrl_code, 1);

		// receive to clear Rx buffer
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send dummy byte to receive ACK/NACK
		SPI_SendData(SPI2, &dummy_write, 1);

		// receive ACK/NACK
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		// if ACK, send argument (ANALOG_PIN0)
		if (SPI_VerifyResponse(ackbyte)) {
			args[0] = LED_PIN;
			args[1] = LED_ON;

			// send argument
			SPI_SendData(SPI2, args, 2);

			// dummy read
			SPI_ReceiveData(SPI2, &dummy_read, 2);

			printf("COMMAND_LED_CTRL Executed\n");
		}
		//end of COMMAND_LED_CTRL

		// wait for button to be pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, 0));

		// button debouncing
		delay();

		//2. CMD_SENOSR_READ   <analog pin number(A0)>
		uint8_t command_sensor_read_code = COMMAND_SENSOR_READ_CODE;

		// send command code
		SPI_SendData(SPI2, &command_sensor_read_code, 1);

		// dummy read
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// dummy write to initiate ACK/NACK
		SPI_SendData(SPI2, &dummy_write, 1);

		// read ACK/NACK
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		// If ACK, send argument
		if (SPI_VerifyResponse(ackbyte)) {
			// send argument
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);

			// dummy read
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert some delay so that slave can ready with the data
			// while pin does ADC
			delay();

			// dummy write
			SPI_SendData(SPI2, &dummy_write, 1);

			// receive analog value
			uint8_t analog_read_val;
			SPI_ReceiveData(SPI2, &analog_read_val, 1);
			printf("COMMAND_SENSOR_READ %u\n", analog_read_val);

			printf("CMD_SENOSR_READ Executed\n");
		}
		//end of CMD_SENOSR_READ

		// wait for button to be pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, 0));

		// button debouncing
		delay();

		//3. CMD_LED_READ <pin no(1)>
		uint8_t command_led_read_code = COMMAND_LED_READ_CODE;

		// send command code
		SPI_SendData(SPI2, &command_led_read_code, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// get ackbyte
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);


		if (SPI_VerifyResponse(ackbyte)) {
			// send argument
			args[0] = LED_PIN;
			SPI_SendData(SPI2, args, 1);
			SPI_ReceiveData(SPI2, &dummy_read, 1);  	// to clear off RXNE

			//insert some delay so that slave can ready with the data
			delay();

			// receive led pin state
			uint8_t led_status;
			SPI_SendData(SPI2, &dummy_write, 1);
			SPI_ReceiveData(SPI2, &led_status, 1);
			printf("CMD_LED_READ %u\n", led_status);
		}
		//end of CMD_LED_READ

		// wait for button to be pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, 0));

		// button debouncing (about 200ms)
		delay();

		//4. CMD_PRINT 		<len>  <message>
		uint8_t command_print_code = COMMAND_PRINT_CODE;

		// send command code
		SPI_SendData(SPI2, &command_print_code, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// receive ackbyte
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)) {
			// send message length argument
			uint8_t message[] = "Merry Christmas!";		// notice uint8_t is used instead of char
			uint8_t len = strlen((char*)message);

			args[0] = len;

			SPI_SendData(SPI2, args, 1);
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// send message argument
			for(int i = 0; i < len; i++) {
				SPI_SendData(SPI2, &message[i], 1);
				SPI_ReceiveData(SPI2, &dummy_read, 1);
			}
		}
		printf("COMMAND_PRINT Executed \n");
		//end of CMD_PRINT

		// wait for button to be pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, 0));

		// button debouncing
		delay();

		//5. CMD_ID_READ
		uint8_t command_id_read_code = COMMAND_ID_READ_CODE;

		// send command code
		SPI_SendData(SPI2, &command_id_read_code, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// receive ackbyte;
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)) {
			// receive id string (10 chars)
			uint8_t id_len = 10;
			uint8_t id[11];

			for (int i = 0; i < id_len; i++) {
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}

			// add null_terminator
			id[10] = '\0';

			printf("CMD_ID_READ %s\n", id);
		}
		//end of CMD_ID_READ

		// wait until all data is transmitted
		while (SPI_GetFlagStatus(SPI2, SPI_SR_BSY));

		// Disable the SPI peripheral (setting SPB bit in SPI_CR1)
		SPI_PeriControl(SPI2, DISABLE);

		printf("SPI Communication Closed\n");
	}

	return 0;
}

