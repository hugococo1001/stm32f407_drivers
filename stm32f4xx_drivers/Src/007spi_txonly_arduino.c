/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: Dec 22, 2024
 *      Author: ASUS
 */

/*PB12: SPI2_NSS
 *PB13: SPI2_SCK
 *PB14: SPI2_MISO
 *PB15: SPI2_MOSI
 *Alt Func Mode: 5 */

#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"

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

	// initialize the structure and its members to avoid garbage values
	memset(&SPIPins,0,sizeof(SPIPins));

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
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

	// initialize the structure and its members to avoid garbage values
	memset(&SPI2handle,0,sizeof(SPI2handle));

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;  		// 16MHz/2 = 8MHz
	SPI2handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_PinConfig.SPI_SSM = SPI_SSM_DI;						// hardware mode
	// SPI2handle.SPI_PinConfig.SPI_SSI = SPI_SSI_DI;					// no need to configure SSI in hardware slave management
	SPI2handle.SPI_PinConfig.SPI_SSOE = SPI_SSOE_EN;					// master mode: NSS is kept low until SPE is reset

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit() {
	// set up handle for button
	GPIO_Handle_t GPIO_button;

	// initialize the structure and its members to avoid garbage values
	memset(&GPIO_button,0,sizeof(GPIO_button));

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

//
int main(void) {
	char user_data[] = "happy";

	// Initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// Initialize the SPI2 peripheral parameters
	SPI2_Inits();

	// Initialize button
	GPIO_ButtonInit();

	while (1) {
		while ( ! GPIO_ReadFromInputPin(GPIOA, 0));

		// button debouncing
		delay();

		// Enable the SPI peripheral (setting SPB bit in SPI_CR1)
		SPI_PeriControl(SPI2, ENABLE);

		// Send data length
		uint8_t data_len = strlen(user_data);
		SPI_SendData(SPI2, &data_len, 1);

		// Send data
		SPI_SendData(SPI2, (uint8_t*)user_data, (uint32_t)data_len);

		// wait until all data is transmitted
		while (SPI_GetFlagStatus(SPI2, SPI_SR_BSY));

		// Disable the SPI peripheral (setting SPB bit in SPI_CR1)
		SPI_PeriControl(SPI2, DISABLE);
	}

	return 0;
}
