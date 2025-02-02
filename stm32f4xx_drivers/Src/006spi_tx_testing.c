/*
 * spi_tx_testing.c
 *
 *  Created on: Dec 20, 2024
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

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//NSS pin
	// SPIPins.GPIO_PinConfig.GPIO_PinNumber = 12;
	// GPIO_Init(&SPIPins);

	// SCK pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_Init(&SPIPins);

	// MISO pin
	// SPIPins.GPIO_PinConfig.GPIO_PinNumber = 14;
	// GPIO_Init(&SPIPins);

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
	SPI2handle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV16;  		// 16MHz/2 = 8MHz
	SPI2handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_PinConfig.SPI_SSM = SPI_SSM_EN;						// software mode
	SPI2handle.SPI_PinConfig.SPI_SSI = SPI_SSI_EN;						// set to avoid errors from using alt func
	// SSOE is irrelevant in software slave management

	SPI_Init(&SPI2handle);
}

int main(void) {
	// char user_data[] = "Hello World";
	uint8_t num = 8;

	// Initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// Initialize the SPI2 peripheral parameters
	SPI2_Inits();

	// Enable the SPI peripheral (setting SPB bit in SPI_CR1)
	SPI_PeriControl(SPI2, ENABLE);

	// SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));
	SPI_SendData(SPI2, &num, 1);

	//lets confirm SPI is not busy
	while( SPI_GetFlagStatus(SPI2, SPI_SR_BSY));

	// Disable SPI
	SPI_PeriControl(SPI2, DISABLE);

	return 0;
}
