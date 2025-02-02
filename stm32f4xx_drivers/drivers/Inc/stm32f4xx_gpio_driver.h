/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: Dec 6, 2024
 *      Author: ASUS
 */

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_

#include <stm32f4xx.h>

/********refer to these macros to configure GPIO peripheral*********************/

// @GPIO_PIN_NUMBERS
#define	GPIO_PIN_NO_0		0
#define	GPIO_PIN_NO_1		1
#define	GPIO_PIN_NO_2		2
#define	GPIO_PIN_NO_3		3
#define	GPIO_PIN_NO_4		4
#define	GPIO_PIN_NO_5		5
#define	GPIO_PIN_NO_6		6
#define	GPIO_PIN_NO_7		7
#define	GPIO_PIN_NO_8		8
#define	GPIO_PIN_NO_9		9
#define	GPIO_PIN_NO_10		10
#define	GPIO_PIN_NO_11		11
#define	GPIO_PIN_NO_12		12
#define	GPIO_PIN_NO_13		13
#define	GPIO_PIN_NO_14		14
#define	GPIO_PIN_NO_15		15

// @GPIO_PIN_MODES
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT 	4
#define GPIO_MODE_IT_RT 	5
#define GPIO_MODE_IT_RFT 	6

//@GPIO_PIN_SPEEDS
#define	GPIO_SPEED_LOW		0
#define	GPIO_SPEED_MEDIUM	1
#define	GPIO_SPEED_FAST		2
#define	GPIO_SPEED_HIGH		3

// @GPIO_PIN_PUPD
#define	GPIO_NO_PUPD		0
#define	GPIO_PIN_PU			1
#define	GPIO_PIN_PD			2

// @GPIO_PIN_OUT_TYPE
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

// This struct contains all settings users want to implement
typedef struct {
	uint8_t GPIO_PinNumber;					// possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;					// possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;					// possible values from @GPIO_PIN_SPEEDS
	uint8_t GPIO_PinPuPdControl;			// possible values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;					// possible values from @GPIO_PIN_OUT_TYPE
	uint8_t GPIO_PinAltFuncMode;
} GPIO_PinConfig_t;

// Handle structure for a GPIO pin
typedef struct {
	GPIO_RegDef_t *pGPIOx;					// Holds the base address of the GPIO
											// port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;		// Holds GPIO pin config settings
} GPIO_Handle_t;

/*
***********APIs support by GPIO driver******************************
*/

// GPIO peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *GPIOx, uint8_t EnorDi);

// GPIO init/de-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *GPIOx);

// GPIO data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *GPIOx);		// return content of input data register
void GPIO_WriteToOutputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *GPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber);

// GPIO IRQ config and ISR handling
void GPIO_IRQ_Number_Config(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F4XX_GPIO_DRIVER_H_ */
