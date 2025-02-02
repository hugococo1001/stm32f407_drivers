/*
 * stm32f4xx_i2c_driver.h
 *
 *  Created on: Jan 21, 2025
 *      Author: ASUS
 */

#ifndef INC_STM32F4XX_I2C_DRIVER_H_
#define INC_STM32F4XX_I2C_DRIVER_H_

#include "stm32f4xx.h"

// Configuration structure for I2Cx peripheral
typedef struct {
	uint32_t SCLKSpeed;
	uint32_t AddressMode;
	uint32_t DeviceAddress;
	uint32_t ACKControl;
	uint32_t FMDutyCycle;
}I2C_PinConfig_t;

// Handle structure for I2Cx peripheral
typedef struct {
	I2C_RegDef_t 	*pI2Cx;
	I2C_PinConfig_t	I2C_PinConfig;
}I2C_Handle_t;

/***************refer to these macros to configure I2C peripheral*********************/
//@I2CMasterMode

//@SCLKSpeed
#define I2C_SCLKSPEED_SM	100000
#define I2C_SCLKSPEED_FM	400000

// @AddressMode
#define I2C_7_BIT_ADDRESS_MODE		0
#define I2C_10_BIT_ADDRESS_MODE		1

//@ACKControl
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

//@FMDutyCycle
#define	I2C_FM_DUTY_CYCLE_2		0
#define I2C_FM_DUTY_CYCLE_16_9	1

// Max rise time (in ns)
#define SM_MAX_RISE_TIME	1000
#define FM_MAX_RISE_TIME	300

/***************************API Function Prototypes****************************************/
void delay();
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName);
void I2C_PeriControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_Init(I2C_Handle_t* pI2CHandle);
void I2C_DeInit(I2C_Handle_t *pI2CHandle);

// Blocking Send & Receive functions

// Interrupt-based Send & Receive functions
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t I2C_event);

// I2C interrupt config functions
void I2C_IRQ_Number_Config(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority);

#endif /* INC_STM32F4XX_I2C_DRIVER_H_ */
