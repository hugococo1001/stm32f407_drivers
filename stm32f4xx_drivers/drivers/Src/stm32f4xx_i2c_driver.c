/*
 * stm32f4xx_i2c_driver.c
 *
 *  Created on: Jan 21, 2025
 *      Author: ASUS
 */

#include "stm32f4xx_i2c_driver.h"

uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB_Prescaler[4] = {2, 4, 8, 16};

uint32_t RCC_GetPLLFreq(void) {
	// void
}

uint32_t RCC_GetPCLK1Value(void) {
	// bit 2 and 3 are System clock switch status
	uint8_t clksrc = (RCC->CFGR >> 2) & 0b11;
	uint32_t SystemClkFreq;

	// find system clock
	if (clksrc == 0b00) {
		// clksrc is HSI
		SystemClkFreq = 16000000;
	} else if (clksrc == 0b01) {
		// clksrc is HSE
		SystemClkFreq = 8000000;
	} else {
		// clksrc is PLL
		// will not be used here
		SystemClkFreq = RCC_GetPLLFreq();
	}

	// find HCLK = SystemClkFreq / HPRE
	uint8_t HPRE_bits = (RCC->CFGR >> 4) & 0b1111;
	uint32_t HPRE;

	if (HPRE_bits < 8) {
		HPRE = 1;
	} else {
		HPRE = AHB_Prescaler[HPRE_bits - 8];
	}

	uint32_t AHBClkFreq = SystemClkFreq / HPRE;

	// find PCLK1 = SystemClkFreq / PPRE1
	uint8_t PPRE1_bits = (RCC->CFGR >> 10) & 0b111;
	uint32_t PPRE1;

	if (PPRE1_bits < 4) {
		PPRE1 = 1;
	} else {
		PPRE1 = APB_Prescaler[PPRE1_bits - 4];
	}

	uint32_t APB1ClkFreq = AHBClkFreq / PPRE1;

	return APB1ClkFreq;
}

void I2C_PeriControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pI2Cx->CR1 |= (ENABLE << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 |= (DISABLE << I2C_CR1_PE);
	}

}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_CLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_CLK_EN();
		} else {
			I2C3_CLK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_CLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_CLK_DI();
		} else {
			I2C3_CLK_DI();
	}
}

// memset struct to 0 in application
void I2C_Init(I2C_Handle_t* pI2CHandle) {
	// 1. Enable ACK
	if (pI2CHandle->I2C_PinConfig.ACKControl == ENABLE) {
		pI2CHandle->pI2Cx->CR1 |= (ENABLE << I2C_CR1_ACK);
	} else {
		pI2CHandle->pI2Cx->CR1 &= ~(ENABLE << I2C_CR1_ACK);
	}

	// 2. Input frequency in filled up with APB1 clock
	pI2CHandle->pI2Cx->CR2 |= ((RCC_GetPCLK1Value() / 1000000U) & 0b111111) << I2C_CR2_FREQ;

	// 3. Address Mode
	if (pI2CHandle->I2C_PinConfig.AddressMode == I2C_7_BIT_ADDRESS_MODE) {
		pI2CHandle->pI2Cx->OAR1 &= ~(1 << 15);	// I2C_OAR1_ADDMODE = 15
	} else {
		pI2CHandle->pI2Cx->OAR1 |= (1 << 15);
	}

	// 4. Own Address
	pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->I2C_PinConfig.DeviceAddress & 0b1111111) << 1;	//I2C_OAR1_ADD = 1

	// Per reference manual, bit 14 of I2C_OAR1 should always be kept at 1 by software
	pI2CHandle->pI2Cx->OAR1 |= (1 << 14);

	// 5. Modes & TRISE & CCR Calculation
	if (pI2CHandle->I2C_PinConfig.SCLKSpeed <= I2C_SCLKSPEED_SM) {
		// Set Sm mode
		pI2CHandle->pI2Cx->CCR &= ~(1 << I2C_CCR_F_S);

		// Configure TRISE in Sm mode
		uint32_t T_PLCK1;
		T_PLCK1 = 1 / RCC_GetPCLK1Value();
		uint16_t trise = (SM_MAX_RISE_TIME / T_PLCK1) + 1;
		pI2CHandle->pI2Cx->TRISE |= (trise & 0b111111) << 0;

		// Configure CCR in Sm mode
		uint32_t T_SCLK = 1 / pI2CHandle->I2C_PinConfig.SCLKSpeed;
		uint16_t CCR = (T_SCLK / T_PLCK1) / 2;	// 2 is division factor according to reference manual

		pI2CHandle->pI2Cx->CCR |= (CCR & 0b111111111111) << I2C_CCR_CCR;
	} else if (pI2CHandle->I2C_PinConfig.SCLKSpeed <= I2C_SCLKSPEED_FM) {
		// Set Fm mode
		pI2CHandle->pI2Cx->CCR |= (1 << I2C_CCR_F_S);

		// Configure TRISE in FM mode
		uint32_t T_PLCK1, T_SCLK;
		T_PLCK1 = 1 / RCC_GetPCLK1Value();
		T_SCLK = 1 / pI2CHandle->I2C_PinConfig.SCLKSpeed;
		uint16_t trise = (FM_MAX_RISE_TIME / T_PLCK1) + 1;
		pI2CHandle->pI2Cx->TRISE |= (trise & 0b111111) << 0;

		// Configure CCR in FM mode and Duty Cycle
		if (pI2CHandle->I2C_PinConfig.FMDutyCycle == I2C_FM_DUTY_CYCLE_2) {
			// DUTY = 0
			pI2CHandle->pI2Cx->CCR &= ~(1 << I2C_CCR_DUTY);

			// CCR
			uint16_t CCR = (T_SCLK / T_PLCK1) / 3;	// 3 is division factor according to reference manual
			pI2CHandle->pI2Cx->CCR |= (CCR & 0b111111111111) << I2C_CCR_CCR;
		} else {
			// DUTY = 1
			pI2CHandle->pI2Cx->CCR |= (1 << I2C_CCR_DUTY);

			// CCR
			uint16_t CCR = (T_SCLK / T_PLCK1) / 25;	// 25 is division factor according to reference manual
			pI2CHandle->pI2Cx->CCR |= (CCR & 0b111111111111) << I2C_CCR_CCR;
		}
	}
}

void I2C_DeInit(I2C_Handle_t *pI2CHandle) {
	if (pI2CHandle->pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if (pI2CHandle->pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else {
		I2C3_REG_RESET();
	}
}

