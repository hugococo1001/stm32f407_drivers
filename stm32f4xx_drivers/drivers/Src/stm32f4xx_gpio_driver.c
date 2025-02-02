/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Dec 8, 2024
 *      Author: ASUS
 */

#include <stm32f4xx_gpio_driver.h>

// GPIO peripheral clock setup
/******************************************************************8
  * @fn			- GPIO_PeriClockControl
  *
  * @brief		- Enable a GPIO's clock by turning on the clock for AHB1 bus
  *
  * @param[in]	- pointer to a GPIO
  * @param[in]	- ENABLE or DISABLE
  *
  * @return		- void
  *
  * @Note		- void
*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pGPIOx == GPIOA){
			GPIOA_CLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_CLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_CLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_CLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_CLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_CLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_CLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_CLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_CLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA){
			GPIOA_CLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_CLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_CLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_CLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_CLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_CLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_CLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_CLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_CLK_DI();
		}
	}
}

// GPIO init/de-init
/*******************************************************************
  * @fn			- GPIO_Init
  *
  * @brief		- Initialize a GPIO by configuring its registers
  * 				1. configure the mode of a GPIO pin
  * 				2. configure the speed
  * 				3. configure the pupd settings
  * 				4. configure the optype
  * 				5. configure the alt functionality
  *
  * @param[in]	- pointer to a GPIO handle
  *
  * @return		- void
  *
  * @Note		- also turns on the clock
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	// Enable GPIO clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. configure the mode of a GPIO pin
	uint32_t pin_mode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
	if (pin_mode <= GPIO_MODE_ANALOG) {
		// non-interrupt mode
		uint32_t register_index = 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
		pGPIOHandle->pGPIOx->MODER &= ~(0b11 << register_index); // clearing
		pGPIOHandle->pGPIOx->MODER |= pin_mode << register_index; // setting
	} else {
		// interrupt mode
		// 1. Configure
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // setting
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // setting
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // setting

			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // setting
		}

		// 2. select GPIO port in SYSCFG_EXTICR
		uint8_t SYSCFG_EXTICRx = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t SYSCFG_EXTICRx_reg_index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_TO_SYSCFG_EXTICR_PORTCODE(pGPIOHandle->pGPIOx);

		SYSCFG_CLK_EN();
		SYSCFG->EXTICR[SYSCFG_EXTICRx] = (uint32_t)portcode << (SYSCFG_EXTICRx_reg_index * 4);
		// = is used instead of |= to clear all other bits and just sets the relevant

		// 3. unmask line x for interrupt request
		EXTI->IMR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // setting
	}

	// 2. configure the speed
	uint32_t pin_speed = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed;
	uint32_t register_index = 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0b11 << register_index); // clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= pin_speed << register_index; // setting

	// 3. configure the pupd settings
	uint32_t pin_pupd = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl;
	register_index = 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->PUPDR &= ~(0b11 << register_index); // clearing
	pGPIOHandle->pGPIOx->PUPDR |= pin_pupd << register_index; // setting

	// 4. configure the optype
	uint32_t pin_optype = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType;
	register_index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0b1 << register_index); // clearing
	pGPIOHandle->pGPIOx->OTYPER |= pin_optype << register_index; // setting

	// 5. configure the alt functionality
	if (pin_mode == GPIO_MODE_ALTFN) {
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7) {
			// configure GPIO alternate function low register
			uint32_t pin_alt_func = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode;
			register_index = 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
			pGPIOHandle->pGPIOx->AFR[0] &= ~(0b1 << register_index); // clearing
			pGPIOHandle->pGPIOx->AFR[0] |= pin_alt_func << register_index; // setting
		} else {
			// configure GPIO alternate function high register
			uint32_t pin_alt_func = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode;
			register_index = 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8);
			pGPIOHandle->pGPIOx->AFR[1] &= ~(0b1111 << register_index); // clearing
			pGPIOHandle->pGPIOx->AFR[1] |= pin_alt_func << register_index; // setting
		}
	}
}

/*******************************************************************
  * @fn			- GPIO_DeInit
  *
  * @brief		- Reset all registers of a GPIOx peripheral
  *
  * @param[in]	- pointer to a GPIO base address
  *
  * @return		- void
  *
  * @Note		- void
*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
}


// GPIO data read and write
/******************************************************************8
  * @fn			- GPIO_ReadFromInputPin
  *
  * @brief		- Read from a GPIOx pin
  *
  * @param[in]	- pointer to a GPIOx base address
  * 			- pin number
  *
  * @return		- the value read in (1 or 0)
  *
  * @Note		- void
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	return (uint8_t)((pGPIOx->IDR >> PinNumber) & 0b1);
}

/******************************************************************8
  * @fn			- GPIO_ReadFromInputPort
  *
  * @brief		- Read from a GPIOx port
  *
  * @param[in]	- pointer to a GPIOx base address
  *
  * @return		- content of input data register
  *
  * @Note		- void
*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIO) {
	return (uint16_t)(pGPIO->ODR);
}

/******************************************************************8
  * @fn			- GPIO_WriteToOutputPin
  *
  * @brief		- Write to a GPIOx pin
  *
  * @param[in]	- pointer to a GPIOx base address
  * 			- pin number to write to
  * 			- value to write (1 or 0)
  *
  * @return		- void
  *
  * @Note		- void
*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
	if (Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/******************************************************************8
  * @fn			- GPIO_WriteToOutputPort
  *
  * @brief		- Write to a GPIOx port
  *
  * @param[in]	- pointer to a GPIOx base address
  * 			- value to write (uint16_t)
  *
  * @return		- void
  *
  * @Note		- void
*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

/******************************************************************8
  * @fn			- GPIO_ToggleOutputPin
  *
  * @brief		- Toggle an output pin (1 to 0,  0 to 1)
  *
  * @param[in]	- pointer to a GPIOx base address
  * 			- pin number
  *
  * @return		- void
  *
  * @Note		- void
*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

// GPIO IRQ config and ISR handling
void GPIO_IRQ_Number_Config(uint8_t IRQNumber, uint8_t EnorDi) {
	if(EnorDi == ENABLE)
	{
		if(IRQNumber < 32)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 32) );
		}
	}else
	{
		if(IRQNumber < 32)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 32) );
		}
	}

}

void GPIO_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority) {
	// find out which IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t NVIC_IPR_field_number = IRQNumber % 4;
	uint8_t shift_amount = (NVIC_IPR_field_number * 8) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASE + iprx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber) {
	if (EXTI->PR & (1 << PinNumber)) {
		// clear bit by setting to 1
		EXTI->PR |= (1 << PinNumber);
	}
}
