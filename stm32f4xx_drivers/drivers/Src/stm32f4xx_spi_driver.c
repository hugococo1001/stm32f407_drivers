/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Dec 19, 2024
 *      Author: ASUS
 */

#include "stm32f4xx_spi_driver.h"

// helper functions (invisible to users)
static void SPI_TXE_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_interrupt_handle(SPI_Handle_t *pSPIHandle);


// Get a specific flag status in SPI
/******************************************************************8
  * @fn			- SPI_GetFlagStatus
  *
  * @brief		- Get a specific flag status in SPI
  * @param[in]	- pointer to a SPIx
  * @param[in]	- name of the flag (refer bit position definitions for SPI_SR)
  *
  * @return		- 1 or 0
  *
  * @Note		- void
*/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName) {
	if (pSPIx->SR & (1 << FlagName)) {
		return FLAG_SET;
	} else {
		return FLAG_RESET;
	}
}

// Enable or disable a SPI peripheral
/******************************************************************8
  * @fn			- SPI_PeriControl
  *
  * @brief		- Enable or disable a SPI peripheral by configuring the SPE bit in the SPI_CR1 register.
  * @param[in]	- pointer to a SPIx
  * @param[in]	- ENABLE or DISABLE
  *
  * @return		- void
  *
  * @Note		- void
*/
void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		// Enable: setting SPE bit
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		// Disable: resetting SPE bit
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

// SPI peripheral clock setup
/******************************************************************8
  * @fn			- SPI_PeriClockControl
  *
  * @brief		- Enable a SPI's clock by turning on the clock for APB1/APB2 bus
  *
  * @param[in]	- pointer to a SPIx
  * @param[in]	- ENABLE or DISABLE
  *
  * @return		- void
  *
  * @Note		- void
*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_CLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_CLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_CLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_CLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_CLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_CLK_DI();
		}
	}

};


// SPI initialization
/*******************************************************************
  * @fn			- GPIO_Init
  *
  * @brief		- Initialize a SPIx by configuring its registers
  * 				CR1:
  * 				1. configure the device mode
  * 				2. configure the bus configuration
  * 				3. configure the clock speed
  * 				4. configure the data frame
  * 				5. configure the clock polarity
  * 				6. configure the clock phase
  * 				7. configure the SSM bit
  * 				8. configure the SSI bit
  * 				CR2:
  * 				9. configure the SSOE bit
  *
  * @param[in]	- pointer to a GPIO handle
  *
  * @return		- void
  *
  * @Note		- First resets CR1 register to 0
  * 			- Also enable the clock for SPI
*/
void SPI_Init(SPI_Handle_t* pSPIHandle) {
	// Enable the clock for SPI
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// first, configure the CR1 register
	// 1. Configure device mode
	pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPI_PinConfig.SPI_DeviceMode << 2;

	// 2. Configure bus config
	if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// set BIDIMODE bit to 0
		pSPIHandle->pSPIx->CR1 &= ~(1 << 15);
	} else if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// set BIDIMODE bit to 1
		pSPIHandle->pSPIx->CR1 |= (1 << 15);
	} else if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// set BIDIMODE bit to 0 and RXONLY bit to 1
		// remember to still use MOSI line
		pSPIHandle->pSPIx->CR1 &= ~(1 << 15);
		pSPIHandle->pSPIx->CR1 |= (1 << 10);
	}

	// 3. Configure clock speed
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPI_PinConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// 4. Configure data frame
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPI_PinConfig.SPI_DFF << SPI_CR1_DFF);

	// 5. Configure clock polarity
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPI_PinConfig.SPI_CPOL << SPI_CR1_CPOL);

	// 6. Configure clock phase
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPI_PinConfig.SPI_CPHA << SPI_CR1_CPHA);

	// 7. Configure SSM bit
	if (pSPIHandle->SPI_PinConfig.SPI_SSM == ENABLE) {
		// Enable SSM bit
		pSPIHandle->pSPIx->CR1 |= (ENABLE << SPI_CR1_SSM);
	} else {
		// Disable SSM bit
		pSPIHandle->pSPIx->CR1 &= ~(ENABLE << SPI_CR1_SSM);
	}


	// 8. Configure SSI bit
	if (pSPIHandle->SPI_PinConfig.SPI_SSI == ENABLE) {
		// Enable SSI bit
		pSPIHandle->pSPIx->CR1 |= (ENABLE << SPI_CR1_SSI);
	} else {
		// Disable SSI bit
		pSPIHandle->pSPIx->CR1 &= ~(ENABLE << SPI_CR1_SSI);
	}


	// 9. Configure SSOE bit
	if (pSPIHandle->SPI_PinConfig.SPI_SSOE == ENABLE) {
		// Enable SSOE bit
		pSPIHandle->pSPIx->CR2 |= (ENABLE << SPI_CR2_SSOE);
	} else {
		// Disable SSOE bit
		pSPIHandle->pSPIx->CR2 &= ~(ENABLE << SPI_CR2_SSOE);
	}

}

// SPI De-initialization
/******************************************************************8
  * @fn			- SPI_DeInit
  *
  * @brief		- De-initialized a SPIx by turning off its clock using APBxRST
  *
  * @param[in]	- SPI handle
  *
  * @return		- void
  *
  * @Note		- void
*/
void SPI_DeInit(SPI_Handle_t *pSPIHandle) {
	if (pSPIHandle->pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIHandle->pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIHandle->pSPIx == SPI3) {
		SPI3_REG_RESET();
	}
}

// SPI Send Data
/******************************************************************8
  * @fn			- SPI_SendData
  *
  * @brief		- Send data from Txbuffer to DR register
  *
  * @param[in]	- SPIx
  * @param[in]	- void buffer
  * @param[in]	- length of data to be sent
  *
  * @return		- void
  *
  * @Note		- data is stored in a void buffer which can then be casted to 8 or 16-bit
*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *TxBuffer, uint32_t len) { 	//
	while (len > 0) {
		// 1. Wait until TXE is set
		// Here, we're polling for the TXE flag to SET
		while (SPI_GetFlagStatus(pSPIx, SPI_SR_TXE) == FLAG_RESET);

		// 2. Check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16-bit buffer
			// load data into DR
			pSPIx->DR = *((uint16_t*)TxBuffer);
			// len is decremented by 2 bytes
			len--;
			len--;

			// increment to the next output bit
			(uint16_t*)TxBuffer++;
		} else {
			// 8-bit buffer
			// load data into DR
			pSPIx->DR = *(uint8_t*)TxBuffer;

			len--;

			// increment to the next output bit
			TxBuffer++;
		}
	}
}

// SPI Receive Data
/******************************************************************8
  * @fn			- SPI_ReceiveData
  *
  * @brief		- Receive data from DR register and store it to RxBuffer
  *
  * @param[in]	- SPIx
  * @param[in]	- void buffer
  * @param[in]	- length of data to be received
  *
  * @return		- void
  *
  * @Note		- before that, check if RxBuffer is empty
*/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *RxBuffer, uint32_t len) { 	//
	while (len > 0) {
		// 1. Wait until RXNE is set
		// Here, we're polling for the RXNE flag to RESET
		while (SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE) == FLAG_SET);

		// 2. Check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16-bit buffer
			// read data into RxBuffer
			*((uint16_t*)RxBuffer) = pSPIx->DR;
			// len is decremented by 2 bytes
			len--;
			len--;

			// increment to the next output byte
			(uint16_t*)RxBuffer++;
		} else {
			// 8-bit buffer
			// read data into RxBuffer
			*RxBuffer = pSPIx->DR;
			// len is decremented by 1 byte
			len--;

			// increment to the next input byte
			RxBuffer++;
		}

	}
}

// SPI Interrupt-based Send Data
/******************************************************************8
  * @fn			- SPI_SendDataIT
  *
  * @brief		- Setting up the necessary variables to enable
  * 			- sending data from Txbuffer to DR register
  * 			- when interrupted
  *
  * @param[in]	- pSPIHandle
  * @param[in]	- TxBuffer
  * @param[in]	- length of data to be sent
  *
  * @return		- void
  *
  * @Note		- As long as Tx buffer is filled (TXE is 0), data is sent
  * 			- The actual transmission is implemented in another function (SPI2_IRQHandler)
*/
uint8_t  SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *TxBuffer, uint32_t len) {
	uint8_t SPI_status = pSPIHandle->TxState;

	if (SPI_status != SPI_BUSY_IN_TX) {
		//1. Save TxBuffer's address and length in some global variables.
		pSPIHandle->pTxBuffer = TxBuffer;
		pSPIHandle->TxLen = len;

		//2. The SPI state as busy in Tx so no other code can take over the the same SPI
		// peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE bit in SPI_CR2
		// This bit enables Tx buffer empty interrupt
		pSPIHandle->pSPIx->CR2 |= (ENABLE << SPI_CR2_TXEIE);
	}

	return SPI_status;
}

// SPI Interrupt-based Receive Data
/******************************************************************8
  * @fn			- SPI_ReceiveDataIT
  *
  * @brief		- Setting up the necessary variables to enable
  * 			- receiving data from DR register and store it to RxBuffer
  * 			- when interrupted
  *
  * @param[in]	- pSPIHandle
  * @param[in]	- RxBuffer
  * @param[in]	- length of data to be received
  *
  * @return		- void
  *
  * @Note		- As long as RxBuffer is full (RXNE is 1), data is received
  * 			- The actual reception is in the SPI2_IRQHandler function
*/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *RxBuffer, uint32_t len) {
	uint8_t SPI_status = pSPIHandle->RxState;

	if (SPI_status != SPI_BUSY_IN_RX) {
		//1. Save RxBuffer's address and length in some global variables.
		pSPIHandle->pRxBuffer = RxBuffer;
		pSPIHandle->RxLen = len;

		//2. The SPI state as busy in Rx so no other code can take over the the same SPI
		// peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE bit in SPI_CR2
		// This bit enables Rx buffer not empty interrupt
		pSPIHandle->pSPIx->CR2 |= (ENABLE << SPI_CR2_RXNEIE);
	}

	return SPI_status;
}

// SPI Enable IRQ Number
/******************************************************************8
  * @fn			- SPI_IRQ_Number_Config
  *
  * @brief		- Enable IRQ Number for SPI Interrupts
  *
  * @param[in]	- IRQ number
  * @param[in]	- En or Di
  *
  * @return		- void
  *
  * @Note		- void
*/
void SPI_IRQ_Number_Config(uint8_t IRQNumber, uint8_t EnorDi) {
	if(EnorDi == ENABLE)
	{
		if(IRQNumber < 32)
		{
			//program ISER0 register (0 to 31)
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber < 64 )
		{
			//program ISER1 register (32 to 63)
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber < 96 )
		{
			//program ISER2 register (64 to 95)
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

// SPI Interrupt Priority Config
/******************************************************************8
  * @fn			- SPI_IRQ_Priority_Config
  *
  * @brief		- Set SPI interrupt priority
  *
  * @param[in]	- IRQ number
  * @param[in]	- Priority number
  *
  * @return		- void
  *
  * @Note		- void
*/
void SPI_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority) {
	// find out which IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t NVIC_IPR_field_number = IRQNumber % 4;
	uint8_t shift_amount = (NVIC_IPR_field_number * 8) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASE + iprx) |= (IRQPriority << shift_amount);
}

// SPI Interrupt ISR
/******************************************************************8
  * @fn			- SPI2_IRQHandler
  *
  * @brief		- Handle all SPI interrupts
  *
  * @param[in]	- SPI_Handle_t *pSPIHandle
  *
  * @return		- void
  *
  * @Note		- Handle SPI interrupts from all TXEIE, RNXEIE, ERRIE
  * 			- including  helper functions for each one of those:
  * 			- SPI_TXE_interrupt_handle(pSPIHandle)
  * 			- SPI_RXNE_interrupt_handle(pSPIHandle)
  * 			- SPI_OVR_interrupt_handle(pSPIHandle)
  * 			- to be called by ISE, which is implmented by application
*/
void SPI2_IRQHandling(SPI_Handle_t *pSPIHandle) {
	// Check if TXE and TXEIE bit is set
	uint8_t TXE_bit = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	uint8_t TXEIE_bit = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if (TXE_bit && TXEIE_bit) {
		SPI_TXE_interrupt_handle(pSPIHandle);
	}

	// Check if RXNE and RXNEIE bit is set
	uint8_t RXNE_bit = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	uint8_t RXNEIE_bit = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if (RXNE_bit && RXNEIE_bit) {
		SPI_RXNE_interrupt_handle(pSPIHandle);
	}

	// Check if overrun error has occurred (we'll ignore other errors);
	uint8_t OVR_bit = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	uint8_t ERRIE_bit = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if (OVR_bit && ERRIE_bit) {
		SPI_OVR_interrupt_handle(pSPIHandle);
	}
}

/***********************helper functions implementations*******************/

// Handle SPI TXE Interrupt
/******************************************************************8
  * @fn			- SPI_TXE_interrupt_handle
  *
  * @brief		- Handle SPI TXE interrupt
  * 			- send data (8 or 16 bits) every time TXE is set
  *
  * @param[in]	- SPI_Handle_t *pSPIHandle
  *
  * @return		- void
  *
  * @Note		- void
*/
static void SPI_TXE_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	// Check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16-bit buffer
		// load data into DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		// length is decremented by 2 bytes
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;

		// increment to the next output bit
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else {
		// 8-bit buffer
		// load data into DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;

		pSPIHandle->TxLen--;

		// increment to the next output bit
		pSPIHandle->pTxBuffer++;
	}

	// Close transmission
	if (!pSPIHandle->TxLen) {
		SPI_CloseTransmission(pSPIHandle);

		// callback function
		// we pass in the event
		// user decides what happens when that event occurs
		// implemented by user
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

// Handle SPI RXNE Interrupt
/******************************************************************8
  * @fn			- SPI_RXNE_interrupt_handle
  *
  * @brief		- Handle SPI RXNE interrupt
  * 			- receive data (8 or 16 bits) every time RXNE is set
  *
  * @param[in]	- SPI_Handle_t *pSPIHandle
  *
  * @return		- void
  *
  * @Note		- void
*/
static void SPI_RXNE_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	// Check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16-bit buffer
		// read data into RxBuffer
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		// len is decremented by 2 bytes
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;

		// increment to the next output byte
		(uint16_t*)pSPIHandle->pRxBuffer++;
	} else {
		// 8-bit buffer
		// read data into RxBuffer
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		// len is decremented by 1 byte
		pSPIHandle->RxLen--;

		// increment to the next input byte
		pSPIHandle->pRxBuffer++;
	}

	// Close reception
	if (pSPIHandle->RxLen)
	SPI_CloseReception(pSPIHandle);

	// callback function
	// we pass in the event
	// user decides what happens when that event occurs
	// implemented by user
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
}


static void SPI_OVR_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	// Clear OVR bit if not busy in communication
	uint8_t BSY_flag = pSPIHandle->pSPIx->SR & (1 << SPI_SR_BSY);
	if (BSY_flag == 0) {
		uint16_t temp;
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
		(void)temp;
	} else {
		// user clears OVR bit themselves
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERROR);
	}

}

// SPI Close Transmission
/******************************************************************8
  * @fn			- SPI_CloseTransmission
  *
  * @brief		- Close transmission by reset TXEIE bit and other
  * 			  variables
  *
  * @param[in]	- SPI_Handle_t *pSPIHandle
  *
  * @return		- void
  *
  * @Note		- void
*/
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
	// reset TXEIE bit in SPI_CR2
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

// SPI Close Reception
/******************************************************************8
  * @fn			- SPI_CloseReception
  *
  * @brief		- Close transmission by reset RXNEIE bit and other
  * 			  variables
  *
  * @param[in]	- SPI_Handle_t *pSPIHandle
  *
  * @return		- void
  *
  * @Note		- void
*/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	// reset RXNEIE bit in SPI_CR2
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

// SPI Clear OVR flag
/******************************************************************8
  * @fn			- SPI_ClearOVRFlag
  *
  * @brief		- created for user to clear OVR bit in call
  *
  * @param[in]	- SPI_Handle_t *pSPIHandle
  *
  * @return		- void
  *
  * @Note		- void
*/
void SPI_ClearOVRFlag (SPI_Handle_t *pSPIHandle) {
	uint16_t temp;
	temp = pSPIHandle->pSPIx->DR;
	temp = pSPIHandle->pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t SPI_event) {
	// This is a weak implementation. User can override.
}
