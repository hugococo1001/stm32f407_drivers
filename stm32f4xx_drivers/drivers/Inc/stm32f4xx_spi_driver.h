/*
 * tm32f4xx_spi_driver.h
 *
 *  Created on: Dec 19, 2024
 *      Author: ASUS
 */

#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_


#include "stm32f4xx.h"

/*****************SPI Definition Structures*************************************/
typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;		// used to configure the direction of transmission
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
	uint8_t SPI_SSI;
	uint8_t SPI_SSOE;
}SPI_PinConfig_t;

typedef struct {
	SPI_RegDef_t* pSPIx;
	SPI_PinConfig_t SPI_PinConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;

/***************refer to these macros to configure SPI peripheral*********************/
//@SPI_DeviceMode
#define SPI_DEVICE_MODE_MASTER 		1
#define SPI_DEVICE_MODE_SLAVE 		0

//@SPI_BusConfig
#define	SPI_BUS_CONFIG_FD					0
#define	SPI_BUS_CONFIG_HD					1
// #define	SPI_BUS_CONFIG_SIMPLEX_TXONLY		// Since this mode is just FD
												// without connecting the MISO
												// line, we won't need it.
												// We can't do the same with RXONLY
												// mode because MOSI line has to
												// be connected for the clock to
												// function
#define	SPI_BUS_CONFIG_SIMPLEX_RXONLY		2

//@SPI_SclkSpeed
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

//@SPI_DFF
#define SPI_DFF_8BITS			0
#define SPI_DFF_16BITS			1

//@SPI_CPOL
#define	SPI_CPOL_LOW			0
#define	SPI_CPOL_HIGH			1

//@SPI_CPAL
#define	SPI_CPHA_LOW			0
#define	SPI_CPHA_HIGH			1

//@SPI_SSM
#define SPI_SSM_DI				0
#define SPI_SSM_EN				1


//@SPI_SSI
#define SPI_SSI_DI				0
#define SPI_SSI_EN				1

//@SPI_SSOE
#define SPI_SSOE_DI				0
#define SPI_SSOE_EN				1

//SPI states
#define SPI_READY				0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2

// SPI communication events
// similar to SPI communication outcomes
#define	SPI_EVENT_TX_CMPLT		0
#define	SPI_EVENT_RX_CMPLT		1
#define	SPI_EVENT_OVR_ERROR		2

/***************************API Function Prototypes****************************************/
void delay();
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName);
void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_Handle_t *pSPIHandle);

// Blocking Send & Receive functions
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *TxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *RxBuffer, uint32_t len);

// Interrupt-based Send & Receive functions
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *TxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *RxBuffer, uint32_t len);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ClearOVRFlag (SPI_Handle_t *pSPIHandle);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t SPI_event);

// SPI interrupt config functions
void SPI_IRQ_Number_Config(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI2_IRQHandling(SPI_Handle_t *pSPIHandle);

#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
