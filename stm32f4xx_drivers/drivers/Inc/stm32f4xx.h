/*
 * stm32f4xx.h
 *
 *  Created on: Dec 5, 2024
 *      Author: ASUS
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include <stdint.h>
#include <stddef.h>

// some useful macros
#define __vo 			volatile
#define __weak			__attribute__((weak))
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_SET		SET
#define FLAG_RESET		RESET

// Address of different memory types
#define FLASH_BASE_ADDR		0x08000000UL		// start of flash memory
#define SRAM1_BASE_ADDR 	0x20000000UL		// start of SRAM1
#define SRAM2_BASE_ADDR 	0x2001C000UL		// start of SRAM2
#define ROM_BASE_ADDR 		0x1FFF0000UL		// start of ROM
#define SRAM 				SRAM1_BASE_ADDR		// SRAM is the same as SRAM1

// Address of different bus domains
#define PERIPH_BASE_ADDR	0x40000000UL		// start of bus domains
#define APB1PERIPH_BASE		PERIPH_BASE_ADDR	// start of APB1 bus
#define APB2PERIPH_BASE		0x40010000UL		// start of APB2 bus
#define AHB1PERIPH_BASE		0x40020000UL		// start of AHB1 bus
#define AHB2PERIPH_BASE		0x50000000UL	    // start of AHB2 bus

// Address of peripherals hanging on AHB1 bus
#define GPIOA_BASE			0x40020000UL
#define GPIOB_BASE			0x40020400UL
#define GPIOC_BASE			0x40020800UL
#define GPIOD_BASE			0x40020C00UL
#define GPIOE_BASE			0x40021000UL
#define GPIOF_BASE			0x40021400UL
#define GPIOG_BASE			0x40021800UL
#define GPIOH_BASE			0x40021C00UL
#define GPIOI_BASE			0x40022000UL
#define RCC_BASE			0x40023800UL


// Address of peripherals hanging on APB1 bus
#define I2C1_BASE   		0x40005400UL
#define I2C2_BASE			0x40005800UL
#define I2C3_BASE			0x40005C00UL
#define SPI2_BASE			0x40003800UL
#define SPI3_BASE			0x40003C00UL
#define USART2_BASE			0x40004400UL
#define USART3_BASE			0x40004800UL
#define UART4_BASE			0x40004C00UL
#define UART5_BASE			0x40005000UL

// Address of peripherals hanging on APB2 bus (not all)
#define EXTI_BASE			0x40013C00UL
#define SPI1_BASE			0x40013000UL
#define USART1_BASE			0x40011000UL
#define USART6_BASE			0x40011400UL
#define SYSCFG_BASE			0x40013800UL

// Address of NVIC registers
// (__vo uint32_t*) is used because we need to dereference these
// address later to access the registers at these addresses
#define NVIC_ISER0          ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1          ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2          ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3          ((__vo uint32_t*)0xE000E10C)
// we won't use more than 81 IRQ numbers

#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

#define NVIC_IPR_BASE 		((__vo uint32_t*)0xE000E400) // There are 59 of these

// Number of priority bits implemented in NVIC_IPR in ARM Cortex Mx
#define NO_PR_BITS_IMPLEMENTED  4
/*
************Peripheral Register Definition Structures*****************************
// More to be created for other peripherals as we encounter them
*/

// GPIO registers definition struct
// This struct contains all registers of GPIOx
typedef struct {
	__vo uint32_t MODER;			// GPIO port mode register
	__vo uint32_t OTYPER;        // GPIO port output type register
	__vo uint32_t OSPEEDR;		// GPIO port output speed register
	__vo uint32_t PUPDR;			// GPIO port pull-up/pull-down register
	__vo uint32_t IDR;			// GPIO port input data register
	__vo uint32_t ODR;			// GPIO port output data register
	__vo uint32_t BSRR;			// GPIO port bit set/reset register
	__vo uint32_t LCKR;			// GPIO port configuration lock register
	__vo uint32_t AFR[2];		// AFR[0]: GPIO alternate function low register
							// AFR[1]: GPIO alternate function high register
} GPIO_RegDef_t;

// RCC registers definition struct
typedef struct {
	__vo uint32_t CR;				//
	__vo uint32_t PLLCFGR;			//
	__vo uint32_t CFGR;				//
	__vo uint32_t CIR;				//
	__vo uint32_t AHB1RSTR;			//
	__vo uint32_t AHB2RSTR;			//
	__vo uint32_t AHB3RSTR;			//
	uint32_t RESERVED0;				// 0x1C is reserved
	__vo uint32_t APB1RSTR;			//
	__vo uint32_t APB2RSTR;			//
	uint32_t RESERVED1[2];			// 0x28, 0x2C are reserved
	__vo uint32_t AHB1ENR;			//
	__vo uint32_t AHB2ENR;			//
	__vo uint32_t AHB3ENR;			//
	uint32_t RESERVED2;				// 0x3C is reserved
	__vo uint32_t APB1ENR;			//
	__vo uint32_t APB2ENR;			//
	uint32_t RESERVED3;				// 0x48, 0x4C are reserved
	__vo uint32_t AHB1LPENR;		//
	__vo uint32_t AHB2LPENR;		//
	__vo uint32_t AHB3LPENR;		//
	uint32_t RESERVED4;				// 0x5C is reserved
	__vo uint32_t APB1LPENR;		//
	__vo uint32_t APB2LPENR;		//
	uint32_t RESERVED5;				// 0x68, 0x6C are reserved
	__vo uint32_t BDCR;				//
	__vo uint32_t CSR;				//
	uint32_t RESERVED6;				// 0x78, 0x7C are reserved
	__vo uint32_t SSCGR;			//
	__vo uint32_t PLLI2SCFGR;		//
} RCC_RegDef_t;

typedef struct {
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
} EXTI_RegDef_t;

typedef struct {
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED[2];			// 0x18, 0x1C are reserved
	__vo uint32_t CMPCR;
} SYSCFG_RegDef_t;

typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;

typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
} I2C_RegDef_t;

/*
***************Peripheral definition macros*************************************
*/

// GPIO peripheral definition macros
// GPIO addresses typecasted to GPIOx_RegDef_t*
#define GPIOA 		(GPIO_RegDef_t*) GPIOA_BASE
#define GPIOB 		(GPIO_RegDef_t*) GPIOB_BASE
#define GPIOC 		(GPIO_RegDef_t*) GPIOC_BASE
#define GPIOD 		(GPIO_RegDef_t*) GPIOD_BASE
#define GPIOE 		(GPIO_RegDef_t*) GPIOE_BASE
#define GPIOF 		(GPIO_RegDef_t*) GPIOF_BASE
#define GPIOG 		(GPIO_RegDef_t*) GPIOG_BASE
#define GPIOH 		(GPIO_RegDef_t*) GPIOH_BASE
#define GPIOI 		(GPIO_RegDef_t*) GPIOI_BASE

// RCC peripheral definition macros
// RCC address typecasted to RCC_RegDef_t*
#define RCC			((RCC_RegDef_t*) RCC_BASE)

// EXTI peripheral definition macros
// EXTI address typecasted to EXTI_RegDef_t*
#define EXTI		((EXTI_RegDef_t*) EXTI_BASE)

// SYSCFG peripheral definition macros
// SYSCFG address typecasted to SYSCFG_RegDef_t*
#define SYSCFG		((SYSCFG_RegDef_t*) SYSCFG_BASE)

// SPI peripheral definition macros
// SPIx addresses typecasted to SPI_RegDef_t*
#define SPI1		((SPI_RegDef_t*) SPI1_BASE)  	// hanging onto APB2
#define SPI2		((SPI_RegDef_t*) SPI2_BASE)		// hanging onto APB1
#define SPI3		((SPI_RegDef_t*) SPI3_BASE) 	// hanging onto APB1

// I2C peripheral definition macros
// I2Cx address typecasted to I2C_RegDef_t*
#define I2C1		((I2C_RegDef_t*) I2C1_BASE)		// APB1
#define I2C2		((I2C_RegDef_t*) I2C2_BASE)		// APB1
#define I2C3		((I2C_RegDef_t*) I2C3_BASE)		// APB1

// Function to enable GPIOx clock where x is the port number (A-I)
//void GPIOx_CLK_EN(char x) {
//	if (x >= 65 && x <= 73) {
//		int shift_val = x - 'A';
//		RCC->AHB1ENR |= (1 << shift_val);
//	} else if (x >= 97 && x <= 105) {
//		int shift_val = x - 'a';
//		RCC->AHB1ENR |= (1 << shift_val);
//	} else {
//		printf("GPIO%c is invalid!\n Has to be between A and I", x);
//		while(1);
//	}
//}

// Functions to enable/disable GPIOx's clock
#define GPIOA_CLK_EN() 		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN() 		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN() 		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN() 		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN() 		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN() 		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN() 		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN() 		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_CLK_EN() 		(RCC->AHB1ENR |= (1 << 8))
#define GPIOA_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 8))

// Functions to enable/disable I2Cx's clock
#define I2C1_CLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C1_CLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_CLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C2_CLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_CLK_EN()		(RCC->APB1ENR |= (1 << 23))
#define I2C3_CLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

// Functions to enable/disable SPIx's clock
#define SPI1_CLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI1_CLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_CLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI2_CLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_CLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI3_CLK_DI()		(RCC->APB1ENR &= ~(1 << 15))

// Functions to enable/disable USARTx's clock
#define USART2_CLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART2_CLK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_CLK_EN()		(RCC->APB1ENR |= (1 << 18))
#define USART3_CLK_DI()		(RCC->APB1ENR &= ~(1 << 18))

// Functions to enable/disable UARTx's clock
#define UART4_CLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART4_CLK_DI()		(RCC->APB1ENR |= (0 << 19))
#define UART5_CLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define UART5_CLK_DI()		(RCC->APB1ENR |= (0 << 20))

// Function to enable/disable SYSCFG peripheral
#define SYSCFG_CLK_EN()		(RCC->APB2ENR |= (1 << 14));
#define SYSCFG_CLK_DI()		(RCC->APB2ENR &= ~(1 << 14));

// Functions to reset GPIOx peripherals
// Reference manual says "set and cleared by software"
#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0);
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0);
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0);
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0);
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0);
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0);
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0);
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0);
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));} while(0);

// Functions to reset SPIx peripherals
#define SPI1_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));} while(0);
#define SPI2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB2RSTR &= ~(1 << 14));} while(0);
#define SPI3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB2RSTR &= ~(1 << 15));} while(0);

// Functions to reset I2Cx peripherals
#define I2C1_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));} while(0);
#define I2C2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));} while(0);
#define I2C3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));} while(0);

// Function to get SYSCFG_EXTICR portcode of each GPIO peripheral
#define GPIO_TO_SYSCFG_EXTICR_PORTCODE(pGPIOx)		((pGPIOx == GPIOA) ? 0 :\
													 (pGPIOx == GPIOB) ? 1 :\
													 (pGPIOx == GPIOC) ? 2 :\
													 (pGPIOx == GPIOD) ? 3 :\
													 (pGPIOx == GPIOE) ? 4 :\
													 (pGPIOx == GPIOF) ? 5 :\
													 (pGPIOx == GPIOG) ? 6 :\
													 (pGPIOx == GPIOH) ? 7 :\
													 (pGPIOx == GPIOI) ? 8 :0)

// IRQ numbers for each EXTI line
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

// IRQ numbers for SPI
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36


// Possible priority levels
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15

/**************Bit Position Defintions********************/

// bit position definitions for SPI_CR1
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

// bit position definitions for SPI_CR2
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

// bit position definitions for SPI_SR
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

// bit position definitions for I2C_CR1
#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

// bit position definitions for I2C_CR2
#define	I2C_CR2_FREQ			0
#define	I2C_CR2_ITERREN			8
#define	I2C_CR2_ITEVTEN			9
#define	I2C_CR2_ITBUFEN			10
#define	I2C_CR2_DMAEN			11
#define	I2C_CR2_LAST			12

// bit position definitions for I2C_SR1
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RxNE			6
#define I2C_SR1_TxE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

// bit position definitions for I2C_SR2
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8

// bit position definitions for I2C_CCR
#define	I2C_CCR_CCR			0
#define	I2C_CCR_DUTY		14
#define	I2C_CCR_F_S			15

// why are these included at the end???
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"
#include "stm32f4xx_i2c_driver.h"

#endif /* INC_STM32F4XX_H_ */
