/*
 * stm32f407.h
 * Author: Kaitlyn W.
 */
#include <stdint.h>

#ifndef INC_STM32F407_DRIVERS_H_
#define INC_STM32F407_DRIVERS_H_

#define __vo volatile
// Flash addresses of MCU memories, e.g. Flash, SRAM1, SRAM2, ROM
#define FLASH_BASEADRR				0x08000000U // Sector 0 start address
#define SRAM1_BASEADDR				0x20000000U
#define SRAM2_BASEADDR				0x2001C000U // SRAM_BASEADDR + 112kb
#define ROM_BASEADDR				0x1FFF0000U
#define OTP_BASEADDR				0x1FFF7800U
#define SRAM						SRAM1_BASEADDR


// AHB1/2 and APB1/2 bus peripheral base addresses
#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASE				0x50000000U


// Peripherals hanging off of AHB1 bus
#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400U)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800U)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000U)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)


// Peripherals hanging off of APB1 bus
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3 BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)


// Peripherals hanging off of APB2 bus
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)
#define EXT1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)


// Peripheral register definition structs
// GPIO registers
typedef struct
{
	__vo uint32_t MODER;			// GPIO port mode register
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];			// AFR[0]: Alternative function LOW, AFR[1]: Alternative function HIGH
} GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;				// Address offset: 0x00
	__vo uint32_t PLLCFGR;			// Address offset: 0x04
	__vo uint32_t CFGR;				// Address offset: 0x08
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t 	  RESERVED0;
	__vo uint32_t APB1RSTR;
	uint32_t      APB2RSTR;
	uint32_t 	  RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t      RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t 	  RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB12PENR;
	uint32_t      RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t 	  RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLL12SCFGR;
} RCC_RegDef_t;

#define GPIOA						((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG						((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH						((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI						((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define RCC							((RCC_RegDef_t*) RCC_BASEADDR)

// Clock enable macros for GPIOx - AHB1ENR
#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1 << 0)) // GPIOs hang off of AHB1, and GPIOAEN is bit 0
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()				(RCC->AHB1ENR |= (1 << 8))
// Clock disable macros for GPIOx
#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 8))


// Clock enable macros for I2Cx - APB1ENR
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1 << 23))
// Clock disable macros for I2Cx
#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 23))


// Clock enable macros for SPIx - APB2, APB1
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15))
// Clock disable macros for SPIx
#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))


// Clock enable macros for USARTx - APB2, APB1
#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN()			(RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN()			(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1 << 5))
// Clock disable macros for USARTx
#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define USART4_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 19))
#define USART5_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 5))


// Clock enable macros for SYSCFG - APB2
#define SYSCFG_PCLK_EN()			(RCC->APB2EgNR |= (1 << 14))
// Clock disable macros for SYSCFG
#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))

#endif /* INC_STM32F407_DRIVERS_H_ */
