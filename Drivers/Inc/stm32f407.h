/*
 * stm32f407.h
 * Author: Kaitlyn W.
 */


#ifndef INC_STM32F407_DRIVERS_H_
#define INC_STM32F407_DRIVERS_H_

#include <stdint.h>

#define __vo volatile


/*** ARM Cortex-M4 PROCESSOR SPECIFIC MACROS ***/
/*** NVIC Set-enable register addresses ***/
#define NVIC_ISER0_BASEADDR			0xE000E100U
#define NVIC_ISER1_BASEADDR			0xE000E104U
#define NVIC_ISER2_BASEADDR			0xE000E108U
#define NVIC_ISER3_BASEADDR			0xE000E10CU
#define NVIC_ISER4_BASEADDR			0xE000E110U
#define NVIC_ISER5_BASEADDR			0xE000E114U
#define NVIC_ISER6_BASEADDR			0xE000E118U
#define NVIC_ISER7_BASEADDR			0xE000E11CU


/*** NVIC Clear-enable register addresses ***/
#define NVIC_ICER0_BASEADDR			0xE000E180U
#define NVIC_ICER1_BASEADDR			0xE000E184U
#define NVIC_ICER2_BASEADDR			0xE000E188U
#define NVIC_ICER3_BASEADDR			0xE000E18CU
#define NVIC_ICER4_BASEADDR			0xE000E190U
#define NVIC_ICER5_BASEADDR			0xE000E194U
#define NVIC_ICER6_BASEADDR			0xE000E198U
#define NVIC_ICER7_BASEADDR			0xE000E19CU


/*** NVIC Set-pending register addresses ***/
#define NVIC_ISPR0_BASEADDR			0XE000E200U
#define NVIC_ISPR1_BASEADDR			0XE000E204U
#define NVIC_ISPR2_BASEADDR			0XE000E208U
#define NVIC_ISPR3_BASEADDR			0XE000E20CU
#define NVIC_ISPR4_BASEADDR			0XE000E210U
#define NVIC_ISPR5_BASEADDR			0XE000E214U
#define NVIC_ISPR6_BASEADDR			0XE000E218U
#define NVIC_ISPR7_BASEADDR			0XE000E21CU


/*** NVIC Clear-pending register addresses ***/
#define NVIC_ICPR0_BASEADDR			0XE000E280U
#define NVIC_ICPR1_BASEADDR			0XE000E284U
#define NVIC_ICPR2_BASEADDR			0XE000E288U
#define NVIC_ICPR3_BASEADDR			0XE000E28CU
#define NVIC_ICPR4_BASEADDR			0XE000E290U
#define NVIC_ICPR5_BASEADDR			0XE000E294U
#define NVIC_ICPR6_BASEADDR			0XE000E298U
#define NVIC_ICPR7_BASEADDR			0XE000E29CU


/*** NVIC Priority register addresses (60 total) ***/
#define NVIC_IPR0_BASEADDR			0xE000E400U
#define NVIC_IPR59_BASEADDR			0xE000E4EFU
#define NVIC_IPR_BASEADDR			NVIC_IPR0_BASEADDR


/*** NVIC Set-enable registers ***/
#define NVIC_ISER0					((__vo uint32_t*) NVIC_ISER0_BASEADDR)
#define NVIC_ISER1					((__vo uint32_t*) NVIC_ISER1_BASEADDR)
#define NVIC_ISER2					((__vo uint32_t*) NVIC_ISER2_BASEADDR)
#define NVIC_ISER3					((__vo uint32_t*) NVIC_ISER3_BASEADDR)
#define NVIC_ISER4					((__vo uint32_t*) NVIC_ISER4_BASEADDR)
#define NVIC_ISER5					((__vo uint32_t*) NVIC_ISER5_BASEADDR)
#define NVIC_ISER6					((__vo uint32_t*) NVIC_ISER6_BASEADDR)
#define NVIC_ISER7					((__vo uint32_t*) NVIC_ISER7_BASEADDR)


/*** NVIC Clear-enable registers ***/
#define NVIC_ICER0					((__vo uint32_t*) NVIC_ICER0_BASEADDR)
#define NVIC_ICER1					((__vo uint32_t*) NVIC_ICER1_BASEADDR)
#define NVIC_ICER2					((__vo uint32_t*) NVIC_ICER2_BASEADDR)
#define NVIC_ICER3					((__vo uint32_t*) NVIC_ICER3_BASEADDR)
#define NVIC_ICER4					((__vo uint32_t*) NVIC_ICER4_BASEADDR)
#define NVIC_ICER5					((__vo uint32_t*) NVIC_ICER5_BASEADDR)
#define NVIC_ICER6					((__vo uint32_t*) NVIC_ICER6_BASEADDR)
#define NVIC_ICER7					((__vo uint32_t*) NVIC_ICER7_BASEADDR)


/*** NVIC Priority registers ***/
#define NVIC_IPR					((__vo uint32_t*) NVIC_IPR_BASEADDR)
/*** Number of interrupt priority bits implemented ***/
#define NO_IPR_BITS_VALID			4


/*** STM32F407 MCU SPECIFIC MACROS ***/
/*** Flash addresses of MCU memories, e.g. Flash, SRAM1, SRAM2, ROM ***/
#define FLASH_BASEADRR				0x08000000U // Sector 0 start
#define ROM_BASEADDR				0x1FFF0000U // System memory start
#define OTP_BASEADDR				0x1FFF7800U // OTP area start
#define OTPOPT_BASEADDR				0x1FFFC000U // OTP options - 16 bytes

#define SRAM1_BASEADDR				0x20000000U
#define SRAM2_BASEADDR				0x2001C000U // SRAM_BASEADDR + 112kb
#define SRAM						SRAM1_BASEADDR


/*** Bus peripheral base addresses ***/
#define APB1PERIPH_BASEADDR			0x40000000U
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASE				0x50000000U
#define PERIPH_BASEADDR				APB1PERIPH_BASEADDR


/*** AHB1 Bus peripherals ***/
#define GPIOA_BASEADDR				AHB1PERIPH_BASEADDR
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00U)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000U)

#define CRC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3000U)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800U)
#define DMA1_BASEADDR				(AHB1PERIPH_BASEADDR + 0x6000U)
#define DMA2_BASEADDR				(AHB1PERIPH_BASEADDR + 0x6400U)


/*** APB1 Bus peripherals ***/
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00U)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00U)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400U)
#define USART3 BASEADDR				(APB1PERIPH_BASEADDR + 0x4800U)

#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00U)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000U)
#define UART7_BASEADDR				(APB1PERIPH_BASEADDR + 0x7800U)
#define UART8_BASEADDR				(APB1PERIPH_BASEADDR + 0x7C00U)


/*** APB2 Bus peripherals ***/
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR				(APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR				(APB2PERIPH_BASEADDR + 0x5400)

#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)


/*** Peripheral register definitions ***/
/*** GPIO per-port configuration registers ***/
typedef struct {
	__vo uint32_t MODER;			// GPIO port mode; input/output/AF/analogue
	__vo uint32_t OTYPER;			// GPIO output type; push-pull/open-drain
	__vo uint32_t OSPEEDR;			// GPIO output speed; l/m/h/vh
	__vo uint32_t PUPDR;			// GPIO pullup-pulldown resistors
	__vo uint32_t IDR;				// GPIO input data
	__vo uint32_t ODR;				// GPIO output data
	__vo uint32_t BSRR;				// GPIO port bit set/reset
	__vo uint32_t LCKR;				// GPIO port configuration lock
	__vo uint32_t AFR[2];			// GPIO alternate function; AFRL/AFRH
} GPIO_RegDef_t;


/*** SPI system configuration registers ***/
typedef struct {
	__vo uint32_t CR1;				// SPI control register 1
	__vo uint32_t CR2;				// SPI control register 2
	__vo uint32_t SR;				// SPI status register
	__vo uint32_t DR;				// SPI data register
	__vo uint32_t CRCPR;			// SPI CRC polynomial register (not used in I2S mode)
	__vo uint32_t RXCRCR;			// SPI RX CRC register (not used in I2S mode)
	__vo uint32_t TXCRCR;			// SPI TX CRC register (not used in I2S mode)
	__vo uint32_t I2SCFGR;			// SPI_I2S configuration register
	__vo uint32_t I2SPR;			// SPI_I2S prescaler register
} SPI_RegDef_t;


/*** RCC configuration registers ***/
typedef struct {
	__vo uint32_t CR;				// RCC clock control register
	__vo uint32_t PLLCFGR;			// RCC PLL configuration register
	__vo uint32_t CFGR;				// RCC clock configuration register
	__vo uint32_t CIR;				// RCC clock interrupt register
	__vo uint32_t AHB1RSTR;			// RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;			// RCC AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;			// RCC AHB3 peripheral reset register
	uint32_t 	  RESERVED0;
	__vo uint32_t APB1RSTR;			// RCC APB1 peripheral reset register
	uint32_t      APB2RSTR;			// RCC APB2 peripheral reset register
	uint32_t 	  RESERVED1[2];
	__vo uint32_t AHB1ENR;			// RCC AHB1 peripheral enable register
	__vo uint32_t AHB2ENR;			// RCC AHB2 peripheral enable register
	__vo uint32_t AHB3ENR;			// RCC AHB3 peripheral enable register
	uint32_t      RESERVED2;
	__vo uint32_t APB1ENR;			// RCC APB1 peripheral enable register
	__vo uint32_t APB2ENR;			// RCC APB1 peripheral enable register
	uint32_t 	  RESERVED3[2];
	__vo uint32_t AHB1LPENR;		// RCC AHB1 peripheral clock enable in low power mode
	__vo uint32_t AHB2LPENR;		// RCC AHB2 peripheral clock enable in low power mode
	__vo uint32_t AHB3LPENR;		// RCC AHB3 peripheral clock enable in low power mode
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;		// RCC APB1 peripheral clock enable in low power mode
	__vo uint32_t APB12PENR;		// RCC APB2 peripheral clock enable in low power mode
	uint32_t      RESERVED5[2];
	__vo uint32_t BDCR;				// RCC Backup domain control register
	__vo uint32_t CSR;				// RCC clock control & status register
	uint32_t 	  RESERVED6[2];
	__vo uint32_t SSCGR;			// RCC spread spectrum clock generation register
	__vo uint32_t PLL12SCFGR;		// RCC PLLI2S configuration register
} RCC_RegDef_t;


/*** EXTI configuration registers ***/
typedef struct {
	__vo uint32_t IMR;				// Interrupt mask register
	__vo uint32_t EMR;				// Event mask register
	__vo uint32_t RTSR;				// Rising trigger selection register
	__vo uint32_t FTSR;				// Falling trigger selection register
	__vo uint32_t SWIER;			// Software interrupt event register
	__vo uint32_t PR;				// Pending register
} EXTI_RegDef_t;


/*** SYSCFG configuration registers ***/
typedef struct {
	__vo uint32_t MEMRMP;			// SYSCFG memory remap register
	__vo uint32_t PMC;				// SYSCFG peripheral mode configuration register
	__vo uint32_t EXTICR[4];		// SYSCFG external interrupt configuration register 1-4
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;			// Compensation cell control register
	uint32_t RESERVED2[3];
} SYSCFG_RegDef_t;


/*** Peripheral pointers ***/
/*** GPIO port register pointers ***/
#define GPIOA						((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG						((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH						((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI						((GPIO_RegDef_t*) GPIOI_BASEADDR)


/*** SPI register pointers ***/
#define SPI1						((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4						((SPI_RegDef_t*) SPI4_BASEADDR)
#define SPI5						((SPI_RegDef_t*) SPI5_BASEADDR)
#define SPI6						((SPI_RegDef_t*) SPI6_BASEADDR)


/*** RCC, EXTI registers pointers ***/
#define RCC							((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI						((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG						((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)


/*** Peripheral clock enable and disable macros ***/
/*** Clock enable macros for GPIOx (AHB1) ***/
#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()				(RCC->AHB1ENR |= (1 << 8))
/*** Clock disable macros for GPIOx ***/
#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 8))


/*** Clock enable macros for I2Cx (APB1) ***/
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1 << 23))
/*** Clock disable macros for I2Cx ***/
#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 23))


/*** Clock enable macros for SPIx (APB2, APB1) ***/
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()				(RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()				(RCC->APB2ENR |= (1 << 21))
/*** Clock disable macros for SPIx ***/
#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 21))


/*** Clock enable macros for USARTx (APB2, APB1) ***/
#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN()			(RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN()			(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1 << 5))
/*** Clock disable macros for USARTx ***/
#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define USART4_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 19))
#define USART5_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 5))


/*** Clock enable macro for SYSCFG (APB2) ***/
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))
/*** Clock disable macro for SYSCFG ***/
#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))


/*** GPIOx register reset macros ***/
#define GPIOA_REG_RESET()			do { (RCC->AHB1ENR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()			do { (RCC->AHB1ENR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()			do { (RCC->AHB1ENR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()			do { (RCC->AHB1ENR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()			do { (RCC->AHB1ENR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()			do { (RCC->AHB1ENR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()			do { (RCC->AHB1ENR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()			do { (RCC->AHB1ENR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()			do { (RCC->AHB1ENR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)


/*** IRQ positions of various EXTI ***/
#define IRQ_NO_EXT0					6
#define IRQ_NO_EXT1					7
#define IRQ_NO_EXT2					8
#define IRQ_NO_EXT3					9
#define IRQ_NO_EXT4					10
#define IRQ_NO_EXT9_5				23
#define IRQ_NO_EXT15_10				40


/*** NVIC IRQ priorities ***/
#define NVIC_IRQ_PRI0				0
#define NVIC_IRQ_PRI1				1
#define NVIC_IRQ_PRI2				2
#define NVIC_IRQ_PRI3				3
#define NVIC_IRQ_PRI4				4
#define NVIC_IRQ_PRI5				5
#define NVIC_IRQ_PRI6				6
#define NVIC_IRQ_PRI7				7
#define NVIC_IRQ_PRI8				8
#define NVIC_IRQ_PRI9				9
#define NVIC_IRQ_PRI10				10
#define NVIC_IRQ_PRI11				11
#define NVIC_IRQ_PRI12				12
#define NVIC_IRQ_PRI13				13
#define NVIC_IRQ_PRI14				14
#define NVIC_IRQ_PRI15				15


/*** Other macros ***/
#define ENABLED 					1
#define DISABLED					0
#define SET							ENABLED
#define RESET						DISABLED
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET


/*** Return 4-bit code for GPIO port address ***/
#define GPIO_BASEADDR_TO_CODE(x) 	((x == GPIOA) ? 0 : \
									(x == GPIOB) ? 1 : \
									(x == GPIOC) ? 2 : \
									(x == GPIOD) ? 3 : \
									(x == GPIOE) ? 4 : \
									(x == GPIOF) ? 5 : \
									(x == GPIOG) ? 6 : \
									(x == GPIOH) ? 7 : \
									(x == GPIOI) ? 8 : 0)


#include "stm32f407_gpio_driver.h"
#endif /* INC_STM32F407_DRIVERS_H_ */
