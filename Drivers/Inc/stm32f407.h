/*
 * stm32f407.h
 * Author: Kaitlyn W.
 */

#ifndef INC_STM32F407_DRIVERS_H_
#define INC_STM32F407_DRIVERS_H_

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


#endif /* INC_STM32F407_DRIVERS_H_ */
