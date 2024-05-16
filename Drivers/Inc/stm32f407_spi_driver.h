/*
 * stm32f407_spi_driver.h
 * Author: Kaitlyn W.
 */

#ifndef INC_STM32F407_SPI_DRIVER_H_
#define INC_STM32F407_SPI_DRIVER_H_

#include "stm32f407.h"

/*** User configuration struct of SPI properties ***/
typedef struct {
	uint8_t SPI_DeviceMode; // m/s?
	uint8_t SPI_BusMode; // duplex etc
	uint8_t SPI_DFSize;
	uint8_t SPI_ClockPhase; //CPHA
	uint8_t SPI_ClockPolarity; // CPOL
	uint8_t SPI_SlaveManagement; //sw or hw select
	uint8_t SPI_Speed;
} SPI_Config_t;


/*** SPI handle ***/
typedef struct {
	SPI_RegDef_t *p_SPIx;		// SPI peripheral's base address
	SPI_Config_t SPI_Config;	// User-defined SPI settings
} SPI_Handle_t;

#endif /* INC_STM32F407_SPI_DRIVER_H_ */
