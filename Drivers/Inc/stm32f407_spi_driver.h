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


/*** SPI APIs ***/
/*** Enable/disable peripheral clock for SPI ***/
void SPI_PClockControl(SPI_RegDef_t *p_SPIx, uint8_t EnDi);


/*** Initialise/reset SPI peripheral ***/
void SPI_Init(SPI_Handle_t *p_SPIHandle);
void SPI_DeInit(SPI_RegDef_t *p_SPIx);


/*** Read/write using SPI ***/
void SPI_Send(SPI_RegDef_t *p_SPIx, uint8_t p_TxBuffer, uint32_t Length);
void SPI_Receive(SPI_RegDef_t *p_SPIx, uint8_t p_RxBuffer, uint32_t Length);


/*** Interrupt configuration and handling ***/
void SPI_IRQActivationConfig(uint8_t IRQNumber, uint8_t EnDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *p_SPIHandle);


#endif /* INC_STM32F407_SPI_DRIVER_H_ */
