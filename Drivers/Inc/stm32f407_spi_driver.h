/*
 * stm32f407_spi_driver.h
 * Author: Kaitlyn W.
 */

#ifndef INC_STM32F407_SPI_DRIVER_H_
#define INC_STM32F407_SPI_DRIVER_H_

#include "stm32f407.h"

/*** User configuration struct of SPI properties ***/
typedef struct {
	uint8_t SPI_DeviceMode; 			// Values from @SPI_DEVICE_MODE
	uint8_t SPI_BusMode; 				// Values from @SPI_BUS_MODE
	uint8_t SPI_DFSize;					// Values from @SPI_DFF
	uint8_t SPI_ClockPhase; 			// Values from @SPI_CPHA
	uint8_t SPI_ClockPolarity; 			// Values from @SPI_CPOL
	uint8_t SPI_SlaveManagement; 		// Values from @SPI_SSM
	uint8_t SPI_ClockSpeed;				// Values from @SPI_CLKSPEED
} SPI_Config_t;


/*** SPI handle ***/
typedef struct {
	SPI_RegDef_t *p_SPIx; // SPI peripheral's base address
	SPI_Config_t SPI_Config; // User-defined SPI settings
} SPI_Handle_t;


/*** SPI CONSTANTS ***/
/**
 * @SPI_DEVICE_MODE
 */
#define SPI_DEVICE_MODE_SLAVE		0
#define SPI_DEVICE_MODE_MASTER		1


/**
 * @SPI_BUS_MODE
 */
#define	SPI_BUS_MODE_FD				0	// Full-duplex
#define SPI_BUS_MODE_HD				1	// Half-duplex
#define SPI_BUS_MODE_S_TX			2	// Simplex, transmit only
#define SPI_BUS_MODE_S_RX			3	// Simplex, receive only


/**
 * @SPI_DFF
 * Data frame size used in communication
 */
#define SPI_DFF_BITS8				0
#define SPI_DFF_BITS16				1


/**
 * @SPI_CPHA
 */
#define	SPI_CPHA_FIRST				0
#define SPI_CPHA_SECOND				1


/**
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW				0	// Clock idle state low
#define SPI_CPOL_HIGH				1	// Clock idle state high


/**
 * @SPI_SSM
 */
#define SPI_SSM_SW					0
#define SPI_SSM_HW					1


/**
 * @SPI_CLKSPEED
 * Speed of the SPI serial clock, determined by peripheral clock dividing by a prescaler
 * The user may select the prescaler value, with minimum value 2
 */
#define SPI_CLKSPEED_DIV2			0
#define SPI_CLKSPEED_DIV4			1
#define SPI_CLKSPEED_DIV8			2
#define SPI_CLKSPEED_DIV16			3
#define SPI_CLKSPEED_DIV32			4
#define SPI_CLKSPEED_DIV64			5
#define SPI_CLKSPEED_DIV128			6
#define SPI_CLKSPEED_DIV256			7


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
