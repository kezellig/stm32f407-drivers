/*
 * stm32f407_spi_driver.c
 *
 *  Created on: May 16, 2024
 *      Author: kaitlyn
 */

#include "stm32f407.h"


/*** HELPER FUNCTIONS ***/



/*** DRIVER'S GPIO APIs ***/
/**
 * Enable or disable the peripheral clock for an SPI peripheral
 * @param p_SPIx: base address of SPI peripheral
 * @param EnDi: whether the clock is enabled or disabled
 */
void SPI_PClockControl(SPI_RegDef_t *p_SPIx, uint8_t EnDi) {
	if (EnDi == ENABLED) {
		if (p_SPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (p_SPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (p_SPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if (p_SPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (p_SPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (p_SPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}


/**
 * Initialises an SPI peripheral
 * @param: SPI handle containing SPI base address and SPI user configurations
 */
void SPI_Init(SPI_Handle_t *p_SPIHandle) {
	uint32_t temp_cr1 = 0;

	// Device mode
	temp_cr1 |= p_SPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	// Duplex, half duplex, etc.
	if (p_SPIHandle->SPI_Config.SPI_BusMode == SPI_BUS_MODE_FD) {
		temp_cr1 &= ~(1 << SPI_CR1_BIDIMODE); // Full duplex is unidirectional

	} else if (p_SPIHandle->SPI_Config.SPI_BusMode == SPI_BUS_MODE_HD) {
		temp_cr1 |= (1 << SPI_CR1_BIDIMODE); // Half duplex is bidrectional

	} else if (p_SPIHandle->SPI_Config.SPI_BusMode == SPI_BUS_MODE_S_TX) {
		temp_cr1 |= (1 << SPI_CR1_BIDIMODE); // Simplex is unidirectional

	} else if (p_SPIHandle->SPI_Config.SPI_BusMode == SPI_BUS_MODE_S_RX) {
		temp_cr1 &= ~(1 << SPI_CR1_BIDIMODE); // Simplex is unidirectional
		temp_cr1 |= ~(1 << SPI_CR1_RXONLY); // Receive only
	}

	// Dataframe size
	temp_cr1 |= (p_SPIHandle->SPI_Config.SPI_DFSize << SPI_CR1_DFF);

	// Clock phase
	temp_cr1 |= (p_SPIHandle->SPI_Config.SPI_ClockPhase << SPI_CR1_CPHA);

	// Clock polarity
	temp_cr1 |= (p_SPIHandle->SPI_Config.SPI_ClockPolarity << SPI_CR1_CPOL);

	// Slave management
	temp_cr1 |= (p_SPIHandle->SPI_Config.SPI_SlaveManagement << SPI_CR1_SSM);

	// Clock speed
	temp_cr1 |= (p_SPIHandle->SPI_Config.SPI_ClockSpeed << SPI_CR1_BR);

	p_SPIHandle->p_SPIx->CR1 = temp_cr1;
}


/**
 * Resets an SPI peripheral
 * @param p_SPIx: base address of SPI peripheral
 */
void SPI_DeInit(SPI_RegDef_t *p_SPIx) {
	if (p_SPIx == SPI1) {
		SPI1_RESET();
	} else if (p_SPIx == SPI2) {
		SPI2_RESET();
	} else if (p_SPIx == SPI3) {
		SPI3_RESET();
	}
}


/**
 * Send data using SPI
 * @param p_SPIx: base address of SPI peripheral
 * @param p_TxBuffer: address of SPI TX buffer
 * @param Length: length of data to send
 */
void SPI_Send(SPI_RegDef_t *p_SPIx, uint8_t p_TxBuffer, uint32_t Length) {
	while(Length > 0) {
		// Wait until TXE bitfield is empty (1)
		while (!(p_SPIx->SR & (1 << SPI_SR_TXE)));

		// Check if DFF is 8 or 16 bits
		if (!(p_SPIx->CR1 & (1 << SPI_CR1_DFF))) { // 0 bit = 8 bits

		} else {

		}

	}
}
