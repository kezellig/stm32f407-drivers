/*
 * stm32f407_spi_driver.c
 *
 *  Created on: May 16, 2024
 *      Author: kaitlyn
 */

#include "stm32f407.h"


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
 * Resets an SPI peripheral
 * @param p_SPIx: base address of SPI peripheral
 */
void SPI_DeInit(SPI_RegDef_t *p_SPIx) {
	// Reset all registers of the GPIO port

	if (p_SPIx == SPI1) {
		SPI1_RESET();
	} else if (p_SPIx == SPI2) {
		SPI2_RESET();
	} else if (p_SPIx == SPI3) {
		SPI3_RESET();
	}
}
