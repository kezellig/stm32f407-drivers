/*
 * stm32f407_gpio_driver.c
 * Author: Kaitlyn W.
 */

#include "stm32f407_gpio_driver.h"

// Driver's GPIO APIs

/**
 * Enable or disable the peripheral clock for a GPIO port
 * @param: Base address of GPIO port
 * @param: Whether the clock is enabled or disabled - ENABLE or DISABLE macros
 */
void GPIO_PClockControl(GPIO_RegDef_t *p_GPIOx, uint8_t EnDi) {
	// Replace this stuff with a function pointer array in the future
	if (EnDi == ENABLED) {
		if (p_GPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (p_GPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (p_GPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (p_GPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (p_GPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (p_GPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (p_GPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (p_GPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (p_GPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	} else {
		if (p_GPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (p_GPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (p_GPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (p_GPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (p_GPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (p_GPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (p_GPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (p_GPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (p_GPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}
}


/**
 * Initialises a GPIO port
 * @param: A GPIO Handle containing GPIO configs and GPIO port base address
 */
void GPIO_Init(GPIO_Handle_t *p_GPIOHandle) {
	uint32_t temp = 0;
	if (p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// Modes from [0, 3] are non-interrupting
		// Retrieve mode: each pin takes two bit fields, hence multiply pin no. by 2
		temp = (p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		// Clear the register bits first (00) by negating 11
		// Store our mode in MODER - port mode register
		p_GPIOHandle->p_GPIOx->MODER &= ~(0x3 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		p_GPIOHandle->p_GPIOx->MODER |= temp;

	} else {

	}
	temp = 0;

	// Speed config - 2 bit fields
	temp = (p_GPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	p_GPIOHandle->p_GPIOx->OSPEEDR &= ~(0x3 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	p_GPIOHandle->p_GPIOx->OSPEEDR |= temp;
	temp = 0;

	// PUPD config -  2 bit fields
	temp = (p_GPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	p_GPIOHandle->p_GPIOx->PUPDR &= ~(0x3 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	p_GPIOHandle->p_GPIOx->PUPDR |= temp;
	temp = 0;

	// Clear register bit (0) by negating 1
	// Output type config - 1 bit field
	temp = (p_GPIOHandle->GPIO_PinConfig.GPIO_PinOPType << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	p_GPIOHandle->p_GPIOx->OTYPER &= ~(0x1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	p_GPIOHandle->p_GPIOx->OTYPER |= temp;
	temp = 0;

	// Alternate function config
	if (p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		// Do something with alt. fun. registers
		uint32_t temp1, temp2;

		temp1 = p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		p_GPIOHandle->p_GPIOx->AFR[temp1] &= ~(0xF << (4 *temp2));
		p_GPIOHandle->p_GPIOx->AFR[temp1] |= (p_GPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 *temp2));
	}
}

/**
 * Resets a GPIO port
 * @param: Base address of a GPIO port
 */
void GPIO_DeInit(GPIO_RegDef_t *p_GPIOx) {
	// Reset all registers of the GPIO port

	if (p_GPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (p_GPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (p_GPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (p_GPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (p_GPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (p_GPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (p_GPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (p_GPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (p_GPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}

}


/**
 * Read value from a GPIO pin
 * @param: Base address of GPIO port
 * @param: Number of pin to read from
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *p_GPIOx, uint8_t PinNumber) {
	uint8_t value;
	value = (uint8_t)((p_GPIOx->IDR >> PinNumber) & 0x00000001 ); // Move the value to LSB position
	return value; // 0 or 1
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *p_GPIOx) {
	uint16_t values;
	values = (uint16_t)p_GPIOx->IDR; // Simply return the entire register
	return values;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *p_GPIOx, uint8_t PinNumber, uint8_t Value) {
	if (Value == GPIO_PIN_SET) { // Write 1 for that pin
		p_GPIOx->ODR |= (1 << PinNumber);
	} else { // Clear
		p_GPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *p_GPIOx, uint16_t Value) {
	p_GPIOx->ODR = Value; // Write into the whole port
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *p_GPIOx, uint8_t PinNumber) {
	p_GPIOx->ODR = p_GPIOx->ODR ^ (1 << PinNumber);
}


// Interrupt configuration and handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnDi) {

}
void GPIO_IRQHandling(uint8_t PinNumber) {

}
