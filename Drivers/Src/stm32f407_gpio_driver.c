/*
 * stm32f407_gpio_driver.c
 * Author: Kaitlyn W.
 */

#include "stm32f407_gpio_driver.h"


/*** DRIVER'S GPIO APIs ***/
/**
 * Enable or disable the peripheral clock for a GPIO port
 * @param p_GPIOx: base address of GPIO port
 * @param EnDi: whether the clock is enabled or disabled
 * @returns: none
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
 * @param: GPIO handle containing GPIO configs and GPIO port base address
 */
void GPIO_Init(GPIO_Handle_t *p_GPIOHandle) {
	uint32_t temp = 0;

	// Check if pin mode is in non-interrupt mode
	if (p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOGUE) {
		// Store the pin mode (0x0 - 0x3) into the proper slot for that pin number
		// Each pin takes up 2 bit fields, so multiply pin number by 2 before shifting
		temp = (p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		// Clear the selected bits in the pin mode port register first
		p_GPIOHandle->p_GPIOx->MODER &= ~(0b11 << (2 * p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// Actually store the mode in the MODER register
		p_GPIOHandle->p_GPIOx->MODER |= temp;

	} else {
		// Enable rising/falling trigger detection
		if (p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			// Enable rising trigger detection for pin
			EXTI->RTSR |= (1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT) {
			// Enable falling trigger detection for pin
			EXTI->FTSR |= (1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else {
			EXTI->RTSR |= (1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// Select source input for EXTI lines
		uint8_t reg_num, pin_num;
		reg_num = p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		pin_num = p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		uint8_t port_code = GPIO_BASEADDR_TO_CODE(p_GPIOHandle->p_GPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[reg_num] |= (port_code << (pin_num * 4));

		// Unmask interrupt request on pin bit
		EXTI->IMR |= (1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	// Speed select - 2 bit fields
	temp = (p_GPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	p_GPIOHandle->p_GPIOx->OSPEEDR &= ~(0b11 << (2 * p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	p_GPIOHandle->p_GPIOx->OSPEEDR |= temp;
	temp = 0;

	// PUPD select -  2 bit fields
	temp = (p_GPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	p_GPIOHandle->p_GPIOx->PUPDR &= ~(0b11 << (2 * p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	p_GPIOHandle->p_GPIOx->PUPDR |= temp;
	temp = 0;

	// Output type select - 1 bit field
	if (p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_OUT) {
		temp = (p_GPIOHandle->GPIO_PinConfig.GPIO_PinOPType << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		p_GPIOHandle->p_GPIOx->OTYPER &= ~(1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		p_GPIOHandle->p_GPIOx->OTYPER |= temp;
		temp = 0;
	}

	// Alternate functionality select - 4 bit fields
	// Configure AF if GPIO_PinMode is indeed set to AF
	// Use [0] if pin low [0, 7], use [1] if pin high [8, 15]
	if (p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		uint8_t temp1, temp2;

		temp1 = p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; // AF low or high register
		temp2 = p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; // Pin location in the register

		// Clear the 4 bit fields first then write into AFR register
		p_GPIOHandle->p_GPIOx->AFR[temp1] &= ~(0b1111 << (4 * temp2));
		p_GPIOHandle->p_GPIOx->AFR[temp1] |= (p_GPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/**
 * Resets all pins on a GPIO port
 * @param p_GPIOx: base address of a GPIO port
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
 * Read the value from a GPIO pin
 * @param p_GPIOx: base address of GPIO port
 * @param PinNumber: number of pin to read from
 * @returns: read bit 0 or 1
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *p_GPIOx, uint8_t PinNumber) {
	uint8_t value;
	// Right shift relevant bit in IDR to LSB position then test if 0 or 1
	value = (uint8_t)((p_GPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


/**
 * Read values from a GPIO port
 * @param p_GPIOx: base address of GPIO port
 * @returns: read bits
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *p_GPIOx) {
	uint16_t values;
	values = (uint16_t)p_GPIOx->IDR; // Simply return the entire register
	return values;
}


/**
 * Write value to a GPIO pin
 * @param p_GPIOx: base address of GPIO port
 * @param PinNumber: number of the pin to write to
 * @param Value: write bit 0 or 1
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *p_GPIOx, uint8_t PinNumber, uint8_t Value) {
	if (Value == GPIO_PIN_SET) {
		p_GPIOx->ODR |= (1 << PinNumber);
	} else {
		p_GPIOx->ODR &= ~(1 << PinNumber);
	}
}


/**
 * Write value to a GPIO pin
 * @param p_GPIOx: base address of GPIO port
 * @param Value: write bits 0 or 1
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *p_GPIOx, uint16_t Value) {
	p_GPIOx->ODR = Value; // Write into the whole port
}


/**
 * Toggle value of a GPIO pin
 * @param p_GPIOx: base address of GPIO port
 * @param PinNumber: number of the pin to write to
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *p_GPIOx, uint8_t PinNumber) {
	p_GPIOx->ODR ^= (1 << PinNumber);

}


/**
 * Enable or disable an interrupt
 * @param IRQNumber: number of the interrupt
 * @param EnDi: enable or disable
 */
void GPIO_IRQActivationConfig(uint8_t IRQNumber, uint8_t EnDi) {
	uint8_t irq_num = IRQNumber % 32;
	if (EnDi) { // Enable interrupt using ISERs depending on IRQNumber
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << irq_num);
		} else if (IRQNumber <= 64) {
			*NVIC_ISER1 |= (1 << irq_num);
		} else {
			*NVIC_ISER2 |= (1 << irq_num);
		}
	} else { // Disable interrupt
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= (1 << irq_num);
		} else if (IRQNumber <= 64) {
			*NVIC_ICER1 |= (1 << irq_num);
		} else {
			*NVIC_ICER2 |= (1 << irq_num);
		}
	}
}


/**
 * Configure the priority of an interrupt
 * @param IRQNumber: number of the interrupt
 * @param IRQPriority: priority to set the interrupt
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t reg_num = IRQNumber / 4;
	uint8_t irq_num = IRQNumber % 4;

	// Account for unimplemented lower 4 bits in register
	uint8_t shift_num = irq_num * 8 + (8 - NO_IPR_BITS_VALID);
	*(NVIC_IPR + reg_num) |= (IRQPriority << shift_num);
}


/**
 * Handle an interrupt event
 * The default interrupt handler is just an infinite loop
 * @param PinNumber: number of the pin to clear interrupt trigger for
 */
void GPIO_IRQHandling(uint8_t PinNumber) {
	// Clear EXTI pending register's pin number bit
	if (EXTI->PR & (1 << PinNumber)) {
		EXTI->PR |= (1 << PinNumber);
	}

}
