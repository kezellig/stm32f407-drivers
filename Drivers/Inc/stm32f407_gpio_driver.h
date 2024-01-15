/*
 * stm32f407_gpio_driver.h
 * Author: Kaitlyn W.
 */

#ifndef INC_STM32F407_GPIO_DRIVER_H_
#define INC_STM32F407_GPIO_DRIVER_H_

#include "stm32f407.h"

// Configuration properties of a GPIO pin in a struct
typedef struct
{
	uint8_t GPIO_PinNumber; // 0-15 available
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl; // Pullup/Pulldown
	uint8_t GPIO_PinOPType; // Output type
	uint8_t GPIO_PinAltFunMode; // Alternative function mode
} GPIO_PinConfig_t;

// A GPIO pin handle - address and config settings
typedef struct
{
	GPIO_RegDef_t *p_GPIOx; // Holds base address of the GPIO port to which pin belongs
	GPIO_PinConfig_t GPIO_PinConfig // Holds GPIO pin config settings

} GPIO_Handle_t;


#endif /* INC_STM32F407_GPIO_DRIVER_H_ */
