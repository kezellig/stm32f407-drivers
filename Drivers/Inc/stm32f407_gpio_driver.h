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

// GPIO pin handle - address and config settings
typedef struct
{
	GPIO_RegDef_t *p_GPIOx; // Holds base address of the GPIO port to which pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; // Holds GPIO pin config settings
} GPIO_Handle_t;


// Driver's GPIO APIs
// Peripheral clock setup
void GPIO_PClockControl(GPIO_RegDef_t *p_GPIOx, uint8_t EnDi);


// Init/DeInit
void GPIO_Init(GPIO_Handle_t *p_GPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t *p_GPIOx); // We don't need additional configs to reset, just address


// Data R/W
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *p_GPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *p_GPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *p_GPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *p_GPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *p_GPIOx, uint8_t PinNumber);


// Interrupt configuration and handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnDi);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407_GPIO_DRIVER_H_ */
