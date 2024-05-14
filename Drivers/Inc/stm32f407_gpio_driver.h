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
	uint8_t GPIO_PinNumber; 		// Values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			// Values from @GPIO_PIN_MODES
	uint8_t GPIO_PinOPType; 		// Values from @GPIO_OP_TYPE
	uint8_t GPIO_PinSpeed;			// Values from @GPIO_OP_SPEEDS
	uint8_t GPIO_PinPuPdControl; 	// Values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinAltFunMode; 	// Alternative function mode
} GPIO_PinConfig_t;


/*** GPIO pin handle - change port settings + pin settings ***/
typedef struct
{
	GPIO_RegDef_t *p_GPIOx; // Holds pin's port base address
	GPIO_PinConfig_t GPIO_PinConfig; // Holds pin's settings
} GPIO_Handle_t;


/**
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/**
 * @GPIO_PIN_MODES
 * GPIO pin modes as defined by GPIOx_MODER register
 * Note: GPIO_MODE_IT are special interrupt modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOGUE 	3
#define GPIO_MODE_IT_FT  	4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6


/**
 * @GPIO_OP_TYPE
 * GPIOx_OTYPER - GPIO output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1


/**
 * @GPIO_OP_SPEEDS
 * GPIO output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


/**
 * @GPIO_PIN_PUPD
 * GPIO pin pullup and pulldown
 */
#define GPIO_PIN_NO_PUPD	0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/*** GPIO APIs ***/
/*** Enable/disable peripheral clock for a port ***/
void GPIO_PClockControl(GPIO_RegDef_t *p_GPIOx, uint8_t EnDi);


/*** Initialise/reset port ***/
void GPIO_Init(GPIO_Handle_t *p_GPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t *p_GPIOx);


/*** Read/write from pin or port ***/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *p_GPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *p_GPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *p_GPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *p_GPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *p_GPIOx, uint8_t PinNumber);


// Interrupt configuration and handling
// Configue IRQ number of the GPIO pin, handling->user app calls IRQ handling to process interrupt (ISR)
void GPIO_IRQActivationConfig(uint8_t IRQNumber, uint8_t EnDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407_GPIO_DRIVER_H_ */
