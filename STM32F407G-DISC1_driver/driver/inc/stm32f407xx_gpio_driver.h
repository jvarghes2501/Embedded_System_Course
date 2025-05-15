/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Apr 17, 2025
 *      Author: jesti
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * configuration structure for GPIO pin that the user can control
 */
typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPull_up_down_control;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * handler structure for GPIO pin
 */
typedef struct{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

#define GPIO_PIN_SET 	1
#define GPIO_PIN_RESET 	0

/*
 * GPIO pin input modes
 */
# define GPIO_MODE_IN 		0
# define GPIO_MODE_OUT 		1
# define GPIO_MODE_ALTFN 	2
# define GPIO_MODE_ANALOG 	3

# define GPIO_MODE_INTPT_FALLING_TRIG			4
# define GPIO_MODE_INTPT_RISING_TRIG			5
# define GPIO_MODE_INTPT_RISING_FALLING_TRIG	6

/*
 * GPIO pin output modes
 */
#define GPIO_OUT_TYPE_PP 			0
#define GPIO_OUT_TYPE_OD			1

/*
 * GPIO pin output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * GPIO pin pull up and pull down configurations
 */
# define GPIO_NO_PUPD		0
# define GPIO_PIN_PU		1
# define GPIO_PIN_PD		2


/*
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0 		0
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



/*
 * Define API function prototypes for GPIO
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DiInit(GPIO_RegDef_t *pGPIOx);

void GPIO_ClockControl(GPIO_RegDef_t *pGPIOx, uint8_t Enable_or_disable);

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t value );
void GPIO_WritToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t Enable_or_disable);
void GPIO_PriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
