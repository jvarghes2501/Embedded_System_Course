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

/*
 * Define API function prototypes for GPIO
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DiInit(GPIO_RegDef_t *pGPIOx);

void GPIO_ClockControl(GPIO_RegDef_t *pGPIOx, uint8_t Enable_or_disable);

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInpurPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t value );
void GPIO_WritToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t Enable_or_disable);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
