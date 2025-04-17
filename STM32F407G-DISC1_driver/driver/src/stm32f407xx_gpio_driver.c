/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Apr 17, 2025
 *      Author: jesti
 */
# include "stm32f407xx_gpio_driver.h"

void GPIO_ClockControl(GPIO_RegDef_t *pGPIOx, uint8_t Enable_or_disable){
	if (Enable_or_disable==ENABLE){
		if (pGPIOx == GPIOA){
			GPIOA_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOB){
			GPIOB_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOC){
			GPIOC_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOD){
			GPIOD_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOE){
			GPIOE_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOF){
			GPIOF_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOG){
			GPIOG_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOH){
			GPIOH_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOI){
			GPIOI_CLOCK_ENABLE();
		}
	}else{
		if (pGPIOx == GPIOA){
			GPIOA_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOB){
			GPIOB_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOC){
			GPIOC_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOD){
			GPIOD_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOE){
			GPIOE_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOF){
			GPIOF_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOG){
			GPIOG_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOH){
			GPIOH_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOI){
			GPIOI_CLOCK_DISABLE();
		}

	}
}
