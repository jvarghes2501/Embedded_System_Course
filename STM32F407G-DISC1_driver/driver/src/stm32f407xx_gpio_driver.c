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

void GPIO_Init (GPIO_Handle_t * pGPIOHandle){
	// enable the clock
	GPIO_ClockControl(pGPIOHandle->pGPIOx, ENABLE);
	uint32_t temp=0;
	//1) configure the mode of the GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &=~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing the bits
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else{
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INTPT_FALLING_TRIG){
			//1) configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear the rising trigger bit

		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INTPT_RISING_TRIG)
		{
			//2) configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear the falling trigger bit
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INTPT_RISING_FALLING_TRIG)
		{
			// configure the FTSR & RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2) configure the GPIO port selection and SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portCode;
		if (pGPIOHandle->pGPIOx == GPIOA){
			portCode = 0;
		}else if (pGPIOHandle->pGPIOx == GPIOB){
			portCode = 1;
		}else if (pGPIOHandle->pGPIOx == GPIOC){
			portCode = 2;
		}else if (pGPIOHandle->pGPIOx == GPIOD){
			portCode = 3;
		}else if (pGPIOHandle->pGPIOx == GPIOE){
			portCode = 4;
		}else if (pGPIOHandle->pGPIOx == GPIOF){
			portCode = 5;
		}else if (pGPIOHandle->pGPIOx == GPIOG){
			portCode = 6;
		}else if (pGPIOHandle->pGPIOx == GPIOH){
			portCode = 7;
		}


		SYSCFG_CLOCK_ENABLE();
		SYSCFG->EXTICR[temp1] |= portCode << (temp2 * 4);


		//3) enable the EXTI interrupt delivery using IMR (interrupt mask register)
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	//2) configure the GPIO speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &=~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing the bits
	pGPIOHandle->pGPIOx->OSPEEDR|= temp;
	temp = 0;

	//3) configure the pull up and pull down pins
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPull_up_down_control<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &=~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing the bits
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4) configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &=~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing the bits
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//configure the alternative functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN)
	{
		//configure the alternate function registers
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;

		if (temp1 == 0){
			pGPIOHandle->pGPIOx->AFRL &= ~(0xF << (4*temp2));  //clearing
			pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
		}
		else {
			pGPIOHandle->pGPIOx->AFRH &= ~(0xF << (4*temp2));  //clearing
			pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));

		}
	}
}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t Enable_or_disable){
	if (Enable_or_disable == ENABLE){
		if(IRQNumber <=31){
			//change the ISER0 reg
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if (IRQNumber > 31 && IRQNumber < 64){
			// change the ISER1 reg
			*NVIC_ISER1 |= (1 << (IRQNumber%32));

		}else if (IRQNumber >= 64 && IRQNumber < 96){
			// change the ISER2 reg
			*NVIC_ISER2 |= (1 << (IRQNumber%64));

		}
	}else{
		if(IRQNumber <=31){
			//change the ICER0 reg
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if (IRQNumber > 31 && IRQNumber < 64){
			// change the ICER1 reg
			*NVIC_ICER1 |= (1 << (IRQNumber%32));

		}else if (IRQNumber >= 64 && IRQNumber < 96){
			// change the ICER2 reg
			*NVIC_ICER2 |= (1 << (IRQNumber%64));

		}

	}
}
void GPIO_PriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << ((8*iprx_section)+(8-PR_BITS_IMPLEMENTED)));
}

void GPIO_IRQHandling(uint8_t PinNumber){
	// clear the exti pr register bit to the corresponding pin number
	if (EXTI->PR & (1<<PinNumber)){
		EXTI->PR |= (1<<PinNumber);
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if (pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if (pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}


uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber)&0x00000001); //right shift the bit position to the LSB and then mask it to get the value
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t value ){
	if (value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1<<PinNumber); //write 1
	}

	else{
		pGPIOx->ODR &= ~(1<<PinNumber); //write 0
	}
}

void GPIO_WritToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t value){
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR = pGPIOx->ODR ^ (1<<PinNumber);
}
