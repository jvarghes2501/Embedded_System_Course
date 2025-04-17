/*
 * STM32F407xx.h
 *
 *  Created on: Apr 17, 2025
 *      Author: jesti
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include <stdint.h>

/*
 * Flash and SRAM memory base addresses
 */
# define FLASH_BASEADDR		0x08000000U
# define SRAM1_BASEADDR		0x20000000U
# define SRAM2_BASEADDR		0x20001C00U
# define ROM_BASEADDR		0x1FFF0000U
# define SRAM				SRAM1_BASEADDR



/*
 * AHBx and APBx bus peripheral base addresses
 */
# define APB1PERIPH_BASEADDR	0x40000000U
# define APB2PERIPH_BASEADDR	0x40010000U
# define AHB1PERIPH_BASEADDR	0x40020000U
# define AHB2PERIPH_BASEADDR	0x50000000U


/*
 * Peripherals base addresses attached to AHB1 bus
 */
# define GPIOA_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0000)
# define GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0400)
# define GPIOC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0800)
# define GPIOD_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0C00)
# define GPIOE_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1000)
# define GPIOF_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1400)
# define GPIOG_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1800)
# define GPIOH_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1C00)
# define GPIOI_BASEADDR		(AHB1PERIPH_BASEADDR + 0x2000)
# define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)
/*
 * Peripherals base addresses attached to the APB1 bus
 */
# define I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0x5400)
# define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0x5800)
# define I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0x5C00)

# define SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800)
# define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00)

# define USART2_BASEADDR	(APB1PERIPH_BASEADDR + 0x4400)
# define USART3_BASEADDR	(APB1PERIPH_BASEADDR + 0x4800)
# define UART4_BASEADDR		(APB1PERIPH_BASEADDR + 0x4C00)
# define UART5_BASEADDR		(APB1PERIPH_BASEADDR + 0x5000)


/*
 * Peripherals base addresses attached to the APB2 bus
 */
# define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0x3C00)
# define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0x3000)
# define SYSCFG_BASEADDR	(APB2PERIPH_BASEADDR + 0x3800)
# define USART1_BASEADDR	(APB2PERIPH_BASEADDR + 0x1000)
# define USART6_BASEADDR	(APB2PERIPH_BASEADDR + 0x1400)


/*
 * Peripheral structure definitions
 */

typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
}GPIO_RegDef_t;


typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;
}RCC_RegDef_t;

/*
 * Peripheral definitions
 */

# define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
# define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
# define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
# define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
# define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
# define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
# define GPIOG		((GPIO_RegDef_t*)GPIOG_BASEADDR)
# define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)
# define GPIOI		((GPIO_RegDef_t*)GPIOI_BASEADDR)

# define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)


/*
 * Enable the clock for GPIOx peripherals
 */
#define GPIOA_CLOCK_ENABLE()	(RCC->AHB1ENR |= (1<<0))
#define GPIOB_CLOCK_ENABLE()	(RCC->AHB1ENR |= (1<<1))
#define GPIOC_CLOCK_ENABLE()	(RCC->AHB1ENR |= (1<<2))
#define GPIOD_CLOCK_ENABLE()	(RCC->AHB1ENR |= (1<<3))
#define GPIOE_CLOCK_ENABLE()	(RCC->AHB1ENR |= (1<<4))
#define GPIOF_CLOCK_ENABLE()	(RCC->AHB1ENR |= (1<<5))
#define GPIOG_CLOCK_ENABLE()	(RCC->AHB1ENR |= (1<<6))
#define GPIOH_CLOCK_ENABLE()	(RCC->AHB1ENR |= (1<<7))
#define GPIOI_CLOCK_ENABLE()	(RCC->AHB1ENR |= (1<<8))

/*
 * Enable the clock for I2Cx peripherals
 */
#define I2C1_CLOCK_ENABLE()	(RCC->APB1ENR |= (1<<21))
#define I2C2_CLOCK_ENABLE()	(RCC->APB1ENR |= (1<<22))
#define I2C3_CLOCK_ENABLE()	(RCC->APB1ENR |= (1<<23))


/*
 * Enable the clock for SPIx peripherals
 */
#define SPI1_CLOCK_ENABLE()	(RCC->APB2ENR |= (1<<12))
#define SPI2_CLOCK_ENABLE()	(RCC->APB1ENR |= (1<<14))
#define SPI3_CLOCK_ENABLE()	(RCC->APB1ENR |= (1<<15))
#define SPI4_CLOCK_ENABLE()	(RCC->APB2ENR |= (1<<13))


/*
 * Enable the clock for USARTx peripherals
 */
# define USART1_CLOCK_ENABLE()	(RCC->APB2ENR|=(1<<4))
# define USART2_CLOCK_ENABLE()	(RCC->APB1ENR|=(1<<17))
# define USART3_CLOCK_ENABLE()	(RCC->APB1ENR|=(1<<18))
# define USART6_CLOCK_ENABLE()	(RCC->APB1ENR|=(1<<5))

# define UART4_CLOCK_ENABLE()	(RCC->APB1ENR|=(1<<19))
# define UART5_CLOCK_ENABLE()	(RCC->APB1ENR|=(1<<20))

/*
 * Enable the clock for SYSCFG peripheral
 */
# define SYSCFG_CLOCK_ENABLE()	(RCC->APB2ENR|=(1<<14))



/*
 * Disable the clock for GPIOx peripherals
 */
#define GPIOA_CLOCK_DISABLE()	(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_CLOCK_DISABLE()	(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_CLOCK_DISABLE()	(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_CLOCK_DISABLE()	(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_CLOCK_DISABLE()	(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_CLOCK_DISABLE()	(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_CLOCK_DISABLE()	(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_CLOCK_DISABLE()	(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_CLOCK_DISABLE()	(RCC->AHB1ENR &= ~(1<<8))

/*
 * Disable the clock for I2Cx peripherals
 */
#define I2C1_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1<<21))
#define I2C2_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1<<22))
#define I2C3_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1<<23))


/*
 * Disable the clock for SPIx peripherals
 */
#define SPI1_CLOCK_DISABLE()	(RCC->APB2ENR &= ~(1<<12))
#define SPI2_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1<<14))
#define SPI3_CLOCK_DISABLE()	(RCC->APB1ENR &= ~(1<<15))
#define SPI4_CLOCK_DISABLE()	(RCC->APB2ENR &= ~(1<<13))


/*
 * Disable the clock for USARTx peripherals
 */
# define USART1_CLOCK_DISABLE()	(RCC->APB2ENR&=~(1<<4))
# define USART2_CLOCK_DISABLE()	(RCC->APB1ENR&=~(1<<17))
# define USART3_CLOCK_DISABLE()	(RCC->APB1ENR&=~(1<<18))
# define USART6_CLOCK_DISABLE()	(RCC->APB1ENR&=~(1<<5))

# define UART4_CLOCK_DISABLE()	(RCC->APB1ENR&=~(1<<19))
# define UART5_CLOCK_DISABLE()	(RCC->APB1ENR&=~(1<<20))

/*
 * Disable the clock for SYSCFG peripheral
 */
# define SYSCFG_CLOCK_DISABLE()	(RCC->APB2ENR&=~(1<<14))

//Generic macros
#define ENABLE 		1
#define DISABLE 	0
#define SET 		1
#define RESET		0

#endif /* INC_STM32F407XX_H_ */
