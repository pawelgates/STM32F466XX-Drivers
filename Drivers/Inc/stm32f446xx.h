/* STM32F446RE MCU HEADER FILE*/

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

/*************** Processor specific details **************/
/* Arm Cortex Mx Processor NVIC ISERx register addresses */

#define NVIC_ISER0			((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1			((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2			((volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3			((volatile uint32_t*) 0xE000E10C)

/* Arm Cortex Mx Processor NVIC ICERx register addresses */

#define NVIC_ICER0			((volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1			((volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2			((volatile uint32_t*) 0xE000E188)
#define NVIC_ICER3			((volatile uint32_t*) 0xE000E18C)

/* Arm Cortex Mx Processor Priority register addresses */

#define NVIC_PR_BASEADDR	((volatile uint32_t*) 0xE000E400)

#define NO_PR_BITS_IMPLEMENTED		4

/* GENERIC */

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET 		RESET

/* Base addresses of FLASH and SRAM memories */

#define FLASH_BASEADDR 		0x08000000U
#define SRAM1_BASEADDR 		0x20000000U
#define SRAM2_BASEADDR 		0x2001C000U /* SRAM2 = SRAM1 + 112KB */
#define ROM_BASEADDR 		0x1FFF0000U
#define SRAM 				SRAM1_BASEADDR

/* Base addresses of BUS domains */

#define APB1_BASEADDR 		0x40000000U
#define APB2_BASEADDR 		0x40010000U
#define AHB1_BASEADDR 		0x40020000U
#define AHB2_BASEADDR 		0x50000000U

/* Base addresses of AHB1 bus peripherals - GPIO */

#define GPIOA_BASEADDR 		(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 		(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 		(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 		(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 		(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 		(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 		(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 		(AHB1_BASEADDR + 0x1C00)
#define RCC_BASEADDR 		(AHB1_BASEADDR + 0x3800)

/* Base addresses of APB1 bus peripherals - I2C/SPI/UART/USART */

#define I2C1_BASEADDR 		(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR 		(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR 		(APB1_BASEADDR + 0x5C00)
#define SPI2_BASEADDR 		(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR 		(APB1_BASEADDR + 0x3C00)
#define USART2_BASEADDR 	(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR 	(APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR 		(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR 		(APB1_BASEADDR + 0x5000)

/* Base addresses of APB2 bus peripherals - SPI/USART/EXTI/SYSCFG */

#define SPI1_BASEADDR 		(APB2_BASEADDR + 0x3000)
#define USART1_BASEADDR 	(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR 	(APB2_BASEADDR + 0x1400)
#define EXTI_BASEADDR 		(APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR 	(APB2_BASEADDR + 0x3800)

/* CLOCK ENABLE MACROS */

#define GPIOA_CLK_EN() 		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN() 		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN() 		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN() 		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN() 		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN() 		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN() 		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN() 		(RCC->AHB1ENR |= (1 << 7))

#define I2C1_CLK_EN() 		(RCC->APB1ENR |= (1 << 21))
#define I2C2_CLK_EN() 		(RCC->APB1ENR |= (1 << 22))
#define I2C3_CLK_EN() 		(RCC->APB1ENR |= (1 << 23))

#define SPI1_CLK_EN() 		(RCC->APB2ENR |= (1 << 12))
#define SPI2_CLK_EN() 		(RCC->APB1ENR |= (1 << 14))
#define SPI3_CLK_EN() 		(RCC->APB1ENR |= (1 << 15))

#define USART1_CLK_EN() 	(RCC->APB2ENR |= (1 << 4))
#define USART2_CLK_EN() 	(RCC->APB1ENR |= (1 << 17))
#define USART3_CLK_EN() 	(RCC->APB1ENR |= (1 << 18))
#define UART4_CLK_EN() 		(RCC->APB1ENR |= (1 << 19))
#define UART5_CLK_EN() 		(RCC->APB1ENR |= (1 << 20))
#define USART6_CLK_EN() 	(RCC->APB2ENR |= (1 << 5))

#define SYSCFG_CLK_EN() 	(RCC->APB2ENR |= (1 << 14))


/* CLOCK DISABLE MACROS */

#define GPIOA_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DI() 		(RCC->AHB1ENR &= ~(1 << 7))

#define I2C1_CLK_DI() 		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DI() 		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DI() 		(RCC->APB1ENR &= ~(1 << 23))

#define SPI1_CLK_DI() 		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_CLK_DI() 		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DI() 		(RCC->APB1ENR &= ~(1 << 15))

#define USART1_CLK_DI() 	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_CLK_DI() 	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_CLK_DI() 	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_CLK_DI() 		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_CLK_DI() 		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_CLK_DI() 	(RCC->APB2ENR &= ~(1 << 5))

#define SYSCFG_CLK_DI() 	(RCC->APB2ENR &= ~(1 << 14))


/* GPIO RESET MACROS */

#define GPIOA_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)	// Set bit and clear bit function
#define GPIOB_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)	// Set bit and clear bit function
#define GPIOC_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)	// Set bit and clear bit function
#define GPIOD_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)	// Set bit and clear bit function
#define GPIOE_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)	// Set bit and clear bit function
#define GPIOF_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)	// Set bit and clear bit function
#define GPIOG_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)	// Set bit and clear bit function
#define GPIOH_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)	// Set bit and clear bit function


/* GPIO port base address to 4 bit code */

#define GPIO_BASEADDR_TO_CODE(x)		(	(x == GPIOA) ? 0 : \
											(x == GPIOB) ? 1 : \
											(x == GPIOC) ? 2 : \
											(x == GPIOD) ? 3 : \
											(x == GPIOE) ? 4 : \
											(x == GPIOF) ? 5 : \
											(x == GPIOG) ? 6 : \
											(x == GPIOH) ? 7 : 0	)

/* IRQ - Interrupt Request numbers (positions in vector table) */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40


/* GPIO register definition structure*/

typedef struct
{
	volatile uint32_t MODER;	/* GPIO port mode register */
	volatile uint32_t OTYPER;	/* GPIO port output type register */
	volatile uint32_t OSPEEDER; /* GPIO port output speed register */
	volatile uint32_t PUPDR;	/* GPIO port pull-up/pull-down register */
	volatile uint32_t IDR;		/* GPIO port input data register */
	volatile uint32_t ODR;		/* GPIO port output data register */
	volatile uint32_t BSRR;		/* GPIO port bit set/reset register */
	volatile uint32_t LCKR;		/* GPIO port configuration lock register */
	volatile uint32_t AFR[2];	/* GPIO alternate function low register and high register */

} GPIO_RegDef_t;

/* RCC register definition structure*/

typedef struct
{
	volatile uint32_t CR;		  /* RCC clock control register */
	volatile uint32_t PLLCFGR;	  /* RCC PLL configuration register */
	volatile uint32_t CFGR;		  /* RCC clock configuration register */
	volatile uint32_t CIR;		  /* RCC clock interrupt register */
	volatile uint32_t AHB1RSTR;	  /* RCC AHB1 peripheral reset register */
	volatile uint32_t AHB2RSTR;	  /* RCC AHB2 peripheral reset register */
	volatile uint32_t AHB3RSTR;	  /* RCC AHB3 peripheral reset register */
	volatile uint32_t RESERVED0;  /*  */
	volatile uint32_t APB1RSTR;	  /* RCC APB1 peripheral reset register */
	volatile uint32_t APB2RSTR;	  /* RCC APB2 peripheral reset register */
	volatile uint32_t RESERVED1;  /*  */
	volatile uint32_t RESERVED2;  /*  */
	volatile uint32_t AHB1ENR;	  /* RCC AHB1 peripheral clock enable register */
	volatile uint32_t AHB2ENR;	  /* RCC AHB2 peripheral clock enable register */
	volatile uint32_t AHB3ENR;	  /* RCC AHB3 peripheral clock enable register */
	volatile uint32_t RESERVED3;  /*  */
	volatile uint32_t APB1ENR;	  /* RCC APB1 peripheral clock enable register */
	volatile uint32_t APB2ENR;	  /* RCC APB2 peripheral clock enable register */
	volatile uint32_t RESERVED4;  /*  */
	volatile uint32_t RESERVED5;  /*  */
	volatile uint32_t AHB1LPENR;  /* RCC AHB1 peripheral clock enable in low power mode register */
	volatile uint32_t AHB2LPENR;  /* RCC AHB2 peripheral clock enable in low power mode register */
	volatile uint32_t AHB3LPENR;  /* RCC AHB3 peripheral clock enable in low power mode register */
	volatile uint32_t RESERVED6;  /*  */
	volatile uint32_t APB1LPENR;  /* RCC APB1 peripheral clock enable in low power mode register */
	volatile uint32_t APB2LPENR;  /* RCC APB2 peripheral clock enabled in low power mode register */
	volatile uint32_t RESERVED7;  /*  */
	volatile uint32_t RESERVED8;  /*  */
	volatile uint32_t BDCR;		  /* RCC Backup domain control register */
	volatile uint32_t CSR;		  /* RCC clock control & status register */
	volatile uint32_t RESERVED9;  /*  */
	volatile uint32_t RESERVED10; /*  */
	volatile uint32_t SSCGR;	  /* RCC spread spectrum clock generation register */
	volatile uint32_t PLLI2SCFGR; /* RCC PLLI2S configuration register */
	volatile uint32_t PLLSAICFGR; /* RCC PLL configuration register */
	volatile uint32_t DCKCFGR;	  /* RCC dedicated clock configuration register */
	volatile uint32_t CKGATENR;	  /* RCC clocks gated enable register */
	volatile uint32_t DCKCFGR2;	  /* RCC dedicated clocks configuration register 2 */

} RCC_RegDef_t;


/* RCC register definition structure*/

typedef struct
{
	volatile uint32_t IMR;			/* Interrupt mask register */
	volatile uint32_t EMR;			/* Event mask register */
	volatile uint32_t RTSR; 		/* Rising trigger selection register */
	volatile uint32_t FTSR;			/* Falling trigger selection register */
	volatile uint32_t SWIER;		/* Software interrupt event register */
	volatile uint32_t PR;			/* Pending register */

} EXTI_RegDef_t;

/* SYSCFG register definition structure*/

typedef struct
{
	volatile uint32_t MEMRMP;		/* SYSCFG memory re-map register */
	volatile uint32_t PMC;			/* SYSCFG peripheral mode configuration register */
	volatile uint32_t EXTICR[4]; 	/* SYSCFG external interrupt configuration register 1-4 */
	volatile uint32_t RESERVED0[2];	/* */
	volatile uint32_t CMPCR;		/* Compensation cell control register */
	volatile uint32_t RESERVED1[2];	/* */
	volatile uint32_t CFGR;			/* SYSCFG configuration register */

} SYSCFG_RegDef_t;

/* Peripheral base addresses type casted to XXX_RegDef_t */

#define GPIOA 			((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB 			((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC 			((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD 			((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE 			((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF 			((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG 			((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH 			((GPIO_RegDef_t *)GPIOH_BASEADDR)

#define RCC 			((RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI			((EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG			((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)



#include "stm32f446xx_gpio_driver.h"

#endif /* INC_STM32F446XX_H_ */
