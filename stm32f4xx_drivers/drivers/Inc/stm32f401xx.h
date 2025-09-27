/*
 * This header file contains addresses of Flash, SRAM, and all the necessary peripherals i.e. all the MCU specific data
 *
 *
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>

//Generic Macros
#define ENABLE 						1
#define DISABLE 					0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET

//Base Addresses of Flash and SRAM
#define FLASH_BASEADDR				0x08000000U			//Base Address of Flash Memory, refer datasheet Pg.51
#define SRAM_BASEADDR				0x20000000U			//Base Address of SRAM, refer datasheet Pg.51
#define ROM_BASEADDR				0x1FFF0000U 		//ST factory bootloader location  refer datasheet Pg.51

//AHBx and APBx Bus Peripheral base addresses (Refer Pg.52 of datasheet)
#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x50000000U


//Base Addresses of Peripherals mapped to AHB1 bus (Refer Pg.52 of datasheet, also note: this MCU does not have ports F and G)
#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)

//Base Addresses of Peripherals mapped to APB1 bus (Refer Pg.52 of datasheet)
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)


//Base Addresses of Peripherals mapped to APB2 bus (Refer Pg.52 of datasheet)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400)

#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)

/*
 * Peripheral Structure Definitions
 */

//GPIO Port Structure Definition
typedef struct{
	volatile uint32_t MODER;		// GPIO Port mode register
	volatile uint32_t OTYPER;		// GPIO Port output type register
	volatile uint32_t OSPEEDR;		// GPIO port output speed register
	volatile uint32_t PUPDR;		// GPIO port pull-up/pull-down register
	volatile uint32_t IDR;			// GPIO port input data register
	volatile uint32_t ODR;			// GPIO port output data register
	volatile uint32_t BSRR;			// GPIO port bit set/reset register
	volatile uint32_t LCKR;			// GPIO port configuration lock register
	volatile uint32_t AFR[2];		// GPIO alternate function register

}GPIO_RegDef_t;


//RCC Peripheral Structure Definition
typedef struct{
	volatile uint32_t CR;			// RCC clock control register
	volatile uint32_t PLLCFGR;		// RCC PLL configuration register
	volatile uint32_t CFGR;			// RCC clock configuration register
	volatile uint32_t CIR;			// RCC clock interrupt register
	volatile uint32_t AHB1RSTR;		// RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;		// RCC AHB2 peripheral reset register
			 uint32_t RESERVED0[2];	// Reserved
	volatile uint32_t APB1RSTR;		// RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;		// RCC APB2 peripheral reset register
			 uint32_t RESERVED1[2]; // Reserved
	volatile uint32_t AHB1ENR;		// RCC AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;		// RCC AHB2 peripheral clock enable register
			 uint32_t RESERVED2[2];	// Reserved
	volatile uint32_t APB1ENR;		// RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;		// RCC APB2 peripheral clock enable register
			 uint32_t RESERVED3[2];	// Reserved
	volatile uint32_t AHB1LPENR;	// RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;	// RCC AHB2 peripheral clock enable in low power mode register
			 uint32_t RESERVED4[2]; // Reserved
	volatile uint32_t APB1LPENR;	// RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;	// RCC APB2 peripheral clock enable in low power mode register
			 uint32_t RESERVED5[2]; // Reserved
	volatile uint32_t BDCR;			// RCC Backup domain control register
	volatile uint32_t CSR;			// RCC clock control & status register
			 uint32_t RESERVED6[2]; // Reserved
	volatile uint32_t SSCGR;		// RCC spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;	// RCC PLLI2S configuration register
			 uint32_t RESERVED7[2]; // Reserved
	volatile uint32_t DCKCFGR;		// RCC Dedicated Clocks Configuration Register
}RCC_RegDef_t;


/*
 * Peripheral definitions (Peripheral Base Addresses typecasted to peripheral_RegDef_t)
 */
#define GPIOA 						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH 						((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals, refer 6.3.9 RCC AHB1 peripheral clock Enable Register in RM0368
 */
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))


/*
 * Clock Enable Macros for I2Cx peripherals, refer 6.3.11 RCC APB1 peripheral clock Enable Register in RM0368
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))


/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 7))



/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))


/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))


/*
 * Clock Disable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))



#endif /* INC_STM32F401XX_H_ */
