/*
 * This header file contains addresses of Flash, SRAM, and all the necessary peripherals i.e. all the MCU specific data
 *
 *
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>


/*************************************** Cortex M4 Specific macros definitions *******************************************/

/*
 * ARM Cortex-M4 NVIC ISERx(Interrupt Set-enable Register) Addresses, Refer Pg. 4-3 of Cortex M4 user guide
 * TODO: Look into moving this to a structure definition
 */
#define NVIC_ISER0					((volatile uint32_t*)0xE000E100) 			//Address offset 0xE100
#define NVIC_ISER1					((volatile uint32_t*)0xE000E104) 			//Address offset 0xE104
#define NVIC_ISER2					((volatile uint32_t*)0xE000E108) 			//Address offset 0xE108
#define NVIC_ISER3					((volatile uint32_t*)0xE000E10C) 			//Address offset 0xE10C

/*
 * ARM Cortex-M4 NVIC ICERx(Interrupt Clear-enable Register) Addresses, Refer Pg. 4-3 of Cortex M4 user guide
 * TODO: Look into moving this to a structure definition
 */
#define NVIC_ICER0					((volatile uint32_t*)0xE000E180) 			//Address offset 0xE180
#define NVIC_ICER1					((volatile uint32_t*)0xE000E184) 			//Address offset 0xE184
#define NVIC_ICER2					((volatile uint32_t*)0xE000E188) 			//Address offset 0xE188
#define NVIC_ICER3					((volatile uint32_t*)0xE000E18C) 			//Address offset 0xE18C


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation, Refer Pg. 4-3 of Cortex M4 user guide
 * There are total of 60 (0 - 59) Interrupt priority registers. It is easier to program using pointer to the base address
 */
#define NVIC_PR_BASEADDR			((volatile uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

//Generic Macros
#define ENABLE 						1
#define DISABLE 					0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define FLAG_SET					SET
#define FLAG_RESET					RESET

#define HIGH 1
#define LOW	 0
#define BTN_PRESSED					LOW
#define BTN_RELEASED				HIGH

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
#define GPIOA_BASEADDR			    (AHB1PERIPH_BASEADDR + 0x0000)
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
 * Peripheral Structure Definitions. Some of the registers are updated every clock cycle so it is important to declare them as volatile
 */

//GPIO Port Structure Definition
typedef struct
{
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


//SPI Structure Definition
typedef struct
{
	volatile uint32_t CR1;			// SPI control register 1 				Address offset: 0x00
	volatile uint32_t CR2;			// SPI control register 2				Address offset: 0x04
	volatile uint32_t SR;			// SPI status register					Address offset: 0x08
	volatile uint32_t DR;			// SPI data register					Address offset: 0x0C
	volatile uint32_t CRCPR;		// SPI CRC polynomial register 			Address offset: 0x10
	volatile uint32_t RXCRCR;		// SPI RX CRC register					Address offset: 0x14
	volatile uint32_t TXCRCR;		// SPI TX CRC register					Address offset: 0x18
	volatile uint32_t I2SCFGR;		// SPI_I2S configuration register		Address offset: 0x1C
	volatile uint32_t I2SPR;		// SPI_I2S prescaler register			Address offset: 0x20
}SPI_RegDef_t;


//UART Structure Definition
typedef struct
{
	volatile uint32_t USART_SR;		//USART status register									Address offset: 0x00
	volatile uint32_t USART_DR;		//USART data register									Address offset: 0x04
	volatile uint32_t USART_BRR;	//USART baud rate register								Address offset: 0x08
	volatile uint32_t USART_CR1;	//USART control register 1								Address offset: 0x0C
	volatile uint32_t USART_CR2;	//USART control register 2								Address offset: 0x10
	volatile uint32_t USART_CR3;	//USART control register 3								Address offset: 0x14
	volatile uint32_t USART_GTPR;	//USART guard time and prescaler register				Address offset: 0x18
}USART_RegDef_t;




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


//EXTI Peripheral Structure definition
typedef struct
{
	volatile uint32_t IMR;		// EXTI Interrupt Mask Register
	volatile uint32_t EMR;		// EXTI Event Mask Register
	volatile uint32_t RTSR;		// EXTI Rising Trigger Selection Register
	volatile uint32_t FTSR;		// EXTI Falling Trigger Selection Register
	volatile uint32_t SWIER;	// EXTI Software Interrupt Event Register
	volatile uint32_t PR;		// EXTI Pending Register
}EXTI_RegDef_t;

//SYSCFG Peripheral Structure definition
typedef struct
{
	volatile uint32_t MEMRMP; 					// SYSCFG memory remap register
	volatile uint32_t PMC;						// SYSCFG peripheral mode configuration register
	volatile uint32_t EXTICR[4];				// SYSCFG external interrupt configuration registers 1-4
	volatile uint32_t CMPCR;					// Compensation cell control register
}SYSCFG_RegDef_t;


/*
 * Peripheral definitions (Peripheral Base Addresses typecasted to peripheral_RegDef_t)
 */
#define GPIOA 						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH 						((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define SPI1						((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 						((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 						((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4 						((SPI_RegDef_t*)SPI4_BASEADDR)

#define USART1						((USART_RegDef_t*)USART1_BASEADDR)
#define USART2						((USART_RegDef_t*)USART2_BASEADDR)
#define USART6						((USART_RegDef_t*)USART6_BASEADDR)

#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI						((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
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

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()			do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB2RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB2RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()			do{(RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)

/*
 * Macros to reset USARTx peripherals
 */
#define USART1_REG_RESET()			do{(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));}while(0)
#define USART2_REG_RESET()			do{(RCC->APB2RSTR |= (1 << 17)); (RCC->APB2RSTR &= ~(1 << 17));}while(0)
#define USART6_REG_RESET()			do{(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5));}while(0)

/*
 * returns port code for a given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :0)


/*
 * IRQ numbers for STM32F401RE, refer Pg. 203 of the reference manual for the vector table
 */
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40

/*
 * Macros for all the possible priority levels(note: the STM32F4xx family only supports upto numerical 15 due to the 4 bit constraint)
 */
#define NVIC_IRQ_PRI0				0
#define NVIC_IRQ_PRI1				1
#define NVIC_IRQ_PRI2				2
#define NVIC_IRQ_PRI3				3
#define NVIC_IRQ_PRI4				4
#define NVIC_IRQ_PRI5				5
#define NVIC_IRQ_PRI6				6
#define NVIC_IRQ_PRI7				7
#define NVIC_IRQ_PRI8				8
#define NVIC_IRQ_PRI9				9
#define NVIC_IRQ_PRI10				10
#define NVIC_IRQ_PRI11				11
#define NVIC_IRQ_PRI12				12
#define NVIC_IRQ_PRI13				13
#define NVIC_IRQ_PRI14				14
#define NVIC_IRQ_PRI15				15


/************************* Bit Position Defintions for SPI ****************************************/

/*
 * Bit position defintions for SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15


/*
 * Bit positions definitions for SPI_CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * Bit positions definitions for SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


/************************* Bit Position Defintions for UART ****************************************/

/*
 * Bit position defintions for USART_CR1
 */
#define USART_CR1_REWAKEUP	1
#define USART_CR1_RXEN		2
#define USART_CR1_TXEN		3
#define USART_CR1_RXNE		5
#define USART_CR1_TXE		7
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15

/*
 * Bit position defintions for USART_CR2
 */
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12

/*
 * Bit position defintions for USART_CR3
 */
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10



//these includes are for the main applications files so that a single include of "stm32f401xx" gives you everything related to the MCU
#include "stm32f401xx_gpio_driver.h"
#include "stm32f401xx_spi_driver.h"
#include "stm32f401xx_usart_driver.h"

#endif /* INC_STM32F401XX_H_ */
