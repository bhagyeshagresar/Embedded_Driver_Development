/*
 * stm32f401xx_gpio_drivers.h
 *
 */
#ifndef STM32F401XX_GPIO_DRIVERS_H_
#define STM32F401XX_GPIO_DRIVERS_H_

#include "stm32f401xx.h"

/*
 * This is a configuration structure for a GPIO pin
 */
typedef struct{
	uint8_t GPIO_PinNumber;					/*Possible values from @GPIO_PIN_NUMBERS*/
	uint8_t GPIO_PinMode;					/*Possible values from @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;					/*Possible values from @GPIO_PIN_OUTPUT_SPEEDS*/
	uint8_t GPIO_PinPuPdControl;			/*Possible values from @GPIO_PIN_PU_PD_MODES*/
	uint8_t GPIO_PinOPType;					/*Possible values from @GPIO_PIN_OUTPUT_TYPES*/
	uint8_t GPIO_PinAltFundMode;			/*Possible values from @GPIO_PIN_ALTFN_MODES*/
}GPIO_PinConfig_t;


/*
 * Handle Structure for a GPIO Pin
 */
typedef struct{
	GPIO_RegDef_t *pGPIOx;					/*!<This holds the base address of the GPIO Port to which the pin belongs>*/
	GPIO_PinConfig_t GPIO_PinConfig; 		/*!<This holds the GPIO pin configuration settings>*/

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NUM_0		0
#define GPIO_PIN_NUM_1		1
#define GPIO_PIN_NUM_2		2
#define GPIO_PIN_NUM_3		3
#define GPIO_PIN_NUM_4		4
#define GPIO_PIN_NUM_5		5
#define GPIO_PIN_NUM_6		6
#define GPIO_PIN_NUM_7		7
#define GPIO_PIN_NUM_8		8
#define GPIO_PIN_NUM_9		9
#define GPIO_PIN_NUM_10		10
#define GPIO_PIN_NUM_11		11
#define GPIO_PIN_NUM_12		12
#define GPIO_PIN_NUM_13		13
#define GPIO_PIN_NUM_14		14
#define GPIO_PIN_NUM_15		15


/*
 * @GPIO_PIN_MODES
 * GPIO Pin Possible Modes, refer 8.4.1 GPIO Port mode register in RM0368 for the different modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4				//interrupt mode falling edge trigger
#define	GPIO_MODE_IT_RT		5				//interrupt mode rising edge trigger
#define	GPIO_MODE_IT_RFT	6				//interrupt mode rising edge + falling edge trigger

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO Pin Output types, refer 8.4.2 GPIO port output type register in RM0368
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_PIN_OUTPUT_SPEEDS
 * GPIO pin possible output speeds, refer 8.4.3 GPIO port output speed register in RM0368
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_HIGH		2
#define GPIO_SPEED_VHIGH	3

/*
 * @GPIO_PIN_PU_PD_MODES
 * GPIO pin pull up and pull down macros, refer 8.4.4 GPIO port pull-up/pull-down register
 */
#define GPIO_NO_PUPD 		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/****************************************
 * APIs supported by this driver
 *
 ****************************************/

/*
 * Init and Reset
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Reset(GPIO_RegDef_t *pGPIOx);

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx); 								/*Note: each port is of 16 pins so the return type is uint16_t*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* STM32F401XX_GPIO_DRIVERS_H_ */
