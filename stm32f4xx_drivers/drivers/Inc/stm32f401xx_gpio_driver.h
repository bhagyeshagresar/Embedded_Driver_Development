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
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFundMode;
}GPIO_PinConfig_t;


/*
 * Handle Structure for a GPIO Pin
 */
typedef struct{
	GPIO_RegDef_t *pGPIOx;					/*!<This holds the base address of the GPIO Port to which the pin belongs>*/
	GPIO_PinConfig_t GPIO_PinConfig; 		/*!<This holds the GPIO pin configuration settings>*/

}GPIO_Handle_t;


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
