/*
 * stm32f401xx_gpio_driver.c
 *
 *
 */

#include "stm32f401xx_gpio_driver.h"


/*
 * Init and Reset
 */

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes a GPIO port
 *
 * @param[in]         - pointer to a GPIO handle
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

}

/*********************************************************************
 * @fn      		  - GPIO_Reset
 *
 * @brief             - This function resets a GPIO port
 *
 * @param[in]         - pointer to a GPIO handle
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_Reset(GPIO_RegDef_t *pGPIOx){

}

/*
 * Peripheral Clock Setup
 */
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDI == ENABLE){
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}

	}
	else{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}

	}

}

/*
 * Data read and write
 */
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads the state of a pin on a specific GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number of the specific GPIO port
 *
 * @return            - state of the pin, high or low
 *
 * @Note              -  none

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - this function reads data for an entire specific GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 *
 * @return            -  none
 *
 * @Note              -  each port is of 16 pins so the return type is uint16_t

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function sets or resets the state of a pin on a specific GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - specific pin on the GPIO pin number
 * @param[in]		  -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function sets or resets the state of a specific GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - value to set for the entire port
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles the state of a specific GPIO pin on a speicific port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number of a specific pin
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

}

/*
 * IRQ configuration and ISR handling
 */

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi){

}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQHandling(uint8_t PinNumber){

}
