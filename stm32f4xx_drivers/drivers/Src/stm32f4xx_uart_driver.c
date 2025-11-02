/*
 * stm32f4xx_uart_driver.c
 *
 */


#include "stm32f401xx_usart_driver.h"





/*
 * Peripheral Clock Setup
 */
/*********************************************************************
 * @fn      		  - USART_PeriPheralClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given UART port
 *
 * @param[in]         - base address of the UART peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void USART_PeripheralClockControl(USART_RegDef_t *pUSARTx, uint8_t State)
{
	if(State == ENABLE){
		if(pUSARTx == USART1){
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}

	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}


}
