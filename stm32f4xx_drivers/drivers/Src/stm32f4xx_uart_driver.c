/*
 * stm32f4xx_uart_driver.c
 *
 */


#include "stm32f401xx_usart_driver.h"



/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempReg=0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	USART_PeripheralClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempReg|= (1 << USART_CR1_TXEN);
	}

	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempReg |= ( 1 << USART_CR1_RXEN);

	}

	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempReg |= ( ( 1 << USART_CR1_TXEN) | ( 1 << USART_CR1_RXEN) );
	}

    //Implement the code to configure the Word length configuration item
	tempReg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enale the parity control
		tempReg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		tempReg &= ~(1 << USART_CR1_PS);
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
		tempReg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
		tempReg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempReg;

/******************************** Configuration of CR2******************************************/

	tempReg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempReg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempReg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempReg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempReg |= (1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= (( 1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
	}


	pUSARTHandle->pUSARTx->CR3 = tempReg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here

}

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



/*********************************************************************
 * @fn      		  - UART_Reset
 *
 * @brief             - This function resets a UART Peripheral
 *
 * @param[in]         - pointer to a UART handle
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void USART_Reset(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}
	else if(pUSARTx == USART6)
	{
		USART6_REG_RESET();
	}
}


/*********************************************************************
 * @fn      		  - UART_GetFlagStatus
 *
 * @brief             - This function returns the status of a flag in the UART status register
 *
 * @param[in]         - pointer to a UART handle
 * @param[in]         - check for set or reset
 * @return            - flag status(set or reset)
 *
 * @Note              -  none

 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
	if(pUSARTx->SR & StatusFlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}



/*********************************************************************
 * @fn      		  - UART_ClearFlag
 *
 * @brief             - This function clears a flag in the UART status register
 *
 * @param[in]         - pointer to a UART handle
 * @param[in]         -
 * @return            - none
 *
 * @Note              -  none

 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~( StatusFlagName);

}
