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
 * @Note              - The procedure for this is given in the RM0368 character transmission and reception section of USART peripheral

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
		tempReg|= (1 << USART_CR1_RXEN);
	}

	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempReg |= ( 1 << USART_CR1_TXEN);

	}

	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_RXTX)
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
	pUSARTHandle->pUSARTx->USART_CR1 = tempReg;

/******************************** Configuration of CR2******************************************/

	tempReg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempReg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->USART_CR2 = tempReg;

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
		tempReg |= (( 1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
	}


	pUSARTHandle->pUSARTx->USART_CR3 = tempReg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

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
	if(pUSARTx->USART_SR & StatusFlagName)
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
	pUSARTx->USART_SR &= ~( StatusFlagName);

}







/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - send a number of bytes in blocking mode
 *
 * @param[in]         - pointer to a UART handle
 * @param[in]         - pointer to transmit buffer
 * @param[in]         - number of bytes to send
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata; //used if the word length is 9 bits
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->USART_DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer += 2;

			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->USART_DR = *pTxBuffer;

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - Receive data in blocking mode
 *
 * @param[in]         - pointer to a UART handle
 * @param[in]         - pointer to software receive buffer
 * @param[in]         - number of bytes to send
 *
 * @return            - none
 *
 * @Note              -

 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		While(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->USART_DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer += 2;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->USART_DR & 0x00FF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->USART_DR & 0x007F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             - Send data in non blocking mode
 *
 * @param[in]         - pointer to a UART handle
 * @param[in]         - pointer to transmit buffer
 * @param[in]         - number of bytes to send
 *
 * @return            -
 *
 * @Note              -

 */
uint8_t USART_SendDataWithIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX; // setting this bit to ready will be done in IRQHandling

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TXEIE); // 1: An USART interrupt is generated whenever TXE=1 in the USART_SR register


		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TCIE); // 1: An USART interrupt is generated whenever TC=1 in the USART_SR register

	}

	return txstate;

}


/*********************************************************************
 * @fn      		  - USART_ReceiveWithDataIT
 *
 * @brief             - Receive data in nonblocking mode
 *
 * @param[in]         - pointer to a UART handle
 * @param[in]         - pointer to software receive buffer
 * @param[in]         - number of bytes to send
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_ReceiveWithDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX; // setting this bit to ready will be done in IRQHandling

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_RXNE);

	}

	return rxstate;

}






/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         - pointer to UART register definition
 * @param[in]         - desired baudrate
 *
 * @return            -
 *
 * @Note              -

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempReg=0;

	  //Get the value of APB bus clock in to the variable PCLKx
	  if(pUSARTx == USART1 || pUSARTx == USART6)
	  {
		   //USART1 and USART6 are hanging on APB2 bus
		   PCLKx = RCC_GetPCLK2Value();
	  }else
	  {
		   PCLKx = RCC_GetPCLK1Value();
	  }

	  //Check for OVER8 configuration bit
	  if(pUSARTx->USART_CR1 & (1 << USART_CR1_OVER8))
	  {
		   //OVER8 = 1 , over sampling by 8
		   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	  }else
	  {
		   //over sampling by 16
		   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
	  }

	  //Calculate the Mantissa part
	  M_part = usartdiv/100;

	  //Place the Mantissa part in appropriate bit position . refer USART_BRR
	  tempreg |= M_part << 4;

	  //Extract the fraction part
	  F_part = (usartdiv - (M_part * 100));

	  //Calculate the final fractional
	  if(pUSARTx->USART_CR1 & (1 << USART_CR1_OVER8))
	   {
		  //OVER8 = 1 , over sampling by 8
		  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

	   }else
	   {
		   //over sampling by 16
		   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

	   }

	  //Place the fractional part in appropriate bit position . refer USART_BRR
	  tempReg |= F_part;

	  //copy the value of tempreg in to BRR register
	  pUSARTx->USART_BRR = tempReg;
}

