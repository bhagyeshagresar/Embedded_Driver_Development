/*
 * stm32f401xx_uart_driver.h
 *
 */

#ifndef INC_STM32F401XX_UART_DRIVERS_H_
#define INC_STM32F401XX_UART_DRIVERS_H_

#include "stm32f401xx.h"




/*
 * Configuration structure for UART peripheral
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;

}USART_Config_t;


/*
 * Handle structure for UART peripheral
 */
typedef struct
{
	USART_RegDef_t* pUSARTx;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;


/*
 * @USART_Mode
 */
#define USART_MODE_ONLY_TX	0
#define USART_MODE_ONLY_RX	1
#define USART_MODE_RXTX		2

/*
 * @USART_Baud
 * For 16MHZ the max aud rate that can be achieved is 3MBps (Pg. 523 of RM0368 Table 76)
 */
#define USART_BAUDRATE_1200		1200
#define USART_BAUDRATE_2400		2400
#define USART_BAUDRATE_9600		9600
#define USART_BAUDRATE_19200	19200
#define USART_BAUDRATE_38400	38400
#define USART_BAUDRATE_57600	57600
#define USART_BAUDRATE_115200	115200
#define USART_BAUDRATE_230400	230400
#define USART_BAUDRATE_2M		2000000
#define USART_BAUDRATE_3M		3000000


/*
 * @USART_NoOfStopBits
 */
#define USART_STOPBITS_0_5		0
#define USART_STOPBITS_1		1
#define USART_STOPBITS_1_5		2
#define USART_STOPBITS_2		3


/*
 * @USART_WordLength
 */
#define USART_WORDLEN_8BITS		0
#define USART_WORDLEN_9BITS		1


/*
 * @USART_ParityControl
 */
#define USART_PARITY_EN_ODD		2
#define USART_PARITY_EN_EVEN	1
#define USART_PARITY_DISABLE	0

/*
 * @USART_HWFLOWCONTROL
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3


/*
 * USART flags - used with status register
 */

#define USART_FLAG_TXE 			( 1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		( 1 << USART_SR_RXNE)
#define USART_FLAG_TC 			( 1 << USART_SR_TC)


/*
 * Application states
 */
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0




/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/


/*
 * USART Peripheral Clock Setup
 */
void USART_PeripheralClockControl(USART_RegDef_t *pUSARTx, uint8_t State);


/*
 * Init and Reset
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_Reset(USART_RegDef_t *pUSARTx);



/*
 * IRQ Config and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);


/*
 * Other USART Peripheral APIs
 */
void USART_EnableOrDisable(USART_RegDef_t *pUSARTx, uint8_t State);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * USART send and receive data API
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataWithIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveWithDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);




#endif /* INC_STM32F401XX_UART_DRIVER_H_ */




