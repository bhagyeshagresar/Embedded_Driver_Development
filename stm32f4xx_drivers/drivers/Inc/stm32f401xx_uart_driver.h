/*
 * stm32f401xx_uart_driver.h
 *
 */

#ifndef INC_STM32F401XX_UART_DRIVER_H_
#define INC_STM32F401XX_UART_DRIVER_H_

#include "stm32f401xx.h"

#endif /* INC_STM32F401XX_UART_DRIVER_H_ */


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
	uint8_t USART_HWFlowControl

}USART_Config_t;


/*
 * Handle structure for UART peripheral
 */
typedef struct
{
	UART_RegDef_t* pUSARTx;
	UART_Config_t USARTConfig;
}USART_handle_t;
