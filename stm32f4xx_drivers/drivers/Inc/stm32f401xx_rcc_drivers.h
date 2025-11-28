/*
 * stm32f401xx_rcc_driver.h
 *
 */

#ifndef INC_STM32F401XX_RCC_DRIVERS_H_
#define INC_STM32F401XX_RCC_DRIVERS_H_


#include "stm32f401xx.h"


uint32_t  RCC_GetPLLOutputClock(void);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);













#endif /* INC_STM32F401XX_RCC_DRIVERS_H_ */
