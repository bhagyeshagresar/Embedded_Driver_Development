/*
 * stm32f401xx_spi_driver.h
 */

#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_

#include "stm32f401xx.h"

/*
 * Configuration structure for SPI peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


/*
 * Handle structure for SPI peripheras
 */
typedef struct
{
	SPI_RegDef_t *pSPIx; 			/*This handles the base address of the SPIx peripheral*/
	SPI_Config_t SPIConfig;

}SPI_Handle_t;



/****************************************
 * APIs supported by this driver
 *
 ****************************************/
/*
 * Init and Reset
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_Reset(SPI_RegDef_t *pSPIx);

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
