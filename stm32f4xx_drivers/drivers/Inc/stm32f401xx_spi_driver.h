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
	uint8_t SPI_DeviceMode; 		/*Possible values from @SPI_DeviceModes*/
	uint8_t SPI_BusConfig;			/*Possible values from @SPI_BusConfigs*/
	uint8_t SPI_SclkSpeed;			/*Possible values from @SPI_SclkSpeed*/
	uint8_t SPI_DFF;				/*Possible values from @SPI_DFF*/
	uint8_t SPI_CPOL;				/*Possible values from @SPI_CPOL*/
	uint8_t SPI_CPHA;				/*Possible values from @SPI_CPHA*/
	uint8_t SPI_SSM;				/*Possible values from @SPI_SSM*/
}SPI_Config_t;


/*
 * Handle structure for SPI peripheras
 */
typedef struct
{
	SPI_RegDef_t *pSPIx; 			/*This handles the base address of the SPIx peripheral*/
	SPI_Config_t SPIConfig;

}SPI_Handle_t;


/*
 * @SPI_DeviceModes
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE 	0


/*
 * @SPI_BusConfigs
 * Note: We don't need SIMPLEX_TXONLY macro because it is same as duplex, we just don't connect TX wire
 * refer RM0368 Pg. 602	20.5.1 SPI_CR1 control register. Refer Bits 15,14 and 10
 */
#define SPI_BUS_CONFIG_FD				1			/*Full Duplex*/
#define SPI_BUS_CONFIG_HD				2			/*Half Duplex*/
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3			/*Simplex with RX only*/

/*
 * @SPI_SclkSpeed
 * refer RM0368 Pg. 602	20.5.1 SPI_CR1 control register
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7


/*
 * @SPI_DFF
 * refer RM0368 Pg. 602	20.5.1 SPI_CR1 control register
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

/*
 * @SPI_CPOL
 * refer RM0368 Pg. 602	20.5.1 SPI_CR1 control register
 */
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0

/*
 * @SPI_CPHA
 * refer RM0368 Pg. 602	20.5.1 SPI_CR1 control register
 */
#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0


/*
 * @SPI_SSM
 * refer RM0368 Pg. 602	20.5.1 SPI_CR1 control register
 */
#define SPI_SSM_EN			1
#define SPI_SSM_DI			0


/*
 * SPI related status definitions
 */
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE) //this sits here because it is SPI specific
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE) //this sits here because it is SPI specific



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

/*
 * Other peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);


#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
