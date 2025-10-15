/*
 * stm32f401xx_spi_driver.c
 */

#include "stm32f401xx_spi_driver.h"




/*
 * Init and Reset
 */

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function initializes a SPI Peripheral
 *
 * @param[in]         - pointer to a SPI handle
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_Init(SPI_Handle_t *pSPIHandle){


}


/*********************************************************************
 * @fn      		  - SPI_Reset
 *
 * @brief             - This function resets a SPI Peripheral
 *
 * @param[in]         - pointer to a SPI handle
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_Reset(SPI_RegDef_t *pSPIx){

}



/*
 * Peripheral Clock Setup
 */
/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI Peripheral
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}

	}
	else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		}
	}

}


/*
 * Data Send and Receive
 */


/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function sends data over MOSI line
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - pointer to transmit buffer
 * @param[in]		  - size of data to be sent
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

}


/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function received data over MISO line
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - pointer to receive buffer
 * @param[in]		  - size of data to store
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

}

/*
 * IRQ Configuration and ISR Handling
 */


void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


