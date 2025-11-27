/*
 * stm32f401xx_spi_driver.c
 */

#include <stm32f401xx_spi_driver.h>




uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){

	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}



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

	//let's enable the clock here since it needs to be done everytime we initialise a peripheral
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//lets configure the SPI_CR1 register

	uint32_t tempReg = 0;

	//1. Configure the device mode
	tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//clear the  bidi mode bit
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//set the bidi mode bit
		tempReg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){

		//clear the bidi mode bit
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);

		//set the rxonly bit
		tempReg |= (1 << SPI_CR1_RXONLY);

	}

	//3. Configure the SPI SCLK speed
	tempReg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//4. Configure the DFF
	tempReg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//5. Configure the CPOL
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6. Configure the CPHA
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//7. Configure SSM
	tempReg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = tempReg;


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

	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4){
		SPI4_REG_RESET();
	}

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
 * @brief             - This function sends data over MOSI line (blocking)
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - pointer to transmit buffer
 * @param[in]		  - size of data to be sent
 *
 * @return            -  none
 *
 * @Note              -  This is a blocking call

 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	//this is a blocking call, until all the bytes are transferred, the code will be stuck in this while loop
	while(Len > 0){

		//1. wait until Tx buffer is empty
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET); /*Note: Here we are polling SPI_TXE flag*/

		//2. Check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF))){
			//16bit dff
			//1. load data in the DR register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -= 2;
			(uint16_t*)pTxBuffer++;
		}
		else{
			//8bit dff
			//1. load data in the DR register
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}


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


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}


}
