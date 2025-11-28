/*
 * stm32f401xx_rcc_driver.c
 *
 */
#include "stm32f401xx_rcc_drivers.h"

//These arrays store the possible division factors for the AHB and APB buses.
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512}; //The AHB prescaler is controlled by RCC_CFGR[7:4] (HPRE bits)
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};
uint8_t APB2_PreScaler[4] = { 2, 4 , 8, 16};

//If the system clock uses the PLL as its source, you would calculate the PLL output here based on PLLM, PLLN, PLLP values in RCC->PLLCFGR.
//right now it won't work and returns 0
uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}



/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             - This function calculates APB1 Peripheral Clock
 *
 *
 * @return            -
 *
 * @Note              -

 */

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;
	uint8_t clksrc,temp,ahbp,apb1p;

	//Reads SWS bits from RCC_CFGR[3:2] to determine the system clock source
	//00 - HSI, 01 - HSE, 10 - PLL, 11 - Reserved
	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0 )
	{	//clock src is HSI
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		//clock source is HSE
		SystemClk = 8000000;
	}else if (clksrc == 2)
	{
		//clock src is PLL
		SystemClk = RCC_GetPLLOutputClock();
	}

	//Reads HPRE bits (CFGR[7:4]) to determine AHB division
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8)
	{
		//0xxx: system clock not divided
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//Read PPRE1 bits (CFGR[12:10] to determine APB1 prescaler
	temp = ((RCC->CFGR >> 10 ) & 0x7);

	if(temp < 4)
	{
		//0xx: AHB clock not divided
		apb1p = 1;
	}else
	{

		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 =  (SystemClk / ahbp) /apb1p;

	return pclk1;
}



/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             - This function calculates APB2 Peripheral Clock
 *
 * @return            -
 *
 * @Note              -

 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2,SystemClk;
	uint8_t clksrc,temp,ahbp,apb2p;


	//Reads SWS bits from RCC_CFGR[3:2] to determine the system clock source
		//00 - HSI, 01 - HSE, 10 - PLL, 11 - Reserved
		clksrc = ((RCC->CFGR >> 2) & 0x3);

		if(clksrc == 0 )
		{	//clock src is HSI
			SystemClk = 16000000;
		}else if(clksrc == 1)
		{
			//clock source is HSE
			SystemClk = 8000000;
		}else if (clksrc == 2)
		{
			//clock src is PLL
			SystemClk = RCC_GetPLLOutputClock();
		}

		//Reads HPRE bits (CFGR[7:4]) to determine AHB division
		temp = ((RCC->CFGR >> 4 ) & 0xF);

		if(temp < 8)
		{
			//0xxx: system clock not divided
			ahbp = 1;
		}else
		{
			ahbp = AHB_PreScaler[temp-8];
		}

		//Read PPRE2 bits (CFGR[15:13] to determine APB2 prescaler
		temp = ((RCC->CFGR >> 13 ) & 0x7);

		if(temp < 4)
		{
			//0xx: AHB clock not divided
			apb2p = 1;
		}else
		{

			apb2p = APB2_PreScaler[temp-4];
		}

		pclk2 =  (SystemClk / ahbp) /apb2p;

		return pclk2;
}



