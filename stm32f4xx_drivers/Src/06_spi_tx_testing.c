/*
 * Exercise to configure the MCU's SPI2 as master mode and just send a "hello world". This program won't use the slave so only two pins are required, MOSI and SCLK
 * 06spi_tx_testing.c
 *
 *
 */

#include "../drivers/Inc/stm32f401xx.h"
#include <string.h>


/* Configure SPI2, Alternate function number 5 (AF5)
 * From the datasheet Table 9. Alternate function mapping
 * PB15 - MOSI
 * PB14 - MISO
 * PB13 - SCK
 * PB12 - NSS
 *
 */


//TODO: add an appriate delay for switch debouncing
void delay(void){
	for(uint32_t i = 0;i < 500000/2; i++);
}



void GPIO_ButtonInit()
{
	GPIO_Handle_t GPIOBtn, GPIOLed; //these are struct variables and not pointers so cannot use -> operator

	//configure the gpio pin for the button PC13
	GPIOBtn.pGPIOx = GPIOC;  									//this is a pointer to GPIOC baseaddress
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13; 	//Configure the PIN number 13
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;			//Configure the pin as input
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;		//Set high speed
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;	// the nucleo board already has a pull-up resistor connected to this pin, the configuration is ACTIVE LOW

	GPIO_Init(&GPIOBtn);

}


//This function is used to initialise the GPIO pins to behave as SPI2 pins
void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPI2Pins;

	SPI2Pins.pGPIOx = GPIOB;
	SPI2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2Pins.GPIO_PinConfig.GPIO_PinAltFundMode = 5;
	SPI2Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // based on SPI specification
	SPI2Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI2Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCLK
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GPIO_Init(&SPI2Pins);

	//MOSI
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_15;
	GPIO_Init(&SPI2Pins);

	//MISO
	/*SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_14;
	GPIO_Init(&SPI2Pins);*/

	//NSS
	/*SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	GPIO_Init(&SPI2Pins);*/


}


//This function initialises the SPI config structure and also initialises the SPI peripheral
void SPI2_Init(void){
	SPI_Handle_t SPI2handle;
	//fill out the SPI configuration structure and pass it to SPI init to initialise the SPI peripheral
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //gives sclk of 8MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //Software Slave Management enabled for NSS pin. NSS pin is ignored.

	SPI_Init(&SPI2handle);
}

int main(void){

	char user_data[] = "Hello world";

	SPI2_GPIOInit(); //initialise the gpio pins for SPI2

	GPIO_ButtonInit();

	SPI2_Init(); //initialise the SPI2 peripheral handle structure

	/*
	 * Note:
	 * SSI defines the internal NSS level.NSS LOW indicates “I am selected as a slave” and this can't work for Master Mode.
	 * so we need to set SSM bit and also SSI bit so the NSS pin is pulled to high internally and the MCU acts in master mode
	*/
	SPI_SSIConfig(SPI2, ENABLE);

	//enable the SPI peripheral
	//SPI_PeripheralControl(SPI2, ENABLE);



	while(1)
	{
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13));

		delay();

		//enable the SPI peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//SPI send data blocking call
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		/*
		 * Confirm if SPI is busy.
		 * Note: This is important otherwise the peripheral will get disabled immediately after sending data
		 */
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//disable the SPI peripheral so we can see MOSI and CLK lines switch to idler on the analyser
		SPI_PeripheralControl(SPI2, DISABLE);
	}


	return 0;
}
