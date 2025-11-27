/*
 * Exercise to configure the MCU's SPI2 as master mode and just send a "hello world". This program won't use the slave so only two pins are required, MOSI and SCLK
 * 06spi_tx_testing.c
 *
 *
 */

#include "stm32f401xx.h"
#include <string.h>


/* Configure SPI2, Alternate function number 5 (AF5)
 * From the datasheet Table 9. Alternate function mapping
 * PB15 - MOSI
 * PB14 - MISO
 * PB13 - SCK
 * PB12 - NSS
 *
 */


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
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //Software Slave Management enabled for NSS pin

	SPI_Init(&SPI2handle);
}

int main(void){

	char user_data[] = "Hello world";

	SPI2_GPIOInit(); //initialise the gpio pins for SPI2

	SPI2_Init(); //initialise the SPI2 peripheral handle structure

	//enable the SPI peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//SPI send data blocking call
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	while(1);


	return 0;
}
