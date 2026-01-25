/*
 * When button on the master STM32 is pressed, master should send string data to Arduino slave.
 * The data received by the Arduino board will be displayed to the Arduino serial port. In this exercise I am using esp32 so no level shifter is needed.
 * This program configures master's NSS pin in output mode to drive the slave select low
 *	07_spi_tx_arduino.c
 *
 */

#include "../drivers/Inc/stm32f401xx.h"
#include <string.h>


/* Configure SPI2, Alternate function number 5 (AF5)
 * From the datasheet Table 9. Alternate function mapping
 * PB15 - MOSI yellow
 * PB14 - MISO green
 * PB13 - SCK blue wire
 * PB12 - NSS brown
 *
 */


//TODO: add an appriate delay for switch debouncing
void delay(void){
	for(uint32_t i = 0;i < 500000/2; i++);
}



void GPIO_ButtonInit()
{
	GPIO_Handle_t GPIOBtn; //these are struct variables and not pointers so cannot use -> operator

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
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	GPIO_Init(&SPI2Pins);


}


//This function initialises the SPI config structure and also initialises the SPI peripheral
void SPI2_Init(void){
	SPI_Handle_t SPI2handle;
	//fill out the SPI configuration structure and pass it to SPI init to initialise the SPI peripheral
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD; //could go with half-duplex for this test since we are not receiving data from esp32
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //gives sclk of 2MHz ( no particular reason why)
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //Software Slave Management disabled for NSS pin, NSS pin is controlled by the hardware

	SPI_Init(&SPI2handle);
}

int main(void){

	char user_data[] = "Hello world";

	SPI2_GPIOInit(); //initialise the gpio pins for SPI2

	GPIO_ButtonInit();

	SPI2_Init(); //initialise the SPI2 peripheral handle structure

	/*
	 * Note:
	 * For master: NSS output will be enabled when SSOE = 1. When SSOE = 1, SPE = 1, NSS = 0 (NSS is pulled to low when you enable the peripheral). NSS = 1 when SPE = 0
	*/
	SPI_SSOEConfig(SPI2, ENABLE);




	while(1)
	{

		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13));

		delay();

		//enable the SPI peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//arduino sketch expects length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1); //this just sends the data len info of 1byte

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
