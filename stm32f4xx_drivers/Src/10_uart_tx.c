/*
 * 010_uart_tx.c
 * A simple program to send data over UART from STM32 board to Arduino board. The Arduino board should display the message on the Arduino Serial Monitor
 * Baudrate : 115200 bps
 * Frame format: 1 stop bit, 8 data bits, no parity
 */


#include<stdio.h>
#include <stm32f401xx.h>
#include<string.h>


char msg[1024] = "UART Tx testing...\n\r";
USART_Handle_t usart2_handle;




void GPIO_ButtonInit()
{
	GPIO_Handle_t GPIOBtn, GPIOLed; //these are struct variables and not pointers so cannot use -> operator

	//configure the gpio pin for the button
	GPIOBtn.pGPIOx = GPIOC;  									//this is a pointer to GPIOC baseaddress
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13; 	//Configure the PIN number 13
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;			//Configure the pin as input
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;		//Set high speed
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;	// the nucleo board already has a pull-up resistor connected to this pin, the configuration is ACTIVE LOW

	GPIO_Init(&GPIOBtn);

	//configure the led pin (PA5) as output,  push pull configuration
	//set to Push Pull because in open drain output can be either LOW or Hi-Z state
	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VHIGH;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; 	//set to push pull output, pull to Vcc when output =1 and pull to GND when output = 0
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOLed);

}


void USART2_GPIOInit()
{

	GPIO_Handle_t usart2;

	//from the datasheet, PA2 and PA3 can be used for UART TX and UART RX respectively by setting the Alternate functionality to 7
	//We are setting the same configuration for both the pin
	//Need to understand why we are setting the same configuration for tx and rx?
	usart2.pGPIOx = GPIOA;
	usart2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN; 		//set it to alternate function mode
	usart2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart2.GPIO_PinConfig.GPIO_PinAltFunMode =7;

	//USART2 TX
	usart2.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_2;
	GPIO_Init(&usart2);

	//USART2 RX
	usart2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart2);


}





int main(void)
{
	//Configure a GPIO pin for button input(the blue button the nucleo board, refer the schematic of the nucleo board)
	GPIO_ButtonInit();

	//Configure two pins for UART TX and RX using Altenrate functionality mode, refer the datasheet of the MCU
	USART2_GPIOInit();


	USART2_Init();

	return 0;
}
