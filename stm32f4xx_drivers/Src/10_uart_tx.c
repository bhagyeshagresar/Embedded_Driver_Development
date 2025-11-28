/*
 * 10_uart_tx.c
 * A simple program to send data over UART from STM32 board to Arduino board. The Arduino board should display the message on the Arduino Serial Monitor
 * Baudrate : 115200 bps
 * Frame format: 1 stop bit, 8 data bits, no parity
 */


#include<stdio.h>
#include "stm32f401xx.h"
#include<string.h>


#define USER_BUTTON

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

//TODO: add an appriate delay for switch debouncing
void delay(void){
	for(uint32_t i = 0;i < 500000/2; i++);
}


void USART2_GPIOInit()
{

	GPIO_Handle_t usart2;

	//from the datasheet, PA2 and PA3 can be used for UART TX and UART RX respectively by setting the Alternate functionality to 7

	//configure GPIOA Pin 2 as UART2 TX, from the reference manual Pg. 512, the TX line is held high when IDLE also RX line is held high when IDLE
	//From Table 24 Port Bit Configuration table on Pg.148, the output type has to be set. It can either be Push Pull or Open Drain, open drain
	/*
	 * Note: Pull-ups or pull-downs are not needed by default because the UART peripheral can source and sink current on its own.
	 */

	//Lets configure TX pin and RX pins
	usart2.pGPIOx = GPIOA;
	usart2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN; 		//set it to alternate function mode
	usart2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;		//so its tx,the peripheral should be able to drive the pin high and low
	usart2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	usart2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	usart2.GPIO_PinConfig.GPIO_PinAltFundMode =7;

	//initialise TX pin
	usart2.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NUM_2;
	GPIO_Init(&usart2);

	//initialise RX pin
	usart2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_3;
	GPIO_Init(&usart2);


}

void USART2_Init_Config()
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_RXTX;
	usart2_handle.USART_Config.USART_Baud = USART_BAUDRATE_115200;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART_Init(&usart2_handle); //init the uart peripheral
	USART_EnableOrDisable(USART2, ENABLE); //enable the UART peripheral
}





int main(void)
{
	//Configure a GPIO pin for button input(the blue button the nucleo board, refer the schematic of the nucleo board)
	GPIO_ButtonInit();

	//Configure two pins for UART TX and RX using Altenrate functionality mode, refer the datasheet of the MCU
	USART2_GPIOInit();

	//configure the USART2 peripheral and setup all the configuration(baud rate, word length, parity etc.This function calls the USART_Init() function to initialise the peripheral
	USART2_Init_Config();

	while(1)
	{
		//we will send data when a button is pressed

		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13) == BTN_PRESSED){
			delay(); //TODO: figure out the actual delay time
			USART_SendData(&usart2_handle, (uint8_t*)msg, strlen(msg));

		}
	}


	return 0;
}
