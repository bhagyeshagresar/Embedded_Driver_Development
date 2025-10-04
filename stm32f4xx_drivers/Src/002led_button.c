/*
 * Program to use the onboard user btn to toggle the onboard led
 */



#include "stm32f401xx.h"

#define HIGH 1
#define LOW	 0


void delay(void){
	for(uint32_t i = 0;i < 500000/2; i++);
}


int main(){

	GPIO_Handle_t GpioLED, GpioBtn;

	//configure the led pin (PA5) as output,  push pull configuration
	GpioLED.pGPIOx = GPIOA;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VHIGH;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; //set to push pull output
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLED);

	//configure the button pin as input PC13
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //in case of nucleo, there is a pull-up available

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioBtn);

	while(1){
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13) == LOW){
			delay(); //add a delay to account for the switch debouncing
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NUM_5);

		}
	}




	return 0;
}
