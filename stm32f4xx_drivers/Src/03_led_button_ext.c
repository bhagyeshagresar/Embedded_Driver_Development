/*
 * Program to use an external user btn to toggle an extern led
 */



#include "../drivers/Inc/stm32f401xx.h"

void delay(void){
	for(uint32_t i = 0;i < 500000/2; i++);
}


int main(){

	GPIO_Handle_t GpioLED, GpioBtn;

	//configure the led pin (PA6) as output,  push pull configuration
	GpioLED.pGPIOx = GPIOA;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_6;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VHIGH;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; //set to push pull output
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLED);

	//configure the button pin as input PB12
	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //for this case I am using an external pullup(370K)

	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&GpioBtn);

	while(1){
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NUM_12) == BTN_PRESSED){
			delay(); //add a delay to account for the switch debouncing
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NUM_6);

		}
	}

	return 0;
}

