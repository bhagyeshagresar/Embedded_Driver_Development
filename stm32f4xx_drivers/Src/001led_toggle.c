#include "stm32f401xx.h"


void delay(void){
	for(uint32_t i = 0;i < 500000/2; i++);
}


int main(){

	/*
	 * Uncomment the below code to run toggle the onboard LED using output mode set to PUSH PULL Configuration
	 */
	/*GPIO_Handle_t GpioLED;

	GpioLED.pGPIOx = GPIOA;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; //set to push pull output
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLED);

	while(1){
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
		delay();
	}*/


	/*
	 * Uncomment the below code to run toggle the onboard LED using output mode set to OPEN DRAIN Configuration
	 * Note: with GPIO_NO_PUPD the pin won't toggle because the LED will be driven to GND. with GPIO_PU, since the internal pull-up
	 * resistor value is too high(40KOhm) the current flowing throught the led is really small so the led blinks but with very low intensity
	 * in order to fix this, add a small external resistor as pull-up at PA5(5V -> resistor -> PA5) and set the PinPuPdControl to GPIO_NO_PUPD
	 */

	/*GPIO_Handle_t GpioLED;

	GpioLED.pGPIOx = GPIOA;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VHIGH;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; //set to open drain output
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLED);

	while(1){
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
		delay();
	}*/




	return 0;
}
