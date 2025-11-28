/*
 * Connect an external button to PD5 pin and toggle the led whenever interrupt is triggered by the button press. Interrupt should be triggered during falling edge of button press
 */
#include <string.h>
#include "../drivers/Inc/stm32f401xx.h"

void delay(void){
	for(uint32_t i = 0;i < 500000/2; i++);
}


int main(){

	GPIO_Handle_t GpioLED, GpioBtn;
	memset(&GpioLED, 0, sizeof(GpioLED)); //Note: This is done because for the button as input, there is no output type and the compiler stores garbage value sfor the GPIO_Handle_t local variables, so the OTYPER register values get corrupted
	memset(&GpioBtn, 0, sizeof(GpioBtn));


	//configure the led pin (PA6) as output,  push pull configuration
	GpioLED.pGPIOx = GPIOA;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_6;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VHIGH;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; //set to push pull output
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLED);

	//configure the button pin as input PD2
	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_2;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT; //trigger an interrupt on falling edge
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //for this case I am using an external pullup(370K)

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioBtn);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI2, NVIC_IRQ_PRI15); 	//For PD2, the interrupt line is EXTI2 and the priority is arbitrarily set to 15
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI2, ENABLE);

	while(1);

	return 0;
}


//name comes from the startup file
void EXTI2_IRQHandler(void){
	delay(); //add a delay to avoid debouncing/interrupt getting triggered multiple times
	GPIO_IRQHandling(GPIO_PIN_NUM_2); //clear the pending event from the EXTI line
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_6);


}
