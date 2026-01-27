/*
 * 11_i2c_master_rx_testing.c
 *
 * Goal: Setup working I2C serial interface between STM32 and MPU6050 using the custom drivers written in drivers folder for the I2C communication
 *
 */


#include "../drivers/Inc/stm32f401xx.h"
#include <string.h>
#include <stdio.h>

/* Configure I2C1, Alternate function number 4 (AF4)
 * From the datasheet Table 9. Alternate function mapping
 * PB6 -> SCL
 * PB7 -> SDA
 *
 */

//Define slave and master addresses
#define MASTER_ADDR			0x61
#define SLAVE_ADDR			0x68
#define WHO_AM_I_ADDR		0x75 //stores 0x68 as its value
#define ACCELZ_HIGH_ADDR	0x3F
#define ACCELZ_LOW_ADDR		0x40
#define PWR_MGMT_1_ADDR		0x6B

//TODO: add an appriate delay for switch debouncing
void delay(void){
	for(uint32_t i = 0;i < 500000/2; i++);
}

//create an I2C handle variable
I2C_Handle_t I2C1_Handle;

//Create a rcv buffer
uint8_t rcv_buffer[32];

//GPIO Button Init function
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

//This function is used to initialise the GPIO pins to behave as I2C1 pins
void I2C1_GPIOInit(void)
{
	GPIO_Handle_t I2C1Pins;

	I2C1Pins.pGPIOx = GPIOB;
	I2C1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C1Pins.GPIO_PinConfig.GPIO_PinAltFundMode = 4;
	I2C1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; // based on SPI specification
	I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2C1Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//scl
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_6;
	GPIO_Init(&I2C1Pins);

	//sda
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_7;
	GPIO_Init(&I2C1Pins);

}


//This function initialises the I2C config structure and also initialises the I2C peripheral
void I2C1_Init(void){
	I2C1_Handle.pI2Cx = I2C1; //store the I2C1 base address
	I2C1_Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1_Handle.I2C_Config.I2C_DeviceAddress = MASTER_ADDR;
	I2C1_Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1_Handle);

}




int main(void){

	uint8_t commandCode;

	I2C1_GPIOInit(); //initialise the gpio pins for I2C1

	GPIO_ButtonInit();

	I2C1_Init(); //initialise the I2C1 peripheral handle structure

	//Enable I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ack bit is made 1 after PE=1
	//I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	memset(rcv_buffer, 0, sizeof(rcv_buffer));

	while(1)
	{
		//wait here till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//step 1: Read the who_am_I register
		commandCode = WHO_AM_I_ADDR;
		//send what register to read from - who_am_I
		//since there is only one master and slave, repeated start enabling is not important, as no other master can access control of the bus
		I2C_MasterSendData(&I2C1_Handle,&commandCode,1,SLAVE_ADDR, I2C_DISABLE_SR);

		//read 1 byte from the register on the MPU6050, should return 0x68
		I2C_MasterReceiveData(&I2C1_Handle, rcv_buffer, 1, SLAVE_ADDR, I2C_DISABLE_SR);
		memset(rcv_buffer, 0, sizeof(rcv_buffer));

		//step2: write the PWR_MGMT Register to 0x00 to wake up the sensor
		commandCode = PWR_MGMT_1_ADDR;
		uint8_t data[2];
		data[0] = PWR_MGMT_1_ADDR;
		data[1] = 0x00;   // wake up value


		I2C_MasterSendData(&I2C1_Handle,data,2,SLAVE_ADDR, I2C_DISABLE_SR);

		commandCode = ACCELZ_HIGH_ADDR;

		//step 2: see if you can directly read accel readings, raw is fine for now

		//send what register to read from - accelerometer along the Z axis
		I2C_MasterSendData(&I2C1_Handle,&commandCode,1,SLAVE_ADDR, I2C_DISABLE_SR);

		//read 2 byte from the register on the MPU6050, should return 0x68
		I2C_MasterReceiveData(&I2C1_Handle, rcv_buffer, 2, SLAVE_ADDR, I2C_DISABLE_SR);




	}


	return 0;
}


