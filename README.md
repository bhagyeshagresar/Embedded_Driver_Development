# Bare Metal Embedded Driver Development using STM32

This repository contains a collection of bare-metal driver implementations(I2C, SPI, UART, GPIO, etc.) for STM32F401RE MCU running on a NucleoBoard, along with several exercises demonstrating use of these drivers. This project is inspired by [text](https://fastbitlab.com/). Here, the concepts are re-implemented and adapted for the STM32F401RE platform. The motivation behind doing this project is to get a deep understanding of how a microcontroller works and how drivers are implemented.

# Repository Structure

**Note:** The file `startup_stm32f401retx.s` and some peripheral definitions are provided by STMicroelectronics as part of the STM32F4 CMSIS/Startup code. Copyright belongs to ST.  
This repository includes personal modifications and examples for learning purposes.
 

```
├── 005HSI_Measurement/          # Internal oscillator measurement example using a logic analyzer
├── 006HSE_Measurement/          # External oscillator measurement example using a logic analyzer
├── stm32f4xx_drivers/           # Core driver library (I2C, SPI, UART, GPIO, and MCU-specific header files)
│   ├── Src/                     # Challenges to test driver implementations
│   ├── Startup/
│   │   └── startup_stm32f401retx.s   # STM32 startup assembly file provided by ST
│   └── Drivers/
│       ├── Inc/
│       │   ├── stm32f401xx.h
│       │   ├── stm32f4xx_gpio_driver.h
│       │   ├── stm32f4xx_i2c_driver.h
│       │   ├── stm32f4xx_spi_driver.h
│       │   ├── stm32f4xx_usart_driver.h
│       │   └── stm32f4xx_rcc_driver.h
│       └── Src/
│           ├── stm32f4xx_gpio.c
│           ├── stm32f4xx_i2c.c
│           ├── stm32f4xx_spi.c
│           ├── stm32f4xx_usart.c
│           └── stm32f4xx_rcc.c
└── README.md
```

# Implementing the I2C Application code to test master send/receive to a MPU6050
![Description of the image](./images/logic_analyser_trace_reading_accel_z_reading.jpg.png)

Successfully established I2C communication with the MPU6050 sensor. Communication was first verified by reading the WHO_AM_I register. After confirming this, the sensor was taken out of sleep mode by writing 0x00 to the PWR_MGMT_1 register, and raw accelerometer data along the Z-axis was read. This code implementation does not use repeated start. It will be important to implement that feature when there are multiple masters to avoid them from accidentally pull the SCL line low to control the I2C bus.

At this stage, the application focuses on validating correct I2C register access (read/write). While no higher-level processing is implemented yet, the transmit and receive APIs are functioning correctly and provide a reliable base for future development. 


# Challenges to test driver implementations located at /stm32f4xx_drivers/Src/

01_led_toggle.c - Simple program that shows how to toggle an onboard led (Nucleo-F401RE) using push-pull vs open-drain configuration using GPIO drivers.

02_led_button.c - Program to use the onboard user btn to toggle the onboard led using GPIO drivers.

03_led_button_ext.c - Program to use an external user btn to toggle an extern led using GPIO drivers.

04_button_interrupt.c - Connect an external button to PD5 pin and toggle the led whenever interrupt is triggered by the button press. Interrupt should be triggered during falling edge of button press.

06_spi_tx_testing.c - Exercise to configure the MCU's SPI2 as master mode and just send a "hello world". This program won't use the slave so only two pins are required, MOSI and SCLK.

10_uart_tx.c -  A simple program to send data over UART from STM32 board to Arduino board. The Arduino board should display the message on the Arduino Serial Monitor, Baudrate : 115200 bps, Frame format: 1 stop bit, 8 data bits, no parity

11_i2c_master_rx_testing.c - Simple program to read raw accel data from MPU6050 using I2C