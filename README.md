# Bare Metal Embedded Driver Development using STM32

This repository contains a collection of bare-metal driver implementations(I2C, SPI, UART, GPIO, etc.) for STM32F401RE MCU running on a NucleoBoard, along with several exercises demonstrating use of these drivers. This project is inspired by [text](https://fastbitlab.com/). Here, the concepts are re-implemented and adapted for the STM32F401RE platform. The motivation behind doing this project is to get a deep understanding of how a microcontroller works and how drivers are implemented.

# Repository Structure

**Note:** The file `startup_stm32f401retx.s` and some peripheral definitions are provided by STMicroelectronics as part of the STM32F4 CMSIS/Startup code. Copyright belongs to ST.  
This repository includes personal modifications and examples for learning purposes.
 
├── 005HSI_Measurement/       — internal oscillator measurement example using a logic analyser 
├── 006HSE_Measurement/       — external oscillator measurement example using a logic analyser 
├── stm32f4xx_drivers/        — core driver library (I2C, SPI, UART, GPIO and MCU specific header file)  
├── stm32f4xx_drivers/               # Core bare-metal driver folder
│   ├── Src/                         # Challenges to test driver implementations
│   ├── Startup/                     
│       ├── startup_stm32f401retx.s  # STM32 startup assembly file written by ST
|   ├── Drivers/
|
|
└── README.md