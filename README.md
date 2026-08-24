# ICM-20602 SPI Driver

Simple ICM20602 driver (with additional BMP280 I2C driver) written on top of STM32 HAL.  
Used to validate my self-designed STM32F405RGT6 breakout board.

## Purpose

This project was created to validate the hardware design of my custom STM32F405RGT6 breakout board before moving on to a full flight controller PCB.

Main goals:
- Verify SPI communication between STM32F405RGT6 and ICM20602
- Test Data Ready interrupt and non-blocking SPI read
- Confirm basic accelerometer and gyroscope functionality on real hardware
- Serve as a reference while developing a self-designed UAV flight controller

## Features

- SPI communication (Mode 3)
- Data Ready interrupt (EXTI)
- Non-blocking data read using SPI Interrupt
- Basic accelerometer & gyroscope offset calibration
- Output in physical units (g and °/s)
- Example implementation included

## Hardware

- **MCU:** STM32F405RGT6 (custom breakout board)
- **Sensors:** ICM20602 (SPI), BMP280 (I2C)
- **Interface:** SPI + INT pin (ICM20602)
