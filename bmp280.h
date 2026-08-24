#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>

#define I2C_TIMEOUT 100
/* ----------------- CONFIGURATION STRUCTURE FOR BMP280 ----------------- */
typedef struct{
	uint8_t DeviceAddress; // 0x76 (SD0 -> GND) or 0x77 (SD0 -> VCC)
	uint8_t PowerMode;     // Three modes: Sleep, Forced and Normal
	uint8_t Ovrs_press;	   // OverSampling for pressure measurement
	uint8_t Ovrs_temp;	   // OverSampling for temperature measurement
	uint8_t StandbyTime;   // Configure if Normal mode was selected
	uint8_t IIR_Filter;	   //
}BMP280_Config_t;

/* ----------------- HANDLE STRUCTURE FOR BMP280 ----------------- */
typedef struct{
	I2C_HandleTypeDef *pI2CHandle;
	BMP280_Config_t Config;

	float Temperature;
	float Pressure;
	float Altitude;
	float Ground_press;
}BMP280_HandleTypeDef;

/***************** BMP280 MACROS *****************/
/* I2C Slave Address */
#define BMP280_I2C_GND_ADDR 	0x76 	// Pin SDO connected to GND
#define BMP280_I2C_VCC_ADDR 	0x77 	// Pin SDO connected to VCC

/* BMP280 Registers */
#define BMP280_REG_ID 			0xD0	// Hold Chip ID 0x58
#define BMP280_REG_RESET 		0xE0	// Soft reset register
#define BMP280_REG_STATUS		0xF3
#define BMP280_REG_CTRL_MEAS	0xF4	// Configure OverSampling and Power Mode
#define BMP280_REG_CONFIG		0xF5	// Configure IIR filter and Standby time
#define BMP280_REG_PRESS_MSB	0xF7	// Starting Address	of Raw data (6 bytes)
#define BMP280_REG_CALIB_START	0x88	// Starting Address of Calibration Data

/* Soft reset command */
#define BMP280_SOFT_RESET_CMD	0xB6
/* Macros for register CTRL_MEAS */
// Select power mode of the device
#define BMP280_SLEEP			0x00
#define BMP280_FORCED_1			0x01
#define BMP280_FORCED_2			0x02
#define BMP280_NORMAL			0x03

// OverSampling for pressure measurement
#define BMP280_P_OVS_SKIP		(0x00 << 2)
#define BMP280_P_OVS_1			(0x01 << 2)
#define BMP280_P_OVS_2			(0x02 << 2)
#define BMP280_P_OVS_4			(0x03 << 2)
#define BMP280_P_OVS_8			(0x04 << 2)
#define BMP280_P_OVS_16			(0x05 << 2)

// OverSampling for temperature measurement
#define BMP280_T_OVS_SKIP		(0x00 << 5)
#define BMP280_T_OVS_1			(0x01 << 5)
#define BMP280_T_OVS_2			(0x02 << 5)
#define BMP280_T_OVS_4			(0x03 << 5)
#define BMP280_T_OVS_8			(0x04 << 5)
#define BMP280_T_OVS_16			(0x05 << 5)

/* Macros for register CONFIG */
// Select Standby Time (in Normal Mode)
#define BMP280_ST_HALFSEC		(0x00 << 5)
#define BMP280_ST_62HALF		(0x01 << 5)
#define BMP280_ST_125			(0x02 << 5)
#define BMP280_ST_250			(0x03 << 5)
#define BMP280_ST_500			(0x04 << 5)
#define BMP280_ST_1000			(0x05 << 5)
#define BMP280_ST_2000			(0x06 << 5)
#define BMP280_ST_4000			(0x07 << 5)

// IIR Filter setting
#define BMP280_FILTER_OFF      (0x00 << 2)
#define BMP280_FILTER_X2       (0x01 << 2)
#define BMP280_FILTER_X4       (0x02 << 2)
#define BMP280_FILTER_X8       (0x03 << 2)
#define BMP280_FILTER_X16      (0x04 << 2)
/***************** DATA CALIBRATION STRUCTURE *****************/
typedef struct{
	uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
	uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3; int16_t dig_P4;
	int16_t  dig_P5; int16_t dig_P6; int16_t dig_P7; int16_t dig_P8; int16_t dig_P9;
}BMP280_CalibData_t;
/* Declare global variable for Data calibration structure */
extern BMP280_CalibData_t bmp280_calib;

/***************** BMP280's API FUNCTIONS *****************/

void BMP280_Init(BMP280_HandleTypeDef *pBMP280Handle);
//void BMP280_ReadRegIT(uint8_t device_addr, uint8_t reg_addr, uint8_t *pBuffer, uint8_t len);
void BMP280_Read_Temp_Press(BMP280_HandleTypeDef *pBMP280Handle);
#endif
