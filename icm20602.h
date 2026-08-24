#ifndef ICM20602_H_
#define ICM20602_H_
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>


typedef struct{
	uint8_t Gyro_FS;	   // Full-scale range for Gyroscope
	uint8_t Accel_FS;	   // Full-scale range for Accelerator
	uint8_t DLPF_Gyro;	   // Digital low pass filter for Gyroscope
	uint8_t DLPF_Accel;	   // Digital low pass filter for Accelerator
	uint8_t Smplrt_Div;    // Sample rate divider
	uint8_t Int_Pin_Cfg;	   // Configure bits INT_ANYRD_2CLEAR (4) and LATCH_INT_EN (5) of register 55
	uint8_t Int_Enable;	   // Configure register 56
}ICM20602_Config;

typedef struct {
    float Ax_Offset;
    float Ay_Offset;
    float Az_Offset;
    float Gx_Offset;
    float Gy_Offset;
    float Gz_Offset;
} ICM20602_CalibTypeDef;

typedef struct{
	SPI_HandleTypeDef *pSPIHandle;
    GPIO_TypeDef *CS_Port;
    uint16_t            CS_Pin;

	ICM20602_Config Config;
    ICM20602_CalibTypeDef Calib;

    volatile uint8_t data_done_read;

	float Ax, Ay, Az;
	float Gx, Gy, Gz;
}ICM20602_HandleTypeDef;

/* Addresses of registers*/
#define ICM20602_SMPLRT_DIV             0x19
#define ICM20602_CONFIG                 0x1A
#define ICM20602_GYRO_CONFIG            0x1B
#define ICM20602_ACCEL_CONFIG           0x1C
#define ICM20602_ACCEL_CONFIG2          0x1D
#define ICM20602_INT_PIN_CFG            0x37
#define ICM20602_INT_ENABLE             0x38
#define ICM20602_ACCEL_OUT              0x3B
#define ICM20602_TEMP_OUT               0x41
#define ICM20602_GYRO_OUT               0x43
#define ICM20602_PWR_MGMT1              0x6B
#define ICM20602_PWR_MGMT2              0x6C
#define ICM20602_WHO_AM_I               0x75

/* PWR_MGMT1*/
#define ICM20602_PWR1_DEV_RESET         (0x01 << 7)
#define ICM20602_PWR1_SLEEP             (0x01 << 6)
#define ICM20602_PWR1_CYCLE             (0x01 << 5)
#define ICM20602_PWR1_GYRO_STB          (0x01 << 4)
#define ICM20602_TEMP_DIS               (0x01 << 3)   
#define ICM20602_PWR1_CLKSEL_INT        0x00
#define ICM20602_PWR1_CLKSEL_AUTO       0x01
#define ICM20602_PWR1_CLKSEL_STOP       0x07

/* Sample Rate Divider = 1000 / (1 + Value) */
#define SMPLRT_DIV_1000Hz               0x00
#define SMPLRT_DIV_500Hz                0x01
#define SMPLRT_DIV_100HZ                0x09

/* CONFIG register */
#define ICM20602_GYRO_DLPF_250Hz        0x00
#define ICM20602_GYRO_DLPF_176Hz        0x01
#define ICM20602_GYRO_DLPF_92Hz         0x02
#define ICM20602_GYRO_DLPF_41Hz         0x03
#define ICM20602_GYRO_DLPF_20Hz         0x04
#define ICM20602_GYRO_DLPF_10Hz         0x05
#define ICM20602_GYRO_DLPF_5Hz          0x06
#define ICM20602_GYRO_DLPF_3281Hz       0x07

/* GYRO_CONFIG register */
#define ICM20602_GYRO_DLPF_EN           0x00
#define ICM20602_GYRO_DLPF_DI_32KHz     0x01
#define ICM20602_GYRO_FS_SEL_250        (0x00 << 3)
#define ICM20602_GYRO_FS_SEL_500        (0x01 << 3)
#define ICM20602_GYRO_FS_SEL_1000       (0x02 << 3)
#define ICM20602_GYRO_FS_SEL_2000       (0x03 << 3)

/* ACCEL_CONFIG register */
#define ICM20602_ACCEL_FS_SEL_2G        (0x00 << 3)
#define ICM20602_ACCEL_FS_SEL_4G        (0x01 << 3)
#define ICM20602_ACCEL_FS_SEL_8G        (0x02 << 3)
#define ICM20602_ACCEL_FS_SEL_16G       (0x03 << 3)

/* ACCEL_CONFIG2 register*/
#define ICM20602_ACCEL_DLPF_EN          (0x00 << 3)
#define ICM20602_ACCEL_DLPF_DI_32KHz    (0x01 << 3)
#define ICM20602_ACCEL_DLPF_218Hz       0x01 | 0x00
#define ICM20602_ACCEL_DLPF_99Hz        0x02
#define ICM20602_ACCEL_DLPF_45Hz        0x03
#define ICM20602_ACCEL_DLPF_21Hz        0x04
#define ICM20602_ACCEL_DLPF_10Hz        0x05
#define ICM20602_ACCEL_DLPF_5Hz         0x06
#define ICM20602_ACCEL_DLPF_420Hz       0x07

/* INT pin */
#define ICM20602_INT_ACTIVE_LOW         (0x01 << 7)
#define ICM20602_INT_ACTIVE_HIGH        (0x00 << 7)
#define ICM20602_INT_OPEN_DRAIN         (0x01 << 6)
#define ICM20602_INT_PUSHPULL           (0x00 << 6)
#define ICM20602_INT_LATCH_TIL_CLEARED  (0x01 << 5)
#define ICM20602_INT_PULSE_50US         (0x00 << 5)
#define ICM20602_INT_CLEAR_ON_ANY_R     (0x01 << 4)
#define ICM20602_INT_CLEAR_ON_STATUS    (0x00 << 4)
#define ICM20602_FSYNC_INT_ACTIVE_HIGH  (0x00 << 3)
#define ICM20602_FSYNC_INT_ACTIVE_LOW   (0x01 << 3)
#define ICM20602_FSYNC_INT_DIS          (0x00 << 2)
#define ICM20602_FSYNC_INT_EN           (0x01 << 2)

#define INT_RAW_RDY_EN					(0x01 << 0)

HAL_StatusTypeDef ICM20602_Init(ICM20602_HandleTypeDef *hicm20602);
void ICM20602_ReadData(ICM20602_HandleTypeDef *hicm20602);
void ICM20602_ProcessData(ICM20602_HandleTypeDef *hicm20602);
void ICM20602_MspInit(ICM20602_HandleTypeDef *hicm20602);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

#endif /* ICM20602_H_ */