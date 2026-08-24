#include "icm20602.h"
#include "math.h"

#define ICM_CS_LOW(h)   HAL_GPIO_WritePin((h)->CS_Port, (h)->CS_Pin, GPIO_PIN_RESET);
#define ICM_CS_HIGH(h)  HAL_GPIO_WritePin((h)->CS_Port, (h)->CS_Pin, GPIO_PIN_SET);

volatile uint16_t rx_cmplt_count = 0;

static ICM20602_HandleTypeDef *pGlobalICM = NULL;

uint8_t imu_data[14];
const float Accel_Sensitivity_Table[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
const float Gyro_Sensitivity_Table[4]  = {131.0f, 65.5f, 32.8f, 16.4f};

static void ICM20602_WriteReg(ICM20602_HandleTypeDef *hicm20602, uint8_t reg_addr, uint8_t value){
    uint8_t tx_buf[2] = {(uint8_t)(reg_addr & 0x7F), value };
    ICM_CS_LOW(hicm20602);
    HAL_SPI_Transmit(hicm20602->pSPIHandle, tx_buf, 2, HAL_MAX_DELAY);
    ICM_CS_HIGH(hicm20602);   
}

static void ICM20602_ReadReg(ICM20602_HandleTypeDef *hicm20602, uint8_t reg_addr, uint8_t *pBuffer, uint8_t len){
    uint8_t tx_buf[len + 1];
    uint8_t rx_buf[len + 1];

    memset(tx_buf, 0, len + 1);
    tx_buf[0] = (uint8_t)(reg_addr | 0x80); // Bit 7 = 1 (Read)

    ICM_CS_LOW(hicm20602);
    HAL_SPI_TransmitReceive(hicm20602->pSPIHandle, tx_buf, rx_buf, len + 1, HAL_MAX_DELAY);
    ICM_CS_HIGH(hicm20602);

    // Byte index 0 là dummy, dữ liệu thực tế bắt đầu từ index 1
    memcpy(pBuffer, &rx_buf[1], len);
}

static void ICM20602_Calib(ICM20602_HandleTypeDef *hicm20602){
    uint32_t count = 500;
    float Ax_Sum = 0, Ay_Sum = 0, Az_Sum = 0;
    float Gx_Sum = 0, Gy_Sum = 0, Gz_Sum = 0;
    float a_sen = Accel_Sensitivity_Table[hicm20602->Config.Accel_FS >> 3];
    for(uint32_t i = 0; i < count; i++){
        ICM20602_ReadReg(hicm20602, ICM20602_ACCEL_OUT, imu_data, 14);
        int16_t rAx = (int16_t)(imu_data[0] << 8 | imu_data[1]);
        int16_t rAy = (int16_t)(imu_data[2] << 8 | imu_data[3]);
        int16_t rAz = (int16_t)(imu_data[4] << 8 | imu_data[5]);
        int16_t rGx = (int16_t)(imu_data[8] << 8 | imu_data[9]);
        int16_t rGy = (int16_t)(imu_data[10] << 8 | imu_data[11]);
        int16_t rGz = (int16_t)(imu_data[12] << 8 | imu_data[13]);

        Ax_Sum += rAx; Ay_Sum += rAy; Az_Sum += rAz;
        Gx_Sum += rGx; Gy_Sum += rGy; Gz_Sum += rGz;

        HAL_Delay(10);
    }
    hicm20602->Calib.Ax_Offset = Ax_Sum / count;
    hicm20602->Calib.Ay_Offset = Ay_Sum / count;
    hicm20602->Calib.Az_Offset = (Az_Sum / count) - a_sen;

    hicm20602->Calib.Gx_Offset = Gx_Sum / count;
    hicm20602->Calib.Gy_Offset = Gy_Sum / count;
    hicm20602->Calib.Gz_Offset = Gz_Sum / count;
}

HAL_StatusTypeDef ICM20602_Init(ICM20602_HandleTypeDef *hicm20602){
    pGlobalICM = hicm20602;
	uint8_t who_am_i = 0;
    ICM20602_ReadReg(hicm20602, ICM20602_WHO_AM_I, &who_am_i, 1);
    if (who_am_i != 0x12 && who_am_i != 0x2E) {
     return HAL_ERROR; 
    }
	ICM20602_MspInit(hicm20602);
	// 0. Reset Sensor
	ICM20602_WriteReg(hicm20602, ICM20602_PWR_MGMT1, ICM20602_PWR1_DEV_RESET);
	HAL_Delay(50);
	// 1. Wake up sensor and select Auto Clock source
	ICM20602_WriteReg(hicm20602, ICM20602_PWR_MGMT1, ICM20602_PWR1_CLKSEL_AUTO);
	HAL_Delay(50);
	// 2. Configure resolution for GYRO
	ICM20602_WriteReg(hicm20602, ICM20602_GYRO_CONFIG, hicm20602->Config.Gyro_FS);
	// 3. Configure resolution for ACCEL
	ICM20602_WriteReg(hicm20602, ICM20602_ACCEL_CONFIG, hicm20602->Config.Accel_FS);
	// 4. Enable DLPF for GYRO
	ICM20602_WriteReg(hicm20602, ICM20602_CONFIG, hicm20602->Config.DLPF_Gyro);
	// 5. Enable DLPF for GYRO
	ICM20602_WriteReg(hicm20602, ICM20602_ACCEL_CONFIG2 , hicm20602->Config.DLPF_Accel);
	// 6. Set Sample Rate Divider
	ICM20602_WriteReg(hicm20602, ICM20602_SMPLRT_DIV , hicm20602->Config.Smplrt_Div);
	/*** INTERRUPT ***/
	// 1. Enable interrupt;
	ICM20602_WriteReg(hicm20602, ICM20602_INT_ENABLE , hicm20602->Config.Int_Enable);
	// 2. Configure INT pin
	ICM20602_WriteReg(hicm20602, ICM20602_INT_PIN_CFG , hicm20602->Config.Int_Pin_Cfg);
	/*** CALIBRATING DATA ***/
	ICM20602_Calib(hicm20602);
    return HAL_OK;
}

static uint8_t spi_tx_buff[15] = {ICM20602_ACCEL_OUT | 0x80};
static uint8_t spi_rx_buff[15];

void ICM20602_ReadData(ICM20602_HandleTypeDef *hicm20602){
    ICM_CS_LOW(hicm20602);
    HAL_SPI_TransmitReceive_IT(hicm20602->pSPIHandle, spi_tx_buff, spi_rx_buff, 15);
}

void ICM20602_ProcessData(ICM20602_HandleTypeDef *hicm20602){
	// Accelerator
	int16_t raw_accel_X = (int16_t)(imu_data[0] << 8 | imu_data[1]);
	int16_t raw_accel_Y = (int16_t)(imu_data[2] << 8 | imu_data[3]);
	int16_t raw_accel_Z = (int16_t)(imu_data[4] << 8 | imu_data[5]);
	// Temperature = (mpu_data[6] << 8 | mpu_data[7])
	// Gyroscope
	int16_t raw_gyro_X = (imu_data[8] << 8 | imu_data[9]);
	int16_t raw_gyro_Y = (imu_data[10] << 8 | imu_data[11]);
	int16_t raw_gyro_Z = (imu_data[12] << 8 | imu_data[13]);
	// Sensitivity depends on which Full-scale range we selected
	float g_sen = Gyro_Sensitivity_Table[hicm20602->Config.Gyro_FS >> 3];
	float a_sen = Accel_Sensitivity_Table[hicm20602->Config.Accel_FS >> 3];
    // REAL Axis Accelerators (unit: g)
    hicm20602->Ax = (raw_accel_X - hicm20602->Calib.Ax_Offset) / a_sen;
    hicm20602->Ay = (raw_accel_Y - hicm20602->Calib.Ay_Offset) / a_sen;
    hicm20602->Az = (raw_accel_Z - hicm20602->Calib.Az_Offset) / a_sen;

    // REAL Gyroscope measurements (unit: degree/s)
    hicm20602->Gx = (raw_gyro_X - hicm20602->Calib.Gx_Offset) / g_sen;
    hicm20602->Gy = (raw_gyro_Y - hicm20602->Calib.Gy_Offset) / g_sen;
    hicm20602->Gz = (raw_gyro_Z	 - hicm20602->Calib.Gz_Offset) / g_sen;
}

__weak void ICM20602_MspInit(ICM20602_HandleTypeDef *hicm20602){
    UNUSED(hicm20602);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    if (pGlobalICM != NULL && hspi == pGlobalICM->pSPIHandle) {
        ICM_CS_HIGH(pGlobalICM);
        memcpy(imu_data, &spi_rx_buff[1], 14);
        ICM20602_ProcessData(pGlobalICM);
        pGlobalICM->data_done_read = 1;
        rx_cmplt_count++;
    }
}