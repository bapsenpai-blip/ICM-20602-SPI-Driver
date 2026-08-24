#include "bmp280.h"
#include <math.h>

BMP280_CalibData_t bmp280_calib;
uint32_t t_fine;

static void BMP280_WriteReg(BMP280_HandleTypeDef *pBMP280Handle, uint8_t reg_addr, uint8_t value){
    uint16_t devaddr = (uint16_t)(pBMP280Handle->Config.DeviceAddress << 1);
    HAL_I2C_Mem_Write(pBMP280Handle->pI2CHandle, devaddr, reg_addr, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT);
}

static void BMP280_ReadReg(BMP280_HandleTypeDef *pBMP280Handle, uint8_t reg_addr, uint8_t *pBuffer, uint8_t len){
    uint16_t devaddr = (uint16_t)(pBMP280Handle->Config.DeviceAddress << 1);
    HAL_I2C_Mem_Read(pBMP280Handle->pI2CHandle, devaddr, reg_addr, I2C_MEMADD_SIZE_8BIT, pBuffer, len, I2C_TIMEOUT);
}

void BMP280_Init(BMP280_HandleTypeDef *pBMP280Handle)
{
	/* 0. Kiểm tra Chip ID (Sanity Check) */
    uint8_t chip_id = 0;
    BMP280_ReadReg(pBMP280Handle, BMP280_REG_ID, &chip_id, 1);
    
    if (chip_id != 0x58) {
        // Không tìm thấy BMP280 hoặc sai kết nối I2C
        printf("Error: BMP280 not found! (Read ID: 0x%02X, Expected: 0x58)\r\n", chip_id);
        return; 
    }
    /* 1. Soft reset */
    BMP280_WriteReg(pBMP280Handle, BMP280_REG_RESET, BMP280_SOFT_RESET_CMD);
    HAL_Delay(100);

    /* 2. Đợi NVM nạp xong Calibration data */
    uint8_t status = 1;
    while (status & 0x01) {
        BMP280_ReadReg(pBMP280Handle, BMP280_REG_STATUS, &status, 1);
        HAL_Delay(10);
    }

    /* 3. Đọc 24-byte Calibration */
    uint8_t calib_data[24];
    BMP280_ReadReg(pBMP280Handle, BMP280_REG_CALIB_START, calib_data, 24);

    bmp280_calib.dig_T1 = (uint16_t)((calib_data[1] << 8) | calib_data[0]);
    bmp280_calib.dig_T2 = (int16_t) ((calib_data[3] << 8) | calib_data[2]);
    bmp280_calib.dig_T3 = (int16_t) ((calib_data[5] << 8) | calib_data[4]);
    bmp280_calib.dig_P1 = (uint16_t)((calib_data[7] << 8) | calib_data[6]);
    bmp280_calib.dig_P2 = (int16_t) ((calib_data[9] << 8) | calib_data[8]);
    bmp280_calib.dig_P3 = (int16_t) ((calib_data[11] << 8) | calib_data[10]);
    bmp280_calib.dig_P4 = (int16_t) ((calib_data[13] << 8) | calib_data[12]);
    bmp280_calib.dig_P5 = (int16_t) ((calib_data[15] << 8) | calib_data[14]);
    bmp280_calib.dig_P6 = (int16_t) ((calib_data[17] << 8) | calib_data[16]);
    bmp280_calib.dig_P7 = (int16_t) ((calib_data[19] << 8) | calib_data[18]);
    bmp280_calib.dig_P8 = (int16_t) ((calib_data[21] << 8) | calib_data[20]);
    bmp280_calib.dig_P9 = (int16_t) ((calib_data[23] << 8) | calib_data[22]);

    /* 4. Cấu hình CONFIG (IIR Filter + Standby Time) */
    uint8_t config_reg = pBMP280Handle->Config.StandbyTime | pBMP280Handle->Config.IIR_Filter;
    BMP280_WriteReg(pBMP280Handle, BMP280_REG_CONFIG, config_reg);

    /* 5. Cấu hình CTRL_MEAS và bật Normal Mode */
    uint8_t ctrl_meas_config = pBMP280Handle->Config.Ovrs_temp | pBMP280Handle->Config.Ovrs_press | pBMP280Handle->Config.PowerMode;
    BMP280_WriteReg(pBMP280Handle, BMP280_REG_CTRL_MEAS, ctrl_meas_config);

    HAL_Delay(100);

    /* 6. Chờ IIR Filter hội tụ & lấy áp suất tham chiếu ban đầu */
    for(int i = 0; i < 15; i++) {
        BMP280_Read_Temp_Press(pBMP280Handle);
        HAL_Delay(20);
    }

    double sum_pressure = 0;
    for(int i = 0; i < 20; i++) {
        BMP280_Read_Temp_Press(pBMP280Handle);
        sum_pressure += pBMP280Handle->Pressure;
        HAL_Delay(20);
    }
    pBMP280Handle->Ground_press = (float)(sum_pressure / 20.0);
}

void BMP280_Read_Temp_Press(BMP280_HandleTypeDef *pBMP280Handle)
{
    uint8_t data[6];
    BMP280_ReadReg(pBMP280Handle, BMP280_REG_PRESS_MSB, data, 6);

    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);

    // --- TÍNH TOÁN NHIỆT ĐỘ ---
    int32_t var1_t, var2_t;
    var1_t = ((((adc_T >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) * ((int32_t)bmp280_calib.dig_T2)) >> 11;
    var2_t = (((((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1))) >> 12) * ((int32_t)bmp280_calib.dig_T3)) >> 14;
    t_fine = var1_t + var2_t;
    pBMP280Handle->Temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;

    // --- TÍNH TOÁN ÁP SUẤT ---
    int64_t var1_p, var2_p, p;
    var1_p = ((int64_t)t_fine) - 128000;
    var2_p = var1_p * var1_p * (int64_t)bmp280_calib.dig_P6;
    var2_p = var2_p + ((var1_p * (int64_t)bmp280_calib.dig_P5) << 17);
    var2_p = var2_p + (((int64_t)bmp280_calib.dig_P4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)bmp280_calib.dig_P3) >> 8) + ((var1_p * (int64_t)bmp280_calib.dig_P2) << 12);
    var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)bmp280_calib.dig_P1) >> 33;

    if (var1_p == 0) {
        pBMP280Handle->Pressure = 0;
    } else {
        p = 1048576 - adc_P;
        p = (((p << 31) - var2_p) * 3125) / var1_p;
        var1_p = (((int64_t)bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2_p = (((int64_t)bmp280_calib.dig_P8) * p) >> 19;
        p = ((p + var1_p + var2_p) >> 8) + (((int64_t)bmp280_calib.dig_P7) << 4);
        pBMP280Handle->Pressure = (float)p / 256.0f;
    }

    // --- TÍNH TOÁN ĐỘ CAO TƯƠNG ĐỐI ---
    if(pBMP280Handle->Ground_press > 0.0f && pBMP280Handle->Pressure > 0.0f) {
        pBMP280Handle->Altitude = 44330.0f * (1.0f - powf((pBMP280Handle->Pressure / pBMP280Handle->Ground_press), 0.1902949f));
    }
}