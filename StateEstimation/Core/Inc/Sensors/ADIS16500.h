/**
 ******************************************************************************
 * @file    ADIS16500.h
 * @author  Kanav Chugh
 * @brief   ADIS16500 header driver file
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ADIS16500_H
#define _ADIS16500_H

#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include "DWT.h"

typedef enum {
    ADIS_DIAG_STAT = 0x02,
    ADIS_X_GYRO_LOW = 0x04,
    ADIS_X_GYRO_OUT = 0x06,
    ADIS_Y_GYRO_LOW = 0x08,
    ADIS_Y_GYRO_OUT = 0x0A,
    ADIS_Z_GYRO_LOW = 0x0C,
    ADIS_Z_GYRO_OUT = 0x0E,
    ADIS_X_ACCL_LOW = 0x10,
    ADIS_X_ACCL_OUT = 0x12,
    ADIS_Y_ACCL_LOW = 0x14,
    ADIS_Y_ACCL_OUT = 0x16,
    ADIS_Z_ACCL_LOW = 0x18,
    ADIS_Z_ACCL_OUT = 0x1A,
    ADIS_TEMP_OUT = 0x1C,
    ADIS_TIME_STAMP = 0x1E,
    ADIS_DATA_CNTR = 0x22,
    ADIS_X_DELTANG_LOW = 0x24,
    ADIS_X_DELTANG_OUT = 0x26,
    ADIS_Y_DELTANG_LOW = 0x28,
    ADIS_Y_DELTANG_OUT = 0x2A,
    ADIS_Z_DELTANG_LOW = 0x2C,
    ADIS_Z_DELTANG_OUT = 0x2E,
    ADIS_X_DELTVEL_LOW = 0x30,
    ADIS_X_DELTVEL_OUT = 0x32,
    ADIS_Y_DELTVEL_LOW = 0x34,
    ADIS_Y_DELTVEL_OUT = 0x36,
    ADIS_Z_DELTVEL_LOW = 0x38,
    ADIS_Z_DELTVEL_OUT = 0x3A,
    ADIS_XG_BIAS_LOW = 0x40,
    ADIS_XG_BIAS_HIGH = 0x42,
    ADIS_YG_BIAS_LOW = 0x44,
    ADIS_YG_BIAS_HIGH = 0x46,
    ADIS_ZG_BIAS_LOW = 0x48,
    ADIS_ZG_BIAS_HIGH = 0x4A,
    ADIS_XA_BIAS_LOW = 0x4C,
    ADIS_XA_BIAS_HIGH = 0x4E,
    ADIS_YA_BIAS_LOW = 0x50,
    ADIS_YA_BIAS_HIGH = 0x52,
    ADIS_ZA_BIAS_LOW = 0x54,
    ADIS_ZA_BIAS_HIGH = 0x56,
    ADIS_FILT_CTRL = 0x5C,
    ADIS_RANG_MDL = 0x5E,
    ADIS_MSC_CTRL = 0x60,
    ADIS_UP_SCALE = 0x62,
    ADIS_DEC_RATE = 0x64,
    ADIS_GLOB_CMD = 0x68,
    ADIS_FIRM_REV = 0x6C,
    ADIS_FIRM_DM = 0x6E,
    ADIS_FIRM_Y = 0x70,
    ADIS_PROD_ID = 0x72,
    ADIS_SERIAL_NUM = 0x74,
    ADIS_USER_SCR_1 = 0x76,
    ADIS_USER_SCR_2 = 0x78,
    ADIS_USER_SCR_3 = 0x7A,
    ADIS_FLSHCNT_LOW = 0x7C,
    ADIS_FLSHCNT_HIGH = 0x7E
} ADIS_RegAddr;

typedef struct {
    uint16_t diag_stat;
    uint16_t data_cntr;
    uint16_t checksum;
    double x_gyro_out;
    double y_gyro_out;
    double z_gyro_out;
    double x_accl_out;
    double y_accl_out;
    double z_accl_out;
    double temp_out;
} ADIS16500_Data;

struct ADIS_BurstData {
    uint16_t diag_stat;      // Diagnostic status
    float32_t gyro[3];       // Gyroscope data (X, Y, Z)
    float32_t accel[3];      // Accelerometer data (X, Y, Z)
    float32_t temp;          // Temperature
    uint16_t timestamp;      // Time stamp
};

struct ADIS_Device { 
    SPI_HandleTypeDef* spi_handle;
    GPIO_TypeDef *cs_pin;
    uint16_t cs_pin_port;
};

void DWT_Init(void);
void delay_us(uint32_t microseconds);
int16_t adis_read_register(struct ADIS_Device *device, uint8_t addr);
void adis_write_register(struct ADIS_Device *device, uint8_t addr, uint16_t value);
void adis_read_gyro(struct ADIS_Device *device, float32_t gyro_readings[3]);
void adis_read_accel(struct ADIS_Device *device, float32_t accel_readings[3]);
float32_t adis_accel_scale(int16_t raw_data);
float32_t adis_gyro_scale(int16_t raw_data);
float32_t adis_temp_scale(int16_t raw_data);
uint8_t adis_burst_read(struct ADIS_Device *device, uint16_t *burst_data);
void adis_parse_burst(uint16_t *raw_data, struct ADIS_BurstData *parsed_data);
void adis_hardware_reset(struct ADIS_Device *device, GPIO_TypeDef* reset_pin, uint16_t reset_port, uint32_t delay_ms);
int32_t adis_read_gyro_32bit(struct ADIS_Device *device, uint8_t low_reg, uint8_t high_reg);
int32_t adis_read_accel_32bit(struct ADIS_Device *device, uint8_t low_reg, uint8_t high_reg);
void adis_read_gyro_32bit_all(struct ADIS_Device *device, float32_t gyro_readings[3]);
void adis_read_accel_32bit_all(struct ADIS_Device *device, float32_t accel_readings[3]);
void adis_read_delta_angle(struct ADIS_Device *device, float32_t delta_angle[3]);
void adis_read_delta_vel(struct ADIS_Device *device, float32_t delta_vel[3]);
int32_t adis_read_delta_angle_32bit(struct ADIS_Device *device, uint8_t low_reg, uint8_t high_reg);
int32_t adis_read_delta_vel_32bit(struct ADIS_Device *device, uint8_t low_reg, uint8_t high_reg);
void adis_read_delta_angle_32bit_all(struct ADIS_Device *device, float32_t delta_angle[3]);
void adis_read_delta_vel_32bit_all(struct ADIS_Device *device, float32_t delta_vel[3]);

#endif /* _ADIS16500_H */