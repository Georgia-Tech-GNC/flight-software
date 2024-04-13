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

#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h723xx.h"

enum spiSpeed { SPI_SLOW, SPI_MEDIUM, SPI_FAST };


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

typedef struct ADIS16500_Data {  
    int16_t x_gyro_out;
    int16_t y_gyro_out;
    int16_t z_gyro_out;
    int16_t x_accl_out;
    int16_t y_accl_out;
    int16_t z_accl_out;
    int16_t temp_out;
    int16_t time_stamp;
    int16_t data_cntr;
    int16_t x_deltang_out;
    int16_t y_deltang_out;
    int16_t z_deltang_out;
    int16_t x_deltvel_out;
    int16_t y_deltvel_out;
    int16_t z_deltvel_out;
} ADIS16500_Data;

typedef struct {
    GPIO_TypeDef *spi_sck;
    GPIO_TypeDef *spi_miso;
    GPIO_TypeDef *spi_mosi;
    GPIO_TypeDef *spi_cs;
    SPI_TypeDef *spi;
    GPIO_TypeDef *nrst;
    GPIO_TypeDef *dr;
} ADIS16500_Config;

extern const ADIS16500_Config adis_imu;

void adis_init(const ADIS16500_Config *conf);
uint16_t adis_get(ADIS_RegAddr *addr);
void adis_set(ADIS_RegAddr addr, uint16_t value);
void adis_get_data(ADIS16500_Data *data);
void adis_reset(void);
uint16_t adis_self_test(void);

#endif