/**
  ******************************************************************************
  * @file    LIS3MDL.h
  * @author  Kanav Chugh
  * @brief   LIS3MDL magnetometer code
  *
  *
  *
  ******************************************************************************
  */

#ifndef __LIS3MDL_H__
#define __LIS3MDL_H__
#include "stm32h7xx_hal.h"
#include <stdint.h>

/* LIS3MDL Register Map */
#define LIS3MDL_REG_OFFSET_X_L_M        0x05
#define LIS3MDL_REG_OFFSET_X_H_M        0x06
#define LIS3MDL_REG_OFFSET_Y_L_M        0x07
#define LIS3MDL_REG_OFFSET_Y_H_M        0x08
#define LIS3MDL_REG_OFFSET_Z_L_M        0x09
#define LIS3MDL_REG_OFFSET_Z_H_M        0x0A

#define LIS3MDL_REG_WHO_AM_I            0x0F

#define LIS3MDL_REG_CTRL1               0x20
#define LIS3MDL_REG_CTRL2               0x21
#define LIS3MDL_REG_CTRL3               0x22
#define LIS3MDL_REG_CTRL4               0x23
#define LIS3MDL_REG_CTRL5               0x24

#define LIS3MDL_REG_REG                 0x27
#define LIS3MDL_REG_OUT_X_L             0x28
#define LIS3MDL_REG_OUT_X_H             0x29
#define LIS3MDL_REG_OUT_Y_L             0x2A
#define LIS3MDL_REG_OUT_Y_H             0x2B
#define LIS3MDL_REG_OUT_Z_L             0x2C
#define LIS3MDL_REG_OUT_Z_H             0x2D
#define LIS3MDL_REG_TEMP_OUT_L          0x2E
#define LIS3MDL_REG_TEMP_OUT_H          0x2F

#define LIS3MDL_REG_INT_CFG             0x30
#define LIS3MDL_REG_INT_SRC             0x31
#define LIS3MDL_REG_THS_L               0x32
#define LIS3MDL_REG_THS_H               0x33

/* LIS3MDL CTRL_REG1 Configuration Map */
#define LIS3MDL_TEMP_EN                 0b10000000
#define LIS3MDL_TEMP_DIS                0b00000000

#define LIS3MDL_LP_1000Hz               0b00000010
#define LIS3MDL_MP_560Hz                0b00100010
#define LIS3MDL_HP_300Hz                0b01000010
#define LIS3MDL_UHP_155Hz               0b01100010

#define LIS3MDL_LP_0Hz625               0b00000000
#define LIS3MDL_LP_1Hz25                0b00000100
#define LIS3MDL_LP_2Hz5                 0b00001000
#define LIS3MDL_LP_5Hz                  0b00001100
#define LIS3MDL_LP_10Hz                 0b00010000
#define LIS3MDL_LP_20Hz                 0b00010100
#define LIS3MDL_LP_40Hz                 0b00011000
#define LIS3MDL_LP_80Hz                 0b00011100

#define LIS3MDL_MP_0Hz625               0b00100000
#define LIS3MDL_MP_1Hz25                0b00100100
#define LIS3MDL_MP_2Hz5                 0b00101000
#define LIS3MDL_MP_5Hz                  0b00101100
#define LIS3MDL_MP_10Hz                 0b00110000
#define LIS3MDL_MP_20Hz                 0b00110100
#define LIS3MDL_MP_40Hz                 0b00111000
#define LIS3MDL_MP_80Hz                 0b00111100

#define LIS3MDL_HP_0Hz625               0b01000000
#define LIS3MDL_HP_1Hz25                0b01000100
#define LIS3MDL_HP_2Hz5                 0b01001000
#define LIS3MDL_HP_5Hz                  0b01001100
#define LIS3MDL_HP_10Hz                 0b01010000
#define LIS3MDL_HP_20Hz                 0b01010100
#define LIS3MDL_HP_40Hz                 0b01011000
#define LIS3MDL_HP_80Hz                 0b01011100

#define LIS3MDL_UHP_0Hz625              0b01100000
#define LIS3MDL_UHP_1Hz25               0b01100100
#define LIS3MDL_UHP_2Hz5                0b01101000
#define LIS3MDL_UHP_5Hz                 0b01101100
#define LIS3MDL_UHP_10Hz                0b01110000
#define LIS3MDL_UHP_20Hz                0b01110100
#define LIS3MDL_UHP_40Hz                0b01111000
#define LIS3MDL_UHP_80Hz                0b01111100

#define LIS3MDL_SELF_TEST_EN            0b00000001
#define LIS3MDL_SELF_TEST_DIS           0b00000000

/* LIS3MDL CTRL_REG2 Configuration Map */
#define LIS3MDL_FS_4Gauss               0b00000000
#define LIS3MDL_FS_8Gauss               0b00100000
#define LIS3MDL_FS_12Gauss              0b01000000
#define LIS3MDL_FS_16Gauss              0b01100000

// https://community.st.com/t5/mems-sensors/lis3mdl-understanding-reboot-and-soft-rst/td-p/362844
#define LIS3MDL_REBOOT_MEMORY           0b00001000
#define LIS3MDL_SOFT_RST                0b00000100

/* LIS3MDL CTRL_REG3 Configuration Map */
#define LIS3MDL_LOW_POWER               0b00100000
#define LIS3MDL_SPI_4_WIRE              0b00000000
#define LIS3MDL_SPI_3_WIRE              0b00000100

#define LIS3MDL_CONTINUOUS_CONVERSION   0b00000000
#define LIS3MDL_SINGLE_CONVERSION       0b00000001
#define LIS3MDL_POWER_DOWN              0b00000010

/* LIS3MDL CTRL_REG4 Configuration Map */
#define LIS3MDL_Z_LP                    0b00000000
#define LIS3MDL_Z_MP                    0b00000100
#define LIS3MDL_Z_HP                    0b00001000
#define LIS3MDL_Z_UHP                   0b00001100

#define LIS3MDL_LITTLE_ENDIAN           0b00000000
#define LIS3MDL_BIG_ENDIAN              0b00000010

/* LIS3MDL CTRL_REG5 Configuration Map */
#define LIS3MDL_FAST_READ_EN            0b10000000
#define LIS3MDL_FAST_READ_DIS           0b00000000
#define LIS3MDL_BLOCK_UPDATE_EN         0b01000000
#define LIS3MDL_BLOCK_UPDATE_DIS        0b00000000

/* LIS3MSL DEFINES */

enum lis3mdl_err {
    LIS3MDL_ERR_OK,
    LIS3MDL_ERR_GENERAL,
};

struct lis3mdl_device {
    uint8_t temp_enable;
    uint8_t data_rate;
    uint8_t self_test;
    uint8_t full_scale;
    uint8_t z_axis_mode;
    uint8_t endianness;
    SPI_HandleTypeDef *spi_handle;
    GPIO_TypeDef *cs_pin;
    uint16_t cs_pin_port;
};

enum lis3mdl_err lis3mdl_initialize(struct lis3mdl_device *device);
enum lis3mdl_err lis3mdl_write_hard_iron(struct lis3mdl_device *device, double *hard_iron_offset);
enum lis3mdl_err lis3mdl_read_hard_iron(struct lis3mdl_device *device, double *hard_iron_offset);
enum lis3mdl_err lis3mdl_sensitivity_get(struct lis3mdl_device *device, double *sensitivity);
enum lis3mdl_err lis3mdl_read_temp(struct lis3mdl_device *device, double *temp);

enum lis3mdl_err lis3mdl_read_mag(struct lis3mdl_device *device, double *mag_reading);

enum lis3mdl_err lis3mdl_write_register(struct lis3mdl_device *device, uint8_t reg, uint8_t data);
enum lis3mdl_err lis3mdl_read_register(struct lis3mdl_device *device, uint8_t reg, uint8_t *data);
enum lis3mdl_err lis3mdl_write_multiple_registers(struct lis3mdl_device *device, uint8_t start_reg, uint8_t bytes, uint8_t *data);
enum lis3mdl_err lis3mdl_read_multiple_registers(struct lis3mdl_device *device, uint8_t start_reg, uint8_t bytes, uint8_t *data);

#endif /* __LIS3MDL_H__ */