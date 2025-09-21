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

enum magnetometer_err {
    MAG_ERR_OK,
    MAG_ERR_GENERAL,
};

enum magnetometer_err HALAL_magnetometer_initialize();
enum magnetometer_err HALAL_magnetometer_write_hard_iron(double *hard_iron_offset);
enum magnetometer_err HALAL_magnetometer_read_hard_iron(double *hard_iron_offset);
enum magnetometer_err HALAL_magnetometer_sensitivity_get(double *sensitivity);
enum magnetometer_err HALAL_magnetometer_read_temp(double *temp);

enum magnetometer_err HALAL_magnetometer_read_mag(double *mag_reading);

#endif /* __LIS3MDL_H__ */