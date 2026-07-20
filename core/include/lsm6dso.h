#ifndef LSM6DSO_H
#define LSM6DSO_H

#include "main.h"
#include <stm32h7xx_hal_spi.h>

/** Initializes the lsm6dso peripheral and checks the WHO_AM_I register
 *
 * @returns True if the sensor was successfully initialized and the WHO_AM_I register is correct,
 *          and false otherwise
 */
bool lsm6dso_initialize(SPI_HandleTypeDef* spi_handle);

/** Reads and outputs the current reported angular pitch rate from the IMU in degrees/sec */
float lsm6dso_get_pitch_rate(SPI_HandleTypeDef* spi_handle);

/** Reads and outputs the current reported angular yaw rate from the IMU in degrees/sec */
float lsm6dso_get_yaw_rate(SPI_HandleTypeDef* spi_handle);

/** Reads and outputs the current reported angular roll rate from the IMU in degrees/sec */
float lsm6dso_get_roll_rate(SPI_HandleTypeDef* spi_handle);

/** Utility method that waits for the specified duration in microseconds */
void delay_us(unsigned int);

#endif
