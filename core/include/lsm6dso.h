#ifndef LSM6DSO_H
#define LSM6DSO_H

#include "main.h"
#include <stm32h7xx_hal_spi.h>
#include <stdint.h>

#define WHO_AM_I_REG_VALUE 0x6C
#define WHO_AM_I_REG 0x0F

#define CTRL1_XL_REG 0x10
#define CTRL2_G_REG 0x11
#define CTRL3_C_REG 0x12

#define OUTX_H_G 0x23
#define OUTX_L_G 0x22
#define OUTY_H_G 0x25
#define OUTY_L_G 0x24
#define OUTZ_H_G 0x27
#define OUTZ_L_G 0x26


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

/** Reads specific register and returns value */
uint8_t lms6dso_read_register(SPI_HandleTypeDef* spi_handle, uint8_t reg);

/** Writes data in specific register and returns true if succesfull */
bool lms6dso_write_register(SPI_HandleTypeDef* spi_handle, uint8_t reg, uint8_t data);

#endif
