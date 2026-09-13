#ifndef LSM6DSO_H
#define LSM6DSO_H

#include "main.h"
#include <stm32h7xx_hal_spi.h>


#define LSM6DSO_REGISTER_WHO_AM_I_SIZE 1
#define LSM6DSO_REGISTER_OUTX_G_SIZE 2
#define LSM6DSO_REGISTER_OUTY_G_SIZE 2
#define LSM6DSO_REGISTER_OUTZ_G_SIZE 2

#define LSM6DSO_REGISTER_WHO_AM_I_VALUE 0x6C

/** Accelerometer configuration:
 * - 6.66 kHZ
 * - +/- 2g
 */
#define LSM6DSO_CONFIG_CTRL1_XL 0b10100000

/** Gyro configuration:
 * - 6.66 kHZ
 * - +/- 250 dps
 */
#define LSM6DSO_CONFIG_CTRL2_G 0b10100000

/**
 * - Enables Block data update (BDU)
 */
#define LSM6DSO_CONFIG_CTRL3_C 0b01000100

// From table 2 at +/- 250 dps (in mdps/LSB)
#define LSM6DSO_GYRO_So_250DPS 8.75f

#define LSM6DSO_GYRO_NATIVE_UNITS_TO_DEGREES_PER_SECOND (LSM6DSO_GYRO_So_250DPS / 1000.0f)

#define LSM6DSO_T_on_ms 35
#define LSM6DSO_T_su_cs_us 1
#define LSM6DSO_T_h_cs_us 1

typedef enum {
    LSM6DSO_REGISTER_CTRL1_XL = 0x10,
    LSM6DSO_REGISTER_CTRL2_G = 0x11,
    LSM6DSO_REGISTER_CTRL3_C = 0x12,
    LSM6DSO_REGISTER_WHO_AM_I = 0x0F,
    LSM6DSO_REGISTER_OUTX_L_G = 0x22,
    LSM6DSO_REGISTER_OUTY_L_G = 0x24,
    LSM6DSO_REGISTER_OUTZ_L_G = 0x26
} LSM6DSO_Register;

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

/** Reads the value contained in a lsm6dso register  */
bool lsm6dso_read_register(SPI_HandleTypeDef* spi_handle, LSM6DSO_Register reg, uint8_t* data, uint16_t size);

/** Write a value to a lsm6dso register  */
bool lsm6dso_write_register(SPI_HandleTypeDef* spi_handle, LSM6DSO_Register reg, const uint8_t* data);

/** Utility method that waits for the specified duration in microseconds */
void delay_us(unsigned int);

#endif
