#include "lsm6dso.h"

/** Initializes the lsm6dso peripheral and checks the WHO_AM_I register
 *
 * @returns True if the sensor was successfully initialized and the WHO_AM_I register is correct,
 *          and false otherwise
 */
bool lsm6dso_initialize(SPI_HandleTypeDef* spi_handle) {
    // TODO: implement
    UNUSED(spi_handle);
    return false;
}

/** Reads and outputs the current reported angular pitch rate from the IMU in degrees/sec */
float lsm6dso_get_pitch_rate(SPI_HandleTypeDef* spi_handle) {
    // TODO: implement
    UNUSED(spi_handle);
    return 0;
}

/** Reads and outputs the current reported angular yaw rate from the IMU in degrees/sec */
float lsm6dso_get_yaw_rate(SPI_HandleTypeDef* spi_handle) {
    // TODO: implement
    UNUSED(spi_handle);
    return 0;
}

/** Reads and outputs the current reported angular roll rate from the IMU in degrees/sec */
float lsm6dso_get_roll_rate(SPI_HandleTypeDef* spi_handle) {
    // TODO: implement
    UNUSED(spi_handle);
    return 0;
}

/** Utility method that waits for the specified duration in microseconds */
void delay_us(unsigned int microseconds) {
    // THIS METHOD HAS BEEN IMPLEMENTED FOR YOU
    // IT IS NOT RECOMMENDED TO MODIFY THIS METHOD
    __HAL_TIM_SetCounter(&htim2, 0);
    while (__HAL_TIM_GetCounter(&htim2) < microseconds);
}
