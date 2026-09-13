#include "lsm6dso.h"

#include <string.h>

/** Initializes the lsm6dso peripheral and checks the WHO_AM_I register
 *
 * @returns True if the sensor was successfully initialized and the WHO_AM_I register is correct,
 *          and false otherwise
 */
bool lsm6dso_initialize(SPI_HandleTypeDef* spi_handle) {
    HAL_Delay(LSM6DSO_T_on_ms);
    
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    delay_us(LSM6DSO_T_h_cs_us);

    uint8_t result = 0;
    if (!lsm6dso_read_register(spi_handle, LSM6DSO_REGISTER_WHO_AM_I, &result, LSM6DSO_REGISTER_WHO_AM_I_SIZE))
        return false;

    if (result != LSM6DSO_REGISTER_WHO_AM_I_VALUE)
        return false;

    // Configure device
    uint8_t ctrl1_val = LSM6DSO_CONFIG_CTRL1_XL;
    if (!lsm6dso_write_register(spi_handle, LSM6DSO_REGISTER_CTRL1_XL, &ctrl1_val))
        return false;

    uint8_t ctrl2_val = LSM6DSO_CONFIG_CTRL2_G;
    if (!lsm6dso_write_register(spi_handle, LSM6DSO_REGISTER_CTRL2_G, &ctrl2_val))
        return false;

    uint8_t ctr3_val = LSM6DSO_CONFIG_CTRL3_C;
    if (!lsm6dso_write_register(spi_handle, LSM6DSO_REGISTER_CTRL3_C, &ctr3_val))
        return false;

    return true;
}

/** Reads and outputs the current reported angular pitch rate from the IMU in degrees/sec */
float lsm6dso_get_pitch_rate(SPI_HandleTypeDef* spi_handle) {
    uint8_t recv[] = { 0, 0 };
    lsm6dso_read_register(spi_handle, LSM6DSO_REGISTER_OUTX_L_G, recv, LSM6DSO_REGISTER_OUTX_G_SIZE);
    int16_t pitch_rate_native = (int16_t)((int16_t)recv[1] << 8 | recv[0]);

    return (float)pitch_rate_native * LSM6DSO_GYRO_NATIVE_UNITS_TO_DEGREES_PER_SECOND;
}

/** Reads and outputs the current reported angular yaw rate from the IMU in degrees/sec */
float lsm6dso_get_yaw_rate(SPI_HandleTypeDef* spi_handle) {
    uint8_t recv[] = { 0, 0 };
    lsm6dso_read_register(spi_handle, LSM6DSO_REGISTER_OUTZ_L_G, recv, LSM6DSO_REGISTER_OUTZ_G_SIZE);
    int16_t yaw_rate_native = (int16_t)((int16_t)recv[1] << 8 | recv[0]);

    return (float)yaw_rate_native * LSM6DSO_GYRO_NATIVE_UNITS_TO_DEGREES_PER_SECOND;
}

/** Reads and outputs the current reported angular roll rate from the IMU in degrees/sec */
float lsm6dso_get_roll_rate(SPI_HandleTypeDef* spi_handle) {
    uint8_t recv[] = { 0, 0 };
    lsm6dso_read_register(spi_handle, LSM6DSO_REGISTER_OUTY_L_G, recv, LSM6DSO_REGISTER_OUTY_G_SIZE);
    int16_t roll_rate_native = (int16_t)((int16_t)recv[1] << 8 | recv[0]);

    return (float)roll_rate_native * LSM6DSO_GYRO_NATIVE_UNITS_TO_DEGREES_PER_SECOND;
}

bool lsm6dso_read_register(SPI_HandleTypeDef* spi_handle, LSM6DSO_Register reg, uint8_t* data, uint16_t size) {
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    delay_us(LSM6DSO_T_su_cs_us);

    uint8_t spi_read_cmd[] = { (uint8_t)(0x80 | reg), 0, 0 };
    uint8_t recv[] = { 0, 0, 0 };
    if (HAL_SPI_TransmitReceive(spi_handle, spi_read_cmd, recv, size + 1, HAL_MAX_DELAY) != HAL_OK)
        return false;
    
    memcpy(data, &recv[1], size);
    
    delay_us(LSM6DSO_T_h_cs_us);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    return true;
}

bool lsm6dso_write_register(SPI_HandleTypeDef* spi_handle, LSM6DSO_Register reg, const uint8_t* data) {
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    delay_us(LSM6DSO_T_su_cs_us);

    uint8_t spi_write_cmd[] = { (uint8_t)reg, *data };
    if (HAL_SPI_Transmit(spi_handle, spi_write_cmd, 2, HAL_MAX_DELAY) != HAL_OK)
        return false;

    delay_us(LSM6DSO_T_h_cs_us);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    return true;
}

/** Utility method that waits for the specified duration in microseconds */
void delay_us(unsigned int microseconds) {
    // THIS METHOD HAS BEEN IMPLEMENTED FOR YOU
    // IT IS NOT RECOMMENDED TO MODIFY THIS METHOD
    __HAL_TIM_SetCounter(&htim2, 0);
    while (__HAL_TIM_GetCounter(&htim2) < microseconds);
}
