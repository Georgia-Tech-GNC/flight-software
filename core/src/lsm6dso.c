#include "lsm6dso.h"
#include <stdint.h>
#include <stdbool.h>

// enable gyroscope
void lsm6dso_enable_gyroscope(SPI_HandleTypeDef* spi_handle) {

    // pull CS low
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

    // CTRL2_G register is 0x11
    // top 4 bits = ODR_G, next 2 bits = FS1_G/FS0_G, then FS_125, then unused
    // 0100 (104 Hz) + 01 (500 dps) + 0 (FS_125 off) + 0 = 0x44
    // write bit is 0
    uint8_t reg_to_write = 0x11;
    HAL_SPI_Transmit(spi_handle, &reg_to_write, 1, HAL_MAX_DELAY);

    // value to write into CTRL2_G
    uint8_t value_to_write = 0x44;
    HAL_SPI_Transmit(spi_handle, &value_to_write, 1, HAL_MAX_DELAY);

    // pull CS high
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

/** Initializes the lsm6dso peripheral and checks the WHO_AM_I register
 *
 * @returns True if the sensor was successfully initialized and the WHO_AM_I register is correct,
 *          and false otherwise
 */
bool lsm6dso_initialize(SPI_HandleTypeDef* spi_handle) {
    // pull cs low, read who_am_i register, pull cs high, compare with expected who_am_i value from datasheet

    // pull CS low
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

    // WHO_AM_I register is 0x0F = 0x6C

    // tell the imu what register we want to read
    // 5.1.2 in datasheet
    uint8_t reg_to_read = 0x0F | (1 << 7);
    HAL_SPI_Transmit(spi_handle, &reg_to_read, 1, HAL_MAX_DELAY);

    // receive 1 byte from that register
    uint8_t value;
    HAL_SPI_Receive(spi_handle, &value, 1, HAL_MAX_DELAY);

    // pull CS high
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    // compare value from register to expected value
    if (value == 0x6C) {
        lsm6dso_enable_gyroscope(spi_handle);
        return true;
    }

    return false;
}


/** Reads and outputs the current reported angular pitch rate from the IMU in degrees/sec */
float lsm6dso_get_pitch_rate(SPI_HandleTypeDef* spi_handle) {

    // pull cs low, transmit the register(s) we want to read, receive the data from the registers, ..., pull cs high

    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    uint8_t low_register = 0x22 | (1 << 7);
    uint8_t low_byte;
    HAL_SPI_Transmit(spi_handle, &low_register, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi_handle, &low_byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    uint8_t high_register = 0x23 | (1 << 7);
    uint8_t high_byte;
    HAL_SPI_Transmit(spi_handle, &high_register, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi_handle, &high_byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    int16_t data = (int16_t)((high_byte << 8) | low_byte);

    // 500 dsp = 17.50 mdps, mdps/LSB = data?
    float pitch = (0.0175) * data;

    return pitch;
}

/** Reads and outputs the current reported angular yaw rate from the IMU in degrees/sec */
float lsm6dso_get_yaw_rate(SPI_HandleTypeDef* spi_handle) {

    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    uint8_t low_register = 0x24 | (1 << 7);
    uint8_t low_byte;
    HAL_SPI_Transmit(spi_handle, &low_register, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi_handle, &low_byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    uint8_t high_register = 0x25 | (1 << 7);
    uint8_t high_byte;
    HAL_SPI_Transmit(spi_handle, &high_register, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi_handle, &high_byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    int16_t data = (int16_t)((high_byte << 8) | low_byte);
    float yaw = (0.0175) * data;

    return yaw;
}

/** Reads and outputs the current reported angular roll rate from the IMU in degrees/sec */
float lsm6dso_get_roll_rate(SPI_HandleTypeDef* spi_handle) {
 
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    uint8_t low_register = 0x26 | (1 << 7);
    uint8_t low_byte;
    HAL_SPI_Transmit(spi_handle, &low_register, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi_handle, &low_byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    uint8_t high_register = 0x27 | (1 << 7);
    uint8_t high_byte;
    HAL_SPI_Transmit(spi_handle, &high_register, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi_handle, &high_byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    int16_t data = (int16_t)((high_byte << 8) | low_byte);
    float roll = (0.0175) * data;

    return roll;
}

/** Utility method that waits for the specified duration in microseconds */
void delay_us(unsigned int microseconds) {
    // THIS METHOD HAS BEEN IMPLEMENTED FOR YOU
    // IT IS NOT RECOMMENDED TO MODIFY THIS METHOD
    __HAL_TIM_SetCounter(&htim2, 0);
    while (__HAL_TIM_GetCounter(&htim2) < microseconds);
}
