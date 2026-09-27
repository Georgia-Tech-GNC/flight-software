#include "lsm6dso.h"
#include <stdint.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/** Initializes the lsm6dso peripheral and checks the WHO_AM_I register
 *
 * @returns True if the sensor was successfully initialized and the WHO_AM_I register is correct,
 *          and false otherwise
 */
bool lsm6dso_initialize(SPI_HandleTypeDef* spi_handle) {
    // TODO: implement
    HAL_Delay(10);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    uint8_t id = lms6dso_read_register(spi_handle, WHO_AM_I_REG);

    char msg[50];
    sprintf(msg, "ID = 0x%02X\r\n", id);
    HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
    
    HAL_Delay(10);

    if (id != WHO_AM_I_REG_VALUE) {
        return false;
    }

    // Start accelometer
    if (!lms6dso_write_register(spi_handle, CTRL1_XL_REG, 0x60)) {
        return false;
    }

    // Start gyroscope
    if (!lms6dso_write_register(spi_handle, CTRL2_G_REG, 0x60)) {
        return false;
    }

    // Bock Data Update feature
    if (!lms6dso_write_register(spi_handle, CTRL3_C_REG, 0x40)) {
        return false;
    }

    return true;
}

/** Reads and outputs the current reported angular pitch rate from the IMU in degrees/sec */
float lsm6dso_get_pitch_rate(SPI_HandleTypeDef* spi_handle) {
    // TODO: implement
    uint8_t y_low = lms6dso_read_register(spi_handle, OUTY_L_G);
    uint8_t y_high = lms6dso_read_register(spi_handle, OUTY_H_G);
    int16_t gy = (int16_t)((y_high << 8) | y_low);

    return (float)gy * 8.75f / 1000;
}

/** Reads and outputs the current reported angular yaw rate from the IMU in degrees/sec */
float lsm6dso_get_yaw_rate(SPI_HandleTypeDef* spi_handle) {
    // TODO: implement 
    uint8_t z_low = lms6dso_read_register(spi_handle, OUTZ_L_G);
    uint8_t z_high = lms6dso_read_register(spi_handle, OUTZ_H_G);
    int16_t gz = (int16_t)((z_high << 8) | z_low);

    return (float)gz * 8.75f / 1000;
}

/** Reads and outputs the current reported angular roll rate from the IMU in degrees/sec */
float lsm6dso_get_roll_rate(SPI_HandleTypeDef* spi_handle) {
    // TODO: implement

    uint8_t x_low = lms6dso_read_register(spi_handle, OUTX_L_G);
    uint8_t x_high = lms6dso_read_register(spi_handle, OUTX_H_G);
    int16_t gx = (int16_t)((x_high << 8) | x_low);

    return (float)gx * 8.75f / 1000;
}

/** Utility method that waits for the specified duration in microseconds */
void delay_us(unsigned int microseconds) {
    // THIS METHOD HAS BEEN IMPLEMENTED FOR YOU
    // IT IS NOT RECOMMENDED TO MODIFY THIS METHOD
    __HAL_TIM_SetCounter(&htim2, 0);
    while (__HAL_TIM_GetCounter(&htim2) < microseconds);
}

uint8_t lms6dso_read_register(SPI_HandleTypeDef* spi_handle, uint8_t reg) {
    
    uint8_t tx[2];
    uint8_t rx[2];

    // Set the first bit to 1 (reading)
    tx[0] = reg | 0x80;

    // Set second byte as dummy
    tx[1] = 0x00;

    // Set CS pin to low
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    // Transmit and recieve
    HAL_SPI_TransmitReceive(spi_handle, tx, rx, 2, HAL_MAX_DELAY);

    // Set CS pin to high
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    return rx[1];
}

bool lms6dso_write_register(SPI_HandleTypeDef* spi_handle, uint8_t reg, uint8_t data) {
    
    uint8_t tx[2];

    // Set the first bit to 0 (write)
    tx[0] = reg & 0x7F;

    // Data to write
    tx[1] = data;

    // Set CS pin to low
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

    // Transmit data to register
    HAL_StatusTypeDef status = HAL_SPI_Transmit(spi_handle, tx, 2, HAL_MAX_DELAY);

    // Set CS pin to high
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

    return status == HAL_OK;
}
