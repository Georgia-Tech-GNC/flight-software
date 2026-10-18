#include "lsm6dso.h"
#include <stdio.h>

#define LSM6DSO_OUTX_L_G_REG    0x22  // X axis: OUTX_L_G, OUTX_H_G
#define LSM6DSO_OUTY_L_G_REG    0x24  // Y axis: OUTY_L_G, OUTY_H_G
#define LSM6DSO_OUTZ_L_G_REG    0x26  // Z axis: OUTZ_L_G, OUTZ_H_G


static void set_cs_low(void) {
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}

static void set_cs_high(void) {
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

static uint8_t lsm6dso_read_reg(SPI_HandleTypeDef* spi_handle, uint8_t reg) {
    uint8_t tx_data[2] = {reg | 0x80, 0x00};
    uint8_t rx_data[2] = {0};
    
    //activate cs low
    set_cs_low();
    //1microsecond delay
    HAL_Delay(100);
    //send reg address and receive data
    HAL_SPI_TransmitReceive(spi_handle, tx_data, rx_data, 2, HAL_MAX_DELAY);
    //deactivate cs
    set_cs_high();
    // char buffer[100];
    // int len = snprintf(buffer, 100, "Read reg: 0x%02X, Value: 0x%02X\r\n", reg, rx_data[1]);
    // HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, HAL_MAX_DELAY);
    return rx_data[1];
}

static void lsm6dso_write_reg(SPI_HandleTypeDef* spi_handle, uint8_t reg, uint8_t data) {
    set_cs_low();
    HAL_Delay(0.1);
    uint8_t tx_data[2] = {reg & ~0x80, data};
    HAL_SPI_Transmit(spi_handle, tx_data,2, HAL_MAX_DELAY);
    set_cs_high();
}

static int16_t lsm6dso_read_axis(SPI_HandleTypeDef* spi_handle, uint8_t reg) {
    int16_t value = 0;
    value |= (int16_t)lsm6dso_read_reg(spi_handle, reg+1) << 8;
    value |= (int16_t)lsm6dso_read_reg(spi_handle, reg);
    // char buffer[100];
    // int len = snprintf(buffer, 100, "Read axis: 0x%02X, Value: 0x%04X\r\n", reg, value);
    // HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, HAL_MAX_DELAY);
    return value;
}

static float axis_to_dps(int16_t axis) {
    float value = axis * 8.75f / 1000.0f;
    char buffer[100];    
    return value;
}




/** Initializes the lsm6dso peripheral and checks the WHO_AM_I register
 *
 * @returns True if the sensor was successfully initialized and the WHO_AM_I register is correct,
 *          and false otherwise
 */
bool lsm6dso_initialize(SPI_HandleTypeDef* spi_handle) {
    //20ms delay
    HAL_Delay(20);
    set_cs_high();

    uint8_t who_am_i = lsm6dso_read_reg(spi_handle, 0x0F);
    if (who_am_i != 0x7F) { // our imus think this is the correct value
        char msg[100];
        int len = snprintf(msg, 100, "Who_Am_I: %4X\r\n", who_am_i);
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, HAL_MAX_DELAY);
        return false;
    }
    lsm6dso_write_reg(spi_handle, 0x11, 0x40);
    lsm6dso_write_reg(spi_handle, 0x12, 0x20);
    return true;
}

/** Reads and outputs the current reported angular pitch rate from the IMU in degrees/sec */
float lsm6dso_get_pitch_rate(SPI_HandleTypeDef* spi_handle) {
    int16_t axis = lsm6dso_read_axis(spi_handle, LSM6DSO_OUTX_L_G_REG);
    return axis_to_dps(axis);
}

/** Reads and outputs the current reported angular yaw rate from the IMU in degrees/sec */
float lsm6dso_get_yaw_rate(SPI_HandleTypeDef* spi_handle) {
    int16_t axis = lsm6dso_read_axis(spi_handle, LSM6DSO_OUTY_L_G_REG);
    return axis_to_dps(axis);
}

/** Reads and outputs the current reported angular roll rate from the IMU in degrees/sec */
float lsm6dso_get_roll_rate(SPI_HandleTypeDef* spi_handle) {
    int16_t axis = lsm6dso_read_axis(spi_handle, LSM6DSO_OUTZ_L_G_REG);
    return axis_to_dps(axis);
}

/** Utility method that waits for the specified duration in microseconds */
void delay_us(unsigned int microseconds) {
    // THIS METHOD HAS BEEN IMPLEMENTED FOR YOU
    // IT IS NOT RECOMMENDED TO MODIFY THIS METHOD
    __HAL_TIM_SetCounter(&htim2, 0);
    while (__HAL_TIM_GetCounter(&htim2) < microseconds);
}
