#include "lsm6dso.h"
#define WHO_AM_I 0x0F 
#define WHO_AM_I_VAL 0x6C
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define OUTX_L_G 0x22 //pitch (x)
#define OUTY_L_G 0x24 //roll (y)
#define OUTZ_L_G 0x26 //yaw (z)
#define GYRO_SENSITIVITY 0.00875f


/** Initializes the lsm6dso peripheral and checks the WHO_AM_I register
 *
 * @returns True if the sensor was successfully initialized and the WHO_AM_I register is correct,
 *          and false otherwise
 */

static bool lsm6dso_read(SPI_HandleTypeDef* spi_handle, uint8_t reg, uint8_t* data, uint16_t len)
{
    uint8_t addr = reg | 0x80;
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //CS low
    HAL_StatusTypeDef tx = HAL_SPI_Transmit(spi_handle, &addr, 1, HAL_MAX_DELAY);
    HAL_StatusTypeDef rx = HAL_SPI_Receive(spi_handle, data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //CS high
    return tx == HAL_OK && rx == HAL_OK; 
}
static bool lsm6dso_write(SPI_HandleTypeDef* spi_handle, uint8_t reg, uint8_t value)
{
    uint8_t arr[2] = {reg, value};
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef tx = HAL_SPI_Transmit(spi_handle, arr, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); 
    return tx == HAL_OK;
}
bool lsm6dso_initialize(SPI_HandleTypeDef* spi_handle) {
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    lsm6dso_write(spi_handle, CTRL3_C, 0x01);
    HAL_Delay(35);
    uint8_t id;
    if (!lsm6dso_read(spi_handle, WHO_AM_I, &id, 1) || id != WHO_AM_I_VAL) {
        return false;
    }
    lsm6dso_write(spi_handle, CTRL3_C, 0x44);   // BDU + IF_INC
    lsm6dso_write(spi_handle, CTRL2_G, 0x40);   // 104 Hz, +/-250 dps
    return true;
}

/** Reads and outputs the current reported angular pitch rate from the IMU in degrees/sec */
float lsm6dso_get_pitch_rate(SPI_HandleTypeDef* spi_handle) {
    uint8_t buf[2];
    lsm6dso_read(spi_handle, OUTX_L_G, buf, 2);
    int16_t raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    return (float)raw * GYRO_SENSITIVITY;
}

/** Reads and outputs the current reported angular yaw rate from the IMU in degrees/sec */
float lsm6dso_get_yaw_rate(SPI_HandleTypeDef* spi_handle) {
    uint8_t buf[2];
    lsm6dso_read(spi_handle, OUTZ_L_G, buf, 2);
    int16_t raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    return (float)raw * GYRO_SENSITIVITY;
}

/** Reads and outputs the current reported angular roll rate from the IMU in degrees/sec */
float lsm6dso_get_roll_rate(SPI_HandleTypeDef* spi_handle) {
    uint8_t buf[2];
    lsm6dso_read(spi_handle, OUTY_L_G, buf, 2);
    int16_t raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    return (float)raw * GYRO_SENSITIVITY;
}

/** Utility method that waits for the specified duration in microseconds */
void delay_us(unsigned int microseconds) {
    // THIS METHOD HAS BEEN IMPLEMENTED FOR YOU
    // IT IS NOT RECOMMENDED TO MODIFY THIS METHOD
    __HAL_TIM_SetCounter(&htim2, 0);
    while (__HAL_TIM_GetCounter(&htim2) < microseconds);
}
