/**
 ******************************************************************************
 * @file    ADIS16500.c
 * @author  Kanav Chugh
 * @brief   ADIS16500 source file
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */


#include "ADIS16500.h"
#include "arm_math.h"

/**
 * @brief initializes DWT for advanced clock control
 * 
*/
void DWT_Init(void) {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; 
        DWT->CYCCNT = 0; 
    }
}

/**
 * @brief delays system thread by time in microseconds
 * @param microseconds time in microseconds to delay
*/
void delay_us(uint32_t microseconds) {
    uint32_t startTick = DWT->CYCCNT,
            delayTicks = microseconds * (SystemCoreClock / 1000000); 
    while (DWT->CYCCNT - startTick < delayTicks); 
}


/**
 * @brief reads a register given the memory address
 * @param device instance of ADIS IMU device
 * @param addr address to read data from
 * @return data in bits, MSB first
*/
int16_t adis_read_register(struct ADIS_Device *device, uint8_t addr) {
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t)device->cs_pin_port, GPIO_PIN_RESET);
	uint8_t address[2] = {addr, 0x00};
	HAL_SPI_Transmit((SPI_HandleTypeDef*)device->spi_handle, address, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t)device->cs_pin_port, GPIO_PIN_SET);
	delay_us(16);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t)device->cs_pin_port, GPIO_PIN_RESET);
	uint8_t txbuf[2] = {0x00, 0x00};
	uint8_t rxbuf[2];
	HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)device->spi_handle, txbuf, rxbuf, 2, 150);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t)device->cs_pin_port, GPIO_PIN_SET);
	return (rxbuf[1] << 8) | (rxbuf[0] & 0xFF);
}

/**
 * @brief writes a value to a register, given the value
 * @param device instance of ADIS IMU device
 * @param addr register address to write to
 * @param value value to write to register
*/
void adis_write_register(struct ADIS_Device *device, uint8_t addr, uint16_t value) {
	uint16_t address = (addr | 0x80) << 8;
	uint16_t low_word = (address | (value & 0xFF));
	uint16_t high_word = (address | 0x100) | ((value >> 8) & 0xFF);
	uint8_t upper_word[2] = {high_word >> 8, high_word & 0xFF};
	uint8_t lower_word[2] = {low_word >> 8, low_word & 0xFF};

	/* Writing words to SPI channel */
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t) device->cs_pin_port, GPIO_PIN_RESET);
	HAL_SPI_Transmit((SPI_HandleTypeDef*)device->spi_handle, upper_word, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t) device->cs_pin_port, GPIO_PIN_SET);
	delay_us(5);

	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t) device->cs_pin_port, GPIO_PIN_RESET);
	HAL_SPI_Transmit((SPI_HandleTypeDef*)device->spi_handle, lower_word, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device->cs_pin, (uint16_t) device->cs_pin_port, GPIO_PIN_SET);
	delay_us(5);

}

/**
 * @brief Reads gyroscope data from the ADIS IMU
 * @param device Pointer to the ADIS IMU device instance
 * @param gyro_readings Array to store the gyroscope readings (x, y, z)
 * @note The gyroscope readings are in degrees per second
 */
void adis_read_gyro(struct ADIS_Device *device, float32_t gyro_readings[3]) {
    gyro_readings[0] = (float)(adis_read_register(device, ADIS_X_GYRO_OUT)) * 0.1f;
    gyro_readings[1] = (float)(adis_read_register(device, ADIS_Y_GYRO_OUT)) * 0.1f;
    gyro_readings[2] = (float)(adis_read_register(device, ADIS_Z_GYRO_OUT)) * 0.1f;
}

/**
 * @brief Reads accelerometer data from the ADIS IMU
 * @param device Pointer to the ADIS IMU device instance
 * @param accel_readings Array to store the accelerometer readings (x, y, z)
 * @note The accelerometer readings are in g-forces
 */
void adis_read_accel(struct ADIS_Device *device, float32_t accel_readings[3]) {
    accel_readings[0] = (float) (adis_read_register(device, ADIS_X_ACCL_OUT)) * 0.01225f;
    accel_readings[1] = (float) (adis_read_register(device, ADIS_Y_ACCL_OUT)) * 0.01225f;
    accel_readings[2] = (float) (adis_read_register(device, ADIS_Z_ACCL_OUT)) * 0.01225f;
}
