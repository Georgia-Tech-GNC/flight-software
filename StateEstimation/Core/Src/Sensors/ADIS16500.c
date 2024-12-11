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
 * @brief reads a register given the memory address
 * @param device instance of ADIS IMU device
 * @param addr address to read data from
 * @return data in bits, MSB first
*/
int16_t adis_read_register(struct ADIS_Device *device, uint8_t addr) {
	delay_us(5);
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
	delay_us(5);
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


/**
 * @brief Scales accelerometer raw data to g-forces
 * @param raw_data Raw accelerometer data from register
 * @return Scaled accelerometer data in g's
 */
float32_t adis_accel_scale(int16_t raw_data) {
    return (float32_t)raw_data * 0.01225f;  // Using the scale factor from your existing code
}

/**
 * @brief Scales gyroscope raw data to degrees per second
 * @param raw_data Raw gyroscope data from register
 * @return Scaled gyroscope data in degrees/second
 */
float32_t adis_gyro_scale(int16_t raw_data) {
    return (float32_t)raw_data * 0.1f;  // Using the scale factor from your existing code
}

/**
 * @brief Scales temperature data to degrees Celsius
 * @param raw_data Raw temperature data from register
 * @return Scaled temperature in degrees Celsius
 */
float32_t adis_temp_scale(int16_t raw_data) {
    return (float32_t)raw_data * 0.1f;
}

/**
 * @brief Performs burst read of all sensor data
 * @param device Pointer to the ADIS IMU device instance
 * @param burst_data Array to store burst data (should be at least 10 words long)
 * @return Checksum verification status (1 if valid, 0 if invalid)
 */
uint8_t adis_burst_read(struct ADIS_Device *device, uint16_t *burst_data) {
    uint8_t buf[20];
    HAL_GPIO_WritePin((GPIO_TypeDef*)device->cs_pin, (uint16_t)device->cs_pin_port, GPIO_PIN_RESET);
    uint8_t addr[2] = {0x68, 0x00}; 
    HAL_SPI_Transmit((SPI_HandleTypeDef*)device->spi_handle, addr, 2, HAL_MAX_DELAY);
    HAL_SPI_Receive((SPI_HandleTypeDef*)device->spi_handle, buf, 20, HAL_MAX_DELAY);
    HAL_GPIO_WritePin((GPIO_TypeDef*)device->cs_pin, (uint16_t)device->cs_pin_port, GPIO_PIN_SET);  
    for(int i = 0; i < 10; i++) {
        burst_data[i] = (buf[i*2] << 8) | buf[i*2 + 1];
    }
    int16_t calc_checksum = 0;
    for(int i = 0; i < 9; i++) {
        calc_checksum += (burst_data[i] & 0xFF);
        calc_checksum += ((burst_data[i] >> 8) & 0xFF);
    }
    
    return (calc_checksum == burst_data[9]);
}

/**
 * @brief Performs hardware reset of the device
 * @param device Pointer to the ADIS IMU device instance
 * @param reset_pin GPIO pin connected to RST
 * @param reset_port GPIO port for RST pin
 * @param delay_ms Reset delay in milliseconds
 */
void adis_hardware_reset(struct ADIS_Device *device, GPIO_TypeDef* reset_pin, uint16_t reset_port, uint32_t delay_ms) {
    HAL_GPIO_WritePin(reset_pin, reset_port, GPIO_PIN_RESET);
    HAL_Delay(delay_ms);
    HAL_GPIO_WritePin(reset_pin, reset_port, GPIO_PIN_SET);
    HAL_Delay(delay_ms);  // Wait for device to stabilize
}

/**
 * @brief Parses raw burst data into scaled values
 * @param raw_data Raw burst data array
 * @param parsed_data Pointer to structure to store parsed data
 */
void adis_parse_burst(uint16_t *raw_data, struct ADIS_BurstData *parsed_data) {
    parsed_data->diag_stat = raw_data[0];
    parsed_data->gyro[0] = adis_gyro_scale(raw_data[1]);
    parsed_data->gyro[1] = adis_gyro_scale(raw_data[2]);
    parsed_data->gyro[2] = adis_gyro_scale(raw_data[3]);
    parsed_data->accel[0] = adis_accel_scale(raw_data[4]);
    parsed_data->accel[1] = adis_accel_scale(raw_data[5]);
    parsed_data->accel[2] = adis_accel_scale(raw_data[6]);
    parsed_data->temp = adis_temp_scale(raw_data[7]);
    parsed_data->timestamp = raw_data[8];
}

/**
 * @brief Reads full 32-bit gyroscope data for a single axis
 * @param device Pointer to the ADIS IMU device instance
 * @param low_reg Address of the lower register (e.g. ADIS_X_GYRO_LOW)
 * @param high_reg Address of the higher register (e.g. ADIS_X_GYRO_OUT)
 * @return 32-bit gyroscope reading
 */
int32_t adis_read_gyro_32bit(struct ADIS_Device *device, uint8_t low_reg, uint8_t high_reg) {
    int32_t high_word = (int32_t)adis_read_register(device, high_reg);
    int32_t low_word = (int32_t)adis_read_register(device, low_reg);
    return (high_word << 16) | (low_word & 0xFFFF);
}

/**
 * @brief Reads full 32-bit accelerometer data for a single axis
 * @param device Pointer to the ADIS IMU device instance 
 * @param low_reg Address of the lower register (e.g. ADIS_X_ACCL_LOW)
 * @param high_reg Address of the higher register (e.g. ADIS_X_ACCL_OUT)
 * @return 32-bit accelerometer reading
 */
int32_t adis_read_accel_32bit(struct ADIS_Device *device, uint8_t low_reg, uint8_t high_reg) {
    int32_t high_word = (int32_t)adis_read_register(device, high_reg);
    int32_t low_word = (int32_t)adis_read_register(device, low_reg);
    return (high_word << 16) | (low_word & 0xFFFF);
}

/**
 * @brief Reads 32-bit gyroscope data for all axes
 * @param device Pointer to the ADIS IMU device instance
 * @param gyro_readings Array to store the gyroscope readings (x, y, z)
 * @note The gyroscope readings are in degrees per second with full 32-bit precision
 */
void adis_read_gyro_32bit_all(struct ADIS_Device *device, float32_t gyro_readings[3]) {
    int32_t raw_x = adis_read_gyro_32bit(device, ADIS_X_GYRO_LOW, ADIS_X_GYRO_OUT);
    int32_t raw_y = adis_read_gyro_32bit(device, ADIS_Y_GYRO_LOW, ADIS_Y_GYRO_OUT);
    int32_t raw_z = adis_read_gyro_32bit(device, ADIS_Z_GYRO_LOW, ADIS_Z_GYRO_OUT);
    gyro_readings[0] = (float32_t)raw_x * 0.1f / 65536.0f;  
    gyro_readings[1] = (float32_t)raw_y * 0.1f / 65536.0f;
    gyro_readings[2] = (float32_t)raw_z * 0.1f / 65536.0f;
}

/**
 * @brief Reads 32-bit accelerometer data for all axes
 * @param device Pointer to the ADIS IMU device instance
 * @param accel_readings Array to store the accelerometer readings (x, y, z)
 * @note The accelerometer readings are in g-forces with full 32-bit precision
 */
void adis_read_accel_32bit_all(struct ADIS_Device *device, float32_t accel_readings[3]) {
    int32_t raw_x = adis_read_accel_32bit(device, ADIS_X_ACCL_LOW, ADIS_X_ACCL_OUT);
    int32_t raw_y = adis_read_accel_32bit(device, ADIS_Y_ACCL_LOW, ADIS_Y_ACCL_OUT);
    int32_t raw_z = adis_read_accel_32bit(device, ADIS_Z_ACCL_LOW, ADIS_Z_ACCL_OUT);
    accel_readings[0] = (float32_t)raw_x * 0.01225f / 65536.0f; 
    accel_readings[1] = (float32_t)raw_y * 0.01225f / 65536.0f;
    accel_readings[2] = (float32_t)raw_z * 0.01225f / 65536.0f;
}

/**
 * @brief Reads 16-bit delta angle data from the ADIS IMU
 * @param device Pointer to the ADIS IMU device instance
 * @param delta_angle Array to store the delta angle readings (x, y, z)
 * @note Delta angle readings are in degrees
 */
void adis_read_delta_angle(struct ADIS_Device *device, float32_t delta_angle[3]) {
    // Read high words only for 16-bit precision
    int16_t x = adis_read_register(device, ADIS_X_DELTANG_OUT);
    int16_t y = adis_read_register(device, ADIS_Y_DELTANG_OUT);
    int16_t z = adis_read_register(device, ADIS_Z_DELTANG_OUT);
    
    // Scale by ΔθMAX/2^15 where ΔθMAX = ±2160°
    delta_angle[0] = (float32_t)x * (2160.0f / 32768.0f);
    delta_angle[1] = (float32_t)y * (2160.0f / 32768.0f);
    delta_angle[2] = (float32_t)z * (2160.0f / 32768.0f);
}

/**
 * @brief Reads 16-bit delta velocity data from the ADIS IMU
 * @param device Pointer to the ADIS IMU device instance
 * @param delta_vel Array to store the delta velocity readings (x, y, z)
 * @note Delta velocity readings are in m/sec
 */
void adis_read_delta_vel(struct ADIS_Device *device, float32_t delta_vel[3]) {
    // Read high words only for 16-bit precision
    int16_t x = adis_read_register(device, ADIS_X_DELTVEL_OUT);
    int16_t y = adis_read_register(device, ADIS_Y_DELTVEL_OUT);
    int16_t z = adis_read_register(device, ADIS_Z_DELTVEL_OUT);
    
    // Scale by ±400 m/sec range over 16 bits
    delta_vel[0] = (float32_t)x * (400.0f / 32768.0f);
    delta_vel[1] = (float32_t)y * (400.0f / 32768.0f);
    delta_vel[2] = (float32_t)z * (400.0f / 32768.0f);
}

/**
 * @brief Reads 32-bit delta angle data
 * @param device Pointer to the ADIS IMU device instance
 * @param low_reg Address of the lower register
 * @param high_reg Address of the higher register
 * @return 32-bit delta angle reading
 */
int32_t adis_read_delta_angle_32bit(struct ADIS_Device *device, uint8_t low_reg, uint8_t high_reg) {
    int32_t high_word = (int32_t)adis_read_register(device, high_reg);
    int32_t low_word = (int32_t)adis_read_register(device, low_reg);
    return (high_word << 16) | (low_word & 0xFFFF);
}

/**
 * @brief Reads 32-bit delta velocity data
 * @param device Pointer to the ADIS IMU device instance
 * @param low_reg Address of the lower register
 * @param high_reg Address of the higher register
 * @return 32-bit delta velocity reading
 */
int32_t adis_read_delta_vel_32bit(struct ADIS_Device *device, uint8_t low_reg, uint8_t high_reg) {
    int32_t high_word = (int32_t)adis_read_register(device, high_reg);
    int32_t low_word = (int32_t)adis_read_register(device, low_reg);
    return (high_word << 16) | (low_word & 0xFFFF);
}

/**
 * @brief Reads full 32-bit precision delta angle data for all axes
 * @param device Pointer to the ADIS IMU device instance
 * @param delta_angle Array to store the delta angle readings (x, y, z)
 * @note Delta angle readings are in degrees with full 32-bit precision
 */
void adis_read_delta_angle_32bit_all(struct ADIS_Device *device, float32_t delta_angle[3]) {
    int32_t x = adis_read_delta_angle_32bit(device, ADIS_X_DELTANG_LOW, ADIS_X_DELTANG_OUT);
    int32_t y = adis_read_delta_angle_32bit(device, ADIS_Y_DELTANG_LOW, ADIS_Y_DELTANG_OUT);
    int32_t z = adis_read_delta_angle_32bit(device, ADIS_Z_DELTANG_LOW, ADIS_Z_DELTANG_OUT);

    // Scale by ΔθMAX/2^31 where ΔθMAX = ±2160°
    delta_angle[0] = (float32_t)x * (2160.0f / 2147483648.0f);
    delta_angle[1] = (float32_t)y * (2160.0f / 2147483648.0f);
    delta_angle[2] = (float32_t)z * (2160.0f / 2147483648.0f);
}

/**
 * @brief Reads full 32-bit precision delta velocity data for all axes
 * @param device Pointer to the ADIS IMU device instance
 * @param delta_vel Array to store the delta velocity readings (x, y, z)
 * @note Delta velocity readings are in m/sec with full 32-bit precision
 */
void adis_read_delta_vel_32bit_all(struct ADIS_Device *device, float32_t delta_vel[3]) {
    int32_t x = adis_read_delta_vel_32bit(device, ADIS_X_DELTVEL_LOW, ADIS_X_DELTVEL_OUT);
    int32_t y = adis_read_delta_vel_32bit(device, ADIS_Y_DELTVEL_LOW, ADIS_Y_DELTVEL_OUT);
    int32_t z = adis_read_delta_vel_32bit(device, ADIS_Z_DELTVEL_LOW, ADIS_Z_DELTVEL_OUT);
    delta_vel[0] = (float32_t)x * (400.0f / 2147483648.0f);
    delta_vel[1] = (float32_t)y * (400.0f / 2147483648.0f);
    delta_vel[2] = (float32_t)z * (400.0f / 2147483648.0f);
}