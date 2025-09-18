#include "magnetometer.h"

/**
 * @brief initializes magnetometer
 * This function initializes the magnetometer based on settings in device structure
 * @param device: device pointer to lis3mdl_device struct
 * @returns if the magnetometer is ok
*/

enum magnetometer_err magnetometer_initialize() {
    magnetometer_write_register(MAG_REG_CTRL3, MAG_CONTINUOUS_CONVERSION);
    uint8_t ctrl_reg_1 = HALAL_MAGNETOMETER_TEMP_ENABLE | HALAL_MAGNETOMETER_DATA_RATE | HALAL_MAGNETOMETER_SELF_TEST;
    magnetometer_write_register(MAG_REG_CTRL1, ctrl_reg_1);
    uint8_t ctrl_reg_2 = HALAL_MAGNETOMETER_FULL_SCALE;
    magnetometer_write_register(MAG_REG_CTRL2, ctrl_reg_2);
    uint8_t ctrl_reg_4 = HALAL_MAGNETOMETER_Z_AXIS_MODE | HALAL_MAGNETOMETER_ENDIANNESS;
    magnetometer_write_register(MAG_REG_CTRL4, ctrl_reg_4);
    return MAG_ERR_OK;
}

/**
 * @brief read magnetic field vector from device
 * This function reads magnetic field from device and converts the value into double precision.
 * This stores the vector into a provided array. Magnetic field is represented in Gauss.
 * @param mag_reading: pointer to 3-element doubly array to store magnetic field reading in Gauss.
 * @warning: no error checking is performed. Make sure to allocate appropriate array for inputs
*/

enum magnetometer_err magnetometer_read_mag(double *mag_reading) {
    uint8_t mag_read_buf[6];
    double sensitivity = 0;
    magnetometer_read_multiple_registers(MAG_REG_OUT_X_L, 6, mag_read_buf);
    int16_t x_reading = (mag_read_buf[1] << 8) | mag_read_buf[0]; 
    int16_t y_reading = (mag_read_buf[3] << 8) | mag_read_buf[1]; 
    int16_t z_reading = (mag_read_buf[5] << 8) | mag_read_buf[2]; 
    magnetometer_sensitivity_get(&sensitivity);
    mag_reading[0] = (double) x_reading / sensitivity;
    mag_reading[1] = (double) y_reading / sensitivity;
    mag_reading[2] = (double) z_reading / sensitivity;
    return MAG_ERR_OK;
}

/**
 * @brief read temperature from device
 * This function reads temperature sensor on device and converts it double precision in degrees C
 * @param temp: Pointer to double to store measured temperature in degrees C
 * @warning This function only works if LIS3MDL_TEMP_EN is written to LIS3MDL_REG_CTRL_1 during initialization
*/

enum magnetometer_err magnetometer_read_temp(double *temp) {
    uint8_t temp_read_buff[2];
    magnetometer_read_multiple_registers(MAG_REG_TEMP_OUT_L, 2, temp_read_buff);
    int16_t temp_reading = (temp_read_buff[1] << 8) | temp_read_buff[0];
    *temp = (double) temp_reading / 8.0f + 25.0f;
    return MAG_ERR_OK;
}

/**
 * @brief write hard iron offset to device
 * This function writes a user-supplied hard-iron offset to device to provide more accurate outputs
 * @param hard_iron_offset: Pointer to 3-element double array having hard-iron offset in Gauss
 * @warning: No error checking
*/

enum magnetometer_err magnetometer_write_hard_iron(double *hard_iron_offset) {
    int16_t hard_iron_ints[3];
    double sensitivity = 0;
    magnetometer_sensitivity_get(&sensitivity);
    hard_iron_ints[0] = hard_iron_offset[0] * sensitivity;
    hard_iron_ints[1] = hard_iron_offset[1] * sensitivity;
    hard_iron_ints[2] = hard_iron_offset[2] * sensitivity;
    return MAG_ERR_OK;
}

/**
 * @brief get the sensitivity of the device
 *
 * This function determines the sensitivity of the device based on the setting contained in the device structure
 * @param sensitivity pointer to a double to store the determined sensitivity
 * @warning this function determines the sensitivity based off the settings in the device structure. If these settings
 * do not match what is actually contained in the LIS3MDL_REG_CTRL2 register the sensitivty may be inaccurate.
*/
enum magnetometer_err magnetometer_sensitivity_get(double *sensitivity) {
    switch (HALAL_MAGNETOMETER_FULL_SCALE) {
        case LIS3MDL_FS_4Gauss:
            *sensitivity = 6842.0f;
            break;
        case LIS3MDL_FS_8Gauss:
            *sensitivity = 3421.0f;
            break;
        case LIS3MDL_FS_12Gauss:
            *sensitivity = 2281.0f;
            break;
        case LIS3MDL_FS_16Gauss:
            *sensitivity = 1711.0f;
            break;
        default:
            return MAG_ERR_GENERAL;
    }
    return MAG_ERR_OK;
}

/** 
 * @brief Write to a single register 
 * 
 * This function performs a single register write on a device though the STM32 SPI HAL.
 * @param reg register to write to 
 * @param data data to write to register
*/

enum magnetometer_err magnetometer_write_register(uint8_t reg, uint8_t data) {
    uint8_t transmit_buf[2] = {reg, data};
    HAL_GPIO_WritePin((GPIO_TypeDef *)HALAL_MAGNETOMETER_CS_PIN_PORT, (uint16_t)HALAL_MAGNETOMETER_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit((SPI_HandleTypeDef *)HALAL_MAGNETOMETER_SPI_HANDLE, transmit_buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin((GPIO_TypeDef *)HALAL_MAGNETOMETER_CS_PIN_PORT, (uint16_t)HALAL_MAGNETOMETER_CS_PIN, GPIO_PIN_SET);
    return MAG_ERR_OK;
}

/**
 * @brief Read a single register
 * 
 * This function performs a single register read on a device through the STM32 SPI HAL.
 * @param reg register to read 
 * @param data pointer to buffer to store read byte
*/
enum magnetometer_err magnetometer_read_register(uint8_t reg, uint8_t *data) {
    uint8_t transmit_buf[2] = {0x80 | reg, 0x00};
    uint8_t receive_buf[2];
    HAL_GPIO_WritePin((GPIO_TypeDef *)HALAL_MAGNETOMETER_CS_PIN_PORT, (uint16_t)HALAL_MAGNETOMETER_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive((SPI_HandleTypeDef *)HALAL_MAGNETOMETER_SPI_HANDLE, transmit_buf, receive_buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin((GPIO_TypeDef *)HALAL_MAGNETOMETER_CS_PIN_PORT, (uint16_t)HALAL_MAGNETOMETER_CS_PIN, GPIO_PIN_SET);
    *data = receive_buf[1];
    return MAG_ERR_OK;
}

/** 
 * @brief Write to multiple registers
 * 
 * This function perfomrs a multiple-write on a device through the STM32 SPI HAL. The function takes in the first register
 * and uses the auto-incrementation function of the device to write to consecutive registers
 * @param start_reg first register to write to  
 * @param bytes number of bytes to write 
 * @param data pointer to buffer with data to write 
 * @warning no error checking is performed. Make sure to allocate appropriate buffer sizes for all inputs. 
**/
enum magnetometer_err magnetometer_write_multiple_registers(uint8_t start_reg, uint8_t bytes, uint8_t *data) {
    uint8_t transmit_buf[bytes + 1];
    for (int i = 1; i <= bytes; i ++) {
        transmit_buf[i] = data[i - 1];
    }
    transmit_buf[0] = 0x40 | start_reg;
    HAL_GPIO_WritePin((GPIO_TypeDef *)HALAL_MAGNETOMETER_CS_PIN_PORT, (uint16_t)HALAL_MAGNETOMETER_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit((SPI_HandleTypeDef *)HALAL_MAGNETOMETER_SPI_HANDLE, transmit_buf, bytes + 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin((GPIO_TypeDef *)HALAL_MAGNETOMETER_CS_PIN_PORT, (uint16_t)HALAL_MAGNETOMETER_CS_PIN, GPIO_PIN_SET);
    return MAG_ERR_OK;
}

/** 
 * @brief Read multiple registers
 * 
 * This function performs a mulitple-read on a device through the STM32 SPI HAL. The function takes in the 
 * first register and uses the auto-incrementation function of the device to read consecutive registers.
 * @param start_reg first register to read
 * @param bytes number of consecutive registers to read
 * @param data pointer to buffer to store read bytes
 * @warning no error checking is performed. Make sure to allocate appropriate buffer sizes for all inputs. 
*/


enum magnetometer_err magnetometer_read_multiple_registers(uint8_t start_reg, uint8_t bytes, uint8_t *data) {
    // TODO: error handling
    uint8_t transmit_buf[bytes + 1];
    uint8_t receive_buf[bytes + 1];
    for (int i = 1; i <= bytes; i++) {
        transmit_buf[i] = 0x00;
    }
    transmit_buf[0] = 0xC0 | start_reg;
    HAL_GPIO_WritePin((GPIO_TypeDef *)HALAL_MAGNETOMETER_CS_PIN_PORT, (uint16_t)HALAL_MAGNETOMETER_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive((SPI_HandleTypeDef *)HALAL_MAGNETOMETER_SPI_HANDLE, transmit_buf, receive_buf, bytes + 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin((GPIO_TypeDef *)HALAL_MAGNETOMETER_CS_PIN_PORT, (uint16_t)HALAL_MAGNETOMETER_CS_PIN, GPIO_PIN_SET);
    for (int i = 0; i < bytes; i++) {
        data[i] = receive_buf[i + 1];
    }
    return MAG_ERR_OK;
}