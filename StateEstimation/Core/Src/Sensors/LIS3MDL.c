#include "LIS3MDL.h"


/**
 * @brief initializes LIS3MDL magnetometer
 * This function initializes the LIS3MDL magnetometer based on settings in device structure
 * @param device: device poitner to lis3mdl_device struct
 * @returns if the LIS3MDL is ok
*/

enum lis3mdl_err lis3mdl_initialize(struct lis3mdl_device *device) {
    lis3mdl_write_register(device, LIS3MDL_REG_CTRL3, LIS3MDL_CONTINUOUS_CONVERSION);
    uint8_t ctrl_reg_1 = device->temp_enable | device->data_rate | device->self_test;
    lis3mdl_write_register(device, LIS3MDL_REG_CTRL1, ctrl_reg_1);
    uint8_t ctrl_reg_2 = device->full_scale;
    lis3mdl_write_register(device, LIS3MDL_REG_CTRL2, ctrl_reg_2);
    uint8_t ctrl_reg_4 = device->z_axis_mode | device->endianness;
    lis3mdl_write_register(device, LIS3MDL_REG_CTRL4, ctrl_reg_4);
    return LIS3MDL_ERR_OK;
}

/**
 * @brief read magnetic field vector from LIS3MDL device
 * This function reads magnetic field from LIS3MDL and converts the value into double precision.
 * This stores the vector into a provided array. Magnetic field is represented in Gauss.
 * @param device: Pointer to lis3mdl structure
 * @param mag_reading: pionter to 3-element doubly array to store magnetic field reading in Gauss. 
 * @warning: no error checking is performed. Make sure to allocate appropriate array for inputs
*/

enum lis3mdl_err lis3mdl_read_mag(struct lis3mdl_device *device, double *mag_reading) {
    uint8_t mag_read_buf[6];
    double sensitivity = 0;
    lis3mdl_read_multiple_registers(device, LIS3MDL_REG_OUT_X_L, 6, mag_read_buf);
    int16_t x_reading = (mag_read_buf[1] << 8) | mag_read_buf[0]; 
    int16_t y_reading = (mag_read_buf[3] << 8) | mag_read_buf[1]; 
    int16_t z_reading = (mag_read_buf[5] << 8) | mag_read_buf[2]; 
    lis3mdl_sensitivity_get(device, &sensitivity);
    mag_reading[0] = (double) x_reading / sensitivity;
    mag_reading[1] = (double) y_reading / sensitivity;
    mag_reading[2] = (double) z_reading / sensitivity;
    return LIS3MDL_ERR_OK;
}

/**
 * @brief read temperature from LIS3MDL device
 * This function reads temperature sensor on LIS3MDL device and converts it double precision in degrees C
 * @param device: Pointer to device structure
 * @param temp: Pointer to double to store measured temperature in degrees C
 * @warning This function only works if LIS3MDL_TEMP_EN is written to LIS3MDL_REG_CTRL_1 during initialization
*/

enum lis3mdl_err lis3mdl_read_temp(struct lis3mdl_device *device, double *temp) {
    uint8_t temp_read_buff[2];
    lis3mdl_read_multiple_registers(device, LIS3MDL_REG_TEMP_OUT_L, 2, temp_read_buff);
    int16_t temp_reading = (temp_read_buff[1] << 8) | temp_read_buff[0];
    *temp = (double) temp_reading / 8.0f + 25.0f;
    return LIS3MDL_ERR_OK;
}

/**
 * @brief write hard iron offset to LIS3MDL
 * This function writes a user-supplied hard-iron offset to LIS3MDL to provide more accurate outputs
 * @param device: Pointer to device structure
 * @param hard_iron_offset: Pointer to 3-element double array having hard-iron offset in Gauss
 * @warning: No error checking
*/

enum lis3mdl_err lis3mdl_write_hard_iron(struct lis3mdl_device *device, double *hard_iron_offset) {
    int16_t hard_iron_ints[3];
    double sensitivity = 0;
    lis3mdl_sensitivity_get(device, &sensitivity);
    hard_iron_ints[0] = hard_iron_offset[0] * sensitivity;
    hard_iron_ints[1] = hard_iron_offset[1] * sensitivity;
    hard_iron_ints[2] = hard_iron_offset[2] * sensitivity;
    return LIS3MDL_ERR_OK;
}

/**
 * @brief get the sensitivity of the LIS3MDL device 
 * 
 * This function determines the sensitivity of the LIS3MDL based on the setting contained in the device structure 
 * @param device pointer to a lis3mdl_device structure 
 * @param sensitivity pointer to a double to store the determined sensitivity 
 * @warning this function determines the sensitivity based off the settings in the device structure. If these settings 
 * do not match what is actually contained in the LIS3MDL_REG_CTRL2 register the sensitivty may be inaccurate. 
*/
enum lis3mdl_err lis3mdl_sensitivity_get(struct lis3mdl_device *device, double *sensitivity) {
    switch (device->full_scale) {
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
            return LIS3MDL_ERR_GENERAL;
    }
    return LIS3MDL_ERR_OK;
}

/** 
 * @brief Write to a single LIS3MDL register 
 * 
 * This function performs a single register write on a LIS2MDL device though the STM32 SPI HAL.
 * @param device pointer to a lis3mdl_device structure 
 * @param reg register to write to 
 * @param data data to write to register
*/

enum lis3mdl_err lis3mdl_write_register(struct lis3mdl_device *device, uint8_t reg, uint8_t data) {
    uint8_t transmit_buf[2] = {reg, data};
    HAL_GPIO_WritePin((GPIO_TypeDef *)device->cs_pin_port, (uint16_t)device->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit((SPI_HandleTypeDef *)device->spi_handle, transmit_buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin((GPIO_TypeDef *)device->cs_pin_port, (uint16_t)device->cs_pin, GPIO_PIN_SET);
    return LIS3MDL_ERR_OK;
}

/**
 * @brief Read a single LIS3MDL register 
 * 
 * This function performs a single register read on a LIS3MDL device through the STM32 SPI HAL.
 * @param device pointer to a lis3mdl_device structure 
 * @param reg register to read 
 * @param data pointer to buffer to store read byte
*/
enum lis3mdl_err lis3mdl_read_register(struct lis3mdl_device *device, uint8_t reg, uint8_t *data) {
    uint8_t transmit_buf[2] = {0x80 | reg, 0x00};
    uint8_t receive_buf[2];
    HAL_GPIO_WritePin((GPIO_TypeDef*)device->cs_pin_port, (uint16_t)device->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive((SPI_HandleTypeDef *)device->spi_handle, transmit_buf, receive_buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin((GPIO_TypeDef *)device->cs_pin_port, (uint16_t)device->cs_pin, GPIO_PIN_SET);
    *data = receive_buf[1];
    return LIS3MDL_ERR_OK;
}

/** 
 * @brief Write to multile LIS3MDL registers
 * 
 * This function perfomrs a multiple-write on an LIS3MDL device through the STM32 SPI HAL. The function takes in the first register
 * and uses the auto-incrementation function of the LIS3MDL to write to consecutive registers
 * @param device pointer to a lis3mdl_device structure 
 * @param start_reg first register to write to  
 * @param bytes number of bytes to write 
 * @param data pointer to buffer with data to write 
 * @warning no error checking is performed. Make sure to allocate appropriate buffer sizes for all inputs. 
**/
enum lis3mdl_err lis3mdl_write_multiple_registers(struct lis3mdl_device *device, uint8_t start_reg, uint8_t bytes, uint8_t *data) {
    uint8_t transmit_buf[bytes + 1];
    for (int i = 1; i <= bytes; i ++) {
        transmit_buf[i] = data[i - 1];
    }
    transmit_buf[0] = 0x40 | start_reg;
    HAL_GPIO_WritePin((GPIO_TypeDef *)device->cs_pin_port, (uint16_t)device->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit((SPI_HandleTypeDef *)device->spi_handle, transmit_buf, bytes + 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin((GPIO_TypeDef *)device->cs_pin_port, (uint16_t)device->cs_pin, GPIO_PIN_SET);
    return LIS3MDL_ERR_OK;
}

/** 
 * @brief Read multiple LIS3MDL registers
 * 
 * This function performs a mulitple-read on an LIS3MDL device through the STM32 SPI HAL. The function takes in the 
 * first register and uses the auto-incrementation function of the LIS3MDL to read consecutive registers.
 * @param device pointer to a lis3mdl_device structure 
 * @param start_reg first register to read
 * @param bytes number of consecutive registers to read
 * @param data pointer to buffer to store read bytes
 * @warning no error checking is performed. Make sure to allocate appropriate buffer sizes for all inputs. 
*/


enum lis3mdl_err lis3mdl_read_multiple_registers(struct lis3mdl_device *device, uint8_t start_reg, uint8_t bytes, uint8_t *data) {
    // TODO: error handling
    uint8_t transmit_buf[bytes + 1];
    uint8_t receive_buf[bytes + 1];
    for (int i = 1; i <= bytes; i++) {
        transmit_buf[i] = 0x00;
    }
    transmit_buf[0] = 0xC0 | start_reg;
    HAL_GPIO_WritePin((GPIO_TypeDef *)device->cs_pin_port, (uint16_t)device->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive((SPI_HandleTypeDef *)device->spi_handle, transmit_buf, receive_buf, bytes + 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin((GPIO_TypeDef *)device->cs_pin_port, (uint16_t)device->cs_pin, GPIO_PIN_SET);
    for (int i = 0; i < bytes; i++) {
        data[i] = receive_buf[i + 1];
    }
    return LIS3MDL_ERR_OK;
}