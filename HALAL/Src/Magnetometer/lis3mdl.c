#include "magnetometer.h"
#include <stm32f4xx_hal_spi.h>
#include <nucleo_f429zi_halal.h>


/* LIS3MDL Register Map */
#define LIS3MDL_REG_OFFSET_X_L_M        0x05
#define LIS3MDL_REG_OFFSET_X_H_M        0x06
#define LIS3MDL_REG_OFFSET_Y_L_M        0x07
#define LIS3MDL_REG_OFFSET_Y_H_M        0x08
#define LIS3MDL_REG_OFFSET_Z_L_M        0x09
#define LIS3MDL_REG_OFFSET_Z_H_M        0x0A

#define LIS3MDL_REG_WHO_AM_I            0x0F

#define LIS3MDL_REG_CTRL1               0x20
#define LIS3MDL_REG_CTRL2               0x21
#define LIS3MDL_REG_CTRL3               0x22
#define LIS3MDL_REG_CTRL4               0x23
#define LIS3MDL_REG_CTRL5               0x24

#define LIS3MDL_REG_REG                 0x27
#define LIS3MDL_REG_OUT_X_L             0x28
#define LIS3MDL_REG_OUT_X_H             0x29
#define LIS3MDL_REG_OUT_Y_L             0x2A
#define LIS3MDL_REG_OUT_Y_H             0x2B
#define LIS3MDL_REG_OUT_Z_L             0x2C
#define LIS3MDL_REG_OUT_Z_H             0x2D
#define LIS3MDL_REG_TEMP_OUT_L          0x2E
#define LIS3MDL_REG_TEMP_OUT_H          0x2F

#define LIS3MDL_REG_INT_CFG             0x30
#define LIS3MDL_REG_INT_SRC             0x31
#define LIS3MDL_REG_THS_L               0x32
#define LIS3MDL_REG_THS_H               0x33

/* LIS3MDL CTRL_REG1 Configuration Map */
#define LIS3MDL_TEMP_EN                 0b10000000
#define LIS3MDL_TEMP_DIS                0b00000000

#define LIS3MDL_LP_1000Hz               0b00000010
#define LIS3MDL_MP_560Hz                0b00100010
#define LIS3MDL_HP_300Hz                0b01000010
#define LIS3MDL_UHP_155Hz               0b01100010

#define LIS3MDL_LP_0Hz625               0b00000000
#define LIS3MDL_LP_1Hz25                0b00000100
#define LIS3MDL_LP_2Hz5                 0b00001000
#define LIS3MDL_LP_5Hz                  0b00001100
#define LIS3MDL_LP_10Hz                 0b00010000
#define LIS3MDL_LP_20Hz                 0b00010100
#define LIS3MDL_LP_40Hz                 0b00011000
#define LIS3MDL_LP_80Hz                 0b00011100

#define LIS3MDL_MP_0Hz625               0b00100000
#define LIS3MDL_MP_1Hz25                0b00100100
#define LIS3MDL_MP_2Hz5                 0b00101000
#define LIS3MDL_MP_5Hz                  0b00101100
#define LIS3MDL_MP_10Hz                 0b00110000
#define LIS3MDL_MP_20Hz                 0b00110100
#define LIS3MDL_MP_40Hz                 0b00111000
#define LIS3MDL_MP_80Hz                 0b00111100

#define LIS3MDL_HP_0Hz625               0b01000000
#define LIS3MDL_HP_1Hz25                0b01000100
#define LIS3MDL_HP_2Hz5                 0b01001000
#define LIS3MDL_HP_5Hz                  0b01001100
#define LIS3MDL_HP_10Hz                 0b01010000
#define LIS3MDL_HP_20Hz                 0b01010100
#define LIS3MDL_HP_40Hz                 0b01011000
#define LIS3MDL_HP_80Hz                 0b01011100

#define LIS3MDL_UHP_0Hz625              0b01100000
#define LIS3MDL_UHP_1Hz25               0b01100100
#define LIS3MDL_UHP_2Hz5                0b01101000
#define LIS3MDL_UHP_5Hz                 0b01101100
#define LIS3MDL_UHP_10Hz                0b01110000
#define LIS3MDL_UHP_20Hz                0b01110100
#define LIS3MDL_UHP_40Hz                0b01111000
#define LIS3MDL_UHP_80Hz                0b01111100

#define LIS3MDL_SELF_TEST_EN            0b00000001
#define LIS3MDL_SELF_TEST_DIS           0b00000000

/* LIS3MDL CTRL_REG2 Configuration Map */
#define LIS3MDL_FS_4Gauss               0b00000000
#define LIS3MDL_FS_8Gauss               0b00100000
#define LIS3MDL_FS_12Gauss              0b01000000
#define LIS3MDL_FS_16Gauss              0b01100000

// https://community.st.com/t5/mems-sensors/lis3mdl-understanding-reboot-and-soft-rst/td-p/362844
#define LIS3MDL_REBOOT_MEMORY           0b00001000
#define LIS3MDL_SOFT_RST                0b00000100

/* LIS3MDL CTRL_REG3 Configuration Map */
#define LIS3MDL_LOW_POWER               0b00100000
#define LIS3MDL_SPI_4_WIRE              0b00000000
#define LIS3MDL_SPI_3_WIRE              0b00000100

#define LIS3MDL_CONTINUOUS_CONVERSION   0b00000000
#define LIS3MDL_SINGLE_CONVERSION       0b00000001
#define LIS3MDL_POWER_DOWN              0b00000010

/* LIS3MDL CTRL_REG4 Configuration Map */
#define LIS3MDL_Z_LP                    0b00000000
#define LIS3MDL_Z_MP                    0b00000100
#define LIS3MDL_Z_HP                    0b00001000
#define LIS3MDL_Z_UHP                   0b00001100

#define LIS3MDL_LITTLE_ENDIAN           0b00000000
#define LIS3MDL_BIG_ENDIAN              0b00000010

/* LIS3MDL CTRL_REG5 Configuration Map */
#define LIS3MDL_FAST_READ_EN            0b10000000
#define LIS3MDL_FAST_READ_DIS           0b00000000
#define LIS3MDL_BLOCK_UPDATE_EN         0b01000000
#define LIS3MDL_BLOCK_UPDATE_DIS        0b00000000

/* LIS3MSL DEFINES */

SPI_HandleTypeDef mag_spi = {0};

/**
 * @brief initializes magnetometer
 * This function initializes the magnetometer based on settings in device structure
 * @param device: device pointer to lis3mdl_device struct
 * @returns if the magnetometer is ok
*/

magnetometer_err HALAL_magnetometer_initialize() {
    mag_spi.Instance = HALAL_MAGNETOMETER_SPI;
    mag_spi.Init.Mode = SPI_MODE_MASTER;
    mag_spi.Init.Direction = SPI_DIRECTION_2LINES;
    mag_spi.Init.DataSize = SPI_DATASIZE_4BIT;
    mag_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    mag_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    mag_spi.Init.NSS = SPI_NSS_HARD_OUTPUT;
    mag_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    mag_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    mag_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    mag_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    mag_spi.Init.CRCPolynomial = 0x0;
    HAL_SPI_Init(&mag_spi);

    lis3mdl_write_register(MAG_REG_CTRL3, MAG_CONTINUOUS_CONVERSION);
    uint8_t ctrl_reg_1 = HALAL_MAGNETOMETER_TEMP_ENABLE | HALAL_MAGNETOMETER_DATA_RATE | HALAL_MAGNETOMETER_SELF_TEST;
    lis3mdl_write_register(MAG_REG_CTRL1, ctrl_reg_1);
    uint8_t ctrl_reg_2 = HALAL_MAGNETOMETER_FULL_SCALE;
    lis3mdl_write_register(MAG_REG_CTRL2, ctrl_reg_2);
    uint8_t ctrl_reg_4 = HALAL_MAGNETOMETER_Z_AXIS_MODE | HALAL_MAGNETOMETER_ENDIANNESS;
    lis3mdl_write_register(MAG_REG_CTRL4, ctrl_reg_4);
    return MAG_ERR_OK;
}

/**
 * @brief read magnetic field vector from device
 * This function reads magnetic field from device and converts the value into double precision.
 * This stores the vector into a provided array. Magnetic field is represented in Gauss.
 * @param mag_reading: pointer to 3-element doubly array to store magnetic field reading in Gauss.
 * @warning: no error checking is performed. Make sure to allocate appropriate array for inputs
*/

magnetometer_err HALAL_magnetometer_read_mag(double *mag_reading) {
    uint8_t mag_read_buf[6];
    double sensitivity = 0;
    list3mdl_read_multiple_registers(MAG_REG_OUT_X_L, 6, mag_read_buf);
    int16_t x_reading = (mag_read_buf[1] << 8) | mag_read_buf[0]; 
    int16_t y_reading = (mag_read_buf[3] << 8) | mag_read_buf[1]; 
    int16_t z_reading = (mag_read_buf[5] << 8) | mag_read_buf[2]; 
    HALAL_magnetometer_sensitivity_get(&sensitivity);
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

magnetometer_err HALAL_magnetometer_read_temp(double *temp) {
    uint8_t temp_read_buff[2];
    list3mdl_read_multiple_registers(MAG_REG_TEMP_OUT_L, 2, temp_read_buff);
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

magnetometer_err HALAL_magnetometer_write_hard_iron(double *hard_iron_offset) {
    int16_t hard_iron_ints[3];
    double sensitivity = 0;
    HALAL_magnetometer_sensitivity_get(&sensitivity);
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
magnetometer_err HALAL_magnetometer_sensitivity_get(double *sensitivity) {
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

magnetometer_err lis3mdl_write_register(uint8_t reg, uint8_t data) {
    uint8_t transmit_buf[2] = {reg, data};
    HAL_GPIO_WritePin((GPIO_TypeDef *)HALAL_MAGNETOMETER_CS_PIN_PORT, (uint16_t)HALAL_MAGNETOMETER_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&mag_spi, transmit_buf, 2, HAL_MAX_DELAY);
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
magnetometer_err lis3mdl_read_register(uint8_t reg, uint8_t *data) {
    uint8_t transmit_buf[2] = {0x80 | reg, 0x00};
    uint8_t receive_buf[2];
    HAL_GPIO_WritePin((GPIO_TypeDef *)HALAL_MAGNETOMETER_CS_PIN_PORT, (uint16_t)HALAL_MAGNETOMETER_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&mag_spi, transmit_buf, receive_buf, 2, HAL_MAX_DELAY);
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
magnetometer_err lis3mdl_write_multiple_registers(uint8_t start_reg, uint8_t bytes, uint8_t *data) {
    uint8_t transmit_buf[bytes + 1];
    for (int i = 1; i <= bytes; i ++) {
        transmit_buf[i] = data[i - 1];
    }
    transmit_buf[0] = 0x40 | start_reg;
    HAL_GPIO_WritePin((GPIO_TypeDef *)HALAL_MAGNETOMETER_CS_PIN_PORT, (uint16_t)HALAL_MAGNETOMETER_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&mag_spi, transmit_buf, bytes + 1, HAL_MAX_DELAY);
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

magnetometer_err lis3mdl_read_multiple_registers(uint8_t start_reg, uint8_t bytes, uint8_t *data) {
    // TODO: error handling
    uint8_t transmit_buf[bytes + 1];
    uint8_t receive_buf[bytes + 1];
    for (int i = 1; i <= bytes; i++) {
        transmit_buf[i] = 0x00;
    }
    transmit_buf[0] = 0xC0 | start_reg;
    HAL_GPIO_WritePin((GPIO_TypeDef *)HALAL_MAGNETOMETER_CS_PIN_PORT, (uint16_t)HALAL_MAGNETOMETER_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&mag_spi, transmit_buf, receive_buf, bytes + 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin((GPIO_TypeDef *)HALAL_MAGNETOMETER_CS_PIN_PORT, (uint16_t)HALAL_MAGNETOMETER_CS_PIN, GPIO_PIN_SET);
    for (int i = 0; i < bytes; i++) {
        data[i] = receive_buf[i + 1];
    }
    return MAG_ERR_OK;
}