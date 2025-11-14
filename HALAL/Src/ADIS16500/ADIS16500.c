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


#include "imu.h"
#include "stm32h7xx.h"
#include "arm_math.h"
#include "nucleo_f429zi_halal.h"
#include <stdint.h>
#include <stdbool.h>
#ifdef STM32H7xx_HAL_SPI_H
    #include "stm32h7xx_hal_gpio.h"
#endif
#include "stm32h7xx_hal_spi.h"
#ifdef STM32H7xx_HAL_SPI_H
    #include "stm32f4xx_hal_gpio.h"
#endif


typedef __int32 int32_t;
typedef unsigned __int32 uint32_t;


typedef enum {
    ADIS_DIAG_STAT = 0x02,
    ADIS_X_GYRO_LOW = 0x04,
    ADIS_X_GYRO_OUT = 0x06,
    ADIS_Y_GYRO_LOW = 0x08,
    ADIS_Y_GYRO_OUT = 0x0A,
    ADIS_Z_GYRO_LOW = 0x0C,
    ADIS_Z_GYRO_OUT = 0x0E,
    ADIS_X_ACCL_LOW = 0x10,
    ADIS_X_ACCL_OUT = 0x12,
    ADIS_Y_ACCL_LOW = 0x14,
    ADIS_Y_ACCL_OUT = 0x16,
    ADIS_Z_ACCL_LOW = 0x18,
    ADIS_Z_ACCL_OUT = 0x1A,
    ADIS_TEMP_OUT = 0x1C,
    ADIS_TIME_STAMP = 0x1E,
    ADIS_DATA_CNTR = 0x22,
    ADIS_X_DELTANG_LOW = 0x24,
    ADIS_X_DELTANG_OUT = 0x26,
    ADIS_Y_DELTANG_LOW = 0x28,
    ADIS_Y_DELTANG_OUT = 0x2A,
    ADIS_Z_DELTANG_LOW = 0x2C,
    ADIS_Z_DELTANG_OUT = 0x2E,
    ADIS_X_DELTVEL_LOW = 0x30,
    ADIS_X_DELTVEL_OUT = 0x32,
    ADIS_Y_DELTVEL_LOW = 0x34,
    ADIS_Y_DELTVEL_OUT = 0x36,
    ADIS_Z_DELTVEL_LOW = 0x38,
    ADIS_Z_DELTVEL_OUT = 0x3A,
    ADIS_XG_BIAS_LOW = 0x40,
    ADIS_XG_BIAS_HIGH = 0x42,
    ADIS_YG_BIAS_LOW = 0x44,
    ADIS_YG_BIAS_HIGH = 0x46,
    ADIS_ZG_BIAS_LOW = 0x48,
    ADIS_ZG_BIAS_HIGH = 0x4A,
    ADIS_XA_BIAS_LOW = 0x4C,
    ADIS_XA_BIAS_HIGH = 0x4E,
    ADIS_YA_BIAS_LOW = 0x50,
    ADIS_YA_BIAS_HIGH = 0x52,
    ADIS_ZA_BIAS_LOW = 0x54,
    ADIS_ZA_BIAS_HIGH = 0x56,
    ADIS_FILT_CTRL = 0x5C,
    ADIS_RANG_MDL = 0x5E,
    ADIS_MSC_CTRL = 0x60,
    ADIS_UP_SCALE = 0x62,
    ADIS_DEC_RATE = 0x64,
    ADIS_GLOB_CMD = 0x68,
    ADIS_FIRM_REV = 0x6C,
    ADIS_FIRM_DM = 0x6E,
    ADIS_FIRM_Y = 0x70,
    ADIS_PROD_ID = 0x72,
    ADIS_SERIAL_NUM = 0x74,
    ADIS_USER_SCR_1 = 0x76,
    ADIS_USER_SCR_2 = 0x78,
    ADIS_USER_SCR_3 = 0x7A,
    ADIS_FLSHCNT_LOW = 0x7C,
    ADIS_FLSHCNT_HIGH = 0x7E
} ADIS_RegAddr;

typedef enum Determinants{
    ADIS_GYRO,
    ADIS_ACCEL,
    ADIS_DELTA_ANGLE,
    ADIS_DELTA_VEL,
};

typedef struct {
    uint16_t diag_stat;
    uint16_t data_cntr;
    uint16_t checksum;
    double x_gyro_out;
    double y_gyro_out;
    double z_gyro_out;
    double x_accl_out;
    double y_accl_out;
    double z_accl_out;
    double temp_out;
    double x_delta_ang_out;
    double y_delta_ang_out;
    double z_delta_ang_out;
    double x_delta_vel_out;
    double y_delta_vel_out;
    double z_delta_vel_out;
} ADIS16500_Data;

struct ADIS_BurstData {
    uint16_t diag_stat;      // Diagnostic status
    float32_t gyro[3];       // Gyroscope data (X, Y, Z)
    float32_t accel[3];      // Accelerometer data (X, Y, Z)
    float32_t temp;          // Temperature
    uint16_t timestamp;      // Time stamp
};

struct ADIS_Device { 
    SPI_HandleTypeDef* spi_handle;
    GPIO_TypeDef *cs_pin;
    uint16_t cs_pin_port;
};

struct ADIS_Device device;
struct ADIS_BurstData burstData;
ADIS16500_Data adis_data;
float32_t accel_readings[3];
float32_t gyro_readings[3];


/**
 * @brief reads a register given the memory address
 * @param device instance of ADIS IMU device
 * @param addr address to read data from
 * @return data in bits, MSB first
*/

int16_t imu_read_register(uint8_t addr) {
	delay_us(5);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device.cs_pin, (uint16_t)device.cs_pin_port, GPIO_PIN_RESET);
	uint8_t address[2] = {addr, 0x00};
	HAL_SPI_Transmit(device.spi_handle, address, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device.cs_pin, (uint16_t)device.cs_pin_port, GPIO_PIN_SET);
	delay_us(16);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device.cs_pin, (uint16_t)device.cs_pin_port, GPIO_PIN_RESET);
	uint8_t txbuf[2] = {0x00, 0x00};
	uint8_t rxbuf[2];
	HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)device.spi_handle, txbuf, rxbuf, 2, 150);
	HAL_GPIO_WritePin((GPIO_TypeDef*) device.cs_pin, (uint16_t)device.cs_pin_port, GPIO_PIN_SET);
	delay_us(5);
	return (rxbuf[1] << 8) | (rxbuf[0] & 0xFF);
}

void HALAL_imu_init() {
    device.cs_pin = (GPIO_TypeDef*)ADIS_DEVICE_CS_PIN;
    device.cs_pin_port = ADIS_DEVICE_CS_PIN_PORT ;
    device.spi_handle = (SPI_HandleTypeDef*)device.spi_handle; // Assuming hspi1 is defined and initialized elsewhere

    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = device.cs_pin_port;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_init.Alternate = 0;
  
    HAL_GPIO_Init(device.cs_pin, &gpio_init);

    // Set CS pin high
    HAL_GPIO_WritePin(device.cs_pin, device.cs_pin_port, GPIO_PIN_SET);

    // Initialize DWT for microsecond delay
    DWT_Init();
}

void imu_read_and_transmite() {
    imu_read_specific(DETERMINANT, DATA_SIZE_IN_BITS);
}


/**
 * @brief writes a value to a register, given the value
 * @param device instance of ADIS IMU device
 * @param addr register address to write to
 * @param value value to write to register
*/
void imu_write_register(uint8_t addr, uint16_t value) {
	uint16_t address = (addr | 0x80) << 8;
	uint16_t low_word = (address | (value & 0xFF));
	uint16_t high_word = (address | 0x100) | ((value >> 8) & 0xFF);
	uint8_t upper_word[2] = {high_word >> 8, high_word & 0xFF};
	uint8_t lower_word[2] = {low_word >> 8, low_word & 0xFF};

	/* Writing words to SPI channel */
	HAL_GPIO_WritePin((GPIO_TypeDef*)device.cs_pin, (uint16_t)device.cs_pin_port, GPIO_PIN_RESET);
	HAL_SPI_Transmit((SPI_HandleTypeDef*)device.spi_handle, upper_word, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin((GPIO_TypeDef*)device.cs_pin, (uint16_t).cs_pin_port, GPIO_PIN_SET);
	delay_us(5);

	HAL_GPIO_WritePin((GPIO_TypeDef*)device.cs_pin, (uint16_t)device.cs_pin_port, GPIO_PIN_RESET);
	HAL_SPI_Transmit((SPI_HandleTypeDef*)device.spi_handle, lower_word, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin((GPIO_TypeDef*)device.cs_pin, (uint16_t).cs_pin_port, GPIO_PIN_SET);
	delay_us(5);

}

/**
 * @brief Reads value from specific address from IMU
 * @param device Pointer to the ADIS IMU device instance
 * @param addr Address to read from
 * @return Value at the address
 * @note The gyroscope readings are in degrees per second
 */
void imu_read_specific(enum Determinants determinant, int32_t dataSize) {
    if(determinant == ADIS_GYRO) {
        switch(dataSize) {
            case 8 :
                imu_read_gyro();
                break;
            case 32 :
                imu_read_gyro_32bit();
                imu_read_gyro_32bit_all();
                break;
        }
    } else if(determinant == ADIS_ACCEL) {
        switch(dataSize) {
            case 8 :
                imu_read_accel();
                break;
            case 32 :
                imu_read_accel_32bit();
                imu_read_accel_32bit_all();
                break;
        }
    } else if(determinant == ADIS_DELTA_ANGLE) {
        switch(dataSize) {
            case 8 :
                imu_read_delta_angle();
                break;
            case 32 :
                imu_read_delta_angle_32bit();
                imu_read_delta_angle_32bit_all();
                break;
        }
    } else if(determinant == ADIS_DELTA_VEL) {
        switch(dataSize) {
            case 8 :
                imu_read_delta_vel();
                break;
            case 32 :
                imu_read_delta_vel_32bit();
                imu_read_delta_vel_32bit_all();
                break;
        }
    }
   
}

/**
 * @brief Reads gyroscope data from the ADIS IMU
 * @param device Pointer to the ADIS IMU device instance
 * @param gyro_readings Array to store the gyroscope readings (x, y, z)
 * @note The gyroscope readings are in degrees per second
 */
void imu_read_gyro() {
    adis_data.x_gyro_out = (float32_t)(adis_read_register(ADIS_X_GYRO_OUT)) * 0.1f;
    adis_data.y_gyro_out = (float32_t)(adis_read_register(ADIS_Y_GYRO_OUT)) * 0.1f;
    adis_data.z_gyro_out = (float32_t)(adis_read_register(ADIS_Z_GYRO_OUT)) * 0.1f;
}

/**
 * @brief Reads accelerometer data from the ADIS IMU
 * @param device Pointer to the ADIS IMU device instance
 * @param accel_readings Array to store the accelerometer readings (x, y, z)
 * @note The accelerometer readings are in g-forces
 */
void imu_read_accel() {
    adis_data.x_accl_out = (float32_t) (adis_read_register(ADIS_X_ACCL_OUT)) * 0.01225f;
    adis_data.y_accl_out = (float32_t) (adis_read_register(ADIS_Y_ACCL_OUT)) * 0.01225f;
    adis_data.z_accl_out = (float32_t) (adis_read_register(ADIS_Z_ACCL_OUT)) * 0.01225f;
}


/**
 * @brief Scales accelerometer raw data to g-forces
 * @param raw_data Raw accelerometer data from register
 * @return Scaled accelerometer data in g's
 */
float32_t imu_accel_scale(int16_t raw_data) {
    return (float32_t)raw_data * 0.01225f;  // Using the scale factor from your existing code
}

/**
 * @brief Scales gyroscope raw data to degrees per second
 * @param raw_data Raw gyroscope data from register
 * @return Scaled gyroscope data in degrees/second
 */
float32_t imu_gyro_scale(int16_t raw_data) {
    return (float32_t)raw_data * 0.1f;  // Using the scale factor from your existing code
}

/**
 * @brief Scales temperature data to degrees Celsius
 * @param raw_data Raw temperature data from register
 * @return Scaled temperature in degrees Celsius
 */
float32_t imu_temp_scale(int16_t raw_data) {
    return (float32_t)raw_data * 0.1f;
}

/**
 * @brief Performs burst read of all sensor data
 * @param device Pointer to the ADIS IMU device instance
 * @param burst_data Array to store burst data (should be at least 10 words long)
 * @return Checksum verification status (1 if valid, 0 if invalid)
 */
uint8_t imu_burst_read(uint16_t *burst_data) {
    uint8_t buf[20];
    HAL_GPIO_WritePin((GPIO_TypeDef*)device.cs_pin, (uint16_t)device.cs_pin_port, GPIO_PIN_RESET);
    uint8_t addr[2] = {0x68, 0x00}; 
    HAL_SPI_Transmit((SPI_HandleTypeDef*)device.spi_handle, addr, 2, HAL_MAX_DELAY);
    HAL_SPI_Receive((SPI_HandleTypeDef*)device.spi_handle, buf, 20, HAL_MAX_DELAY);
    HAL_GPIO_WritePin((GPIO_TypeDef*)device.cs_pin, (uint16_t)device.cs_pin_port, GPIO_PIN_SET);  
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
void imu_hardware_reset(GPIO_TypeDef* reset_pin, uint16_t reset_port, uint32_t delay_ms) {
    HAL_GPIO_WritePin(reset_pin, reset_port, GPIO_PIN_RESET);
    HAL_Delay(delay_ms);
    HAL_GPIO_WritePin(reset_pin, reset_port, GPIO_PIN_SET);
    HAL_Delay(delay_ms);  // Wait for device to stabilize
}

/**
 * @brief Parses raw burst data into scaled values
 * @param raw_data Raw burst data array
 * @param burstData Pointer to structure to store parsed data
 */
void imu_parse_burst(uint16_t *raw_data) {
    burstData.diag_stat = raw_data[0];
    burstData.gyro[0] = adis_gyro_scale(raw_data[1]);
    burstData.gyro[1] = adis_gyro_scale(raw_data[2]);
    burstData.gyro[2] = adis_gyro_scale(raw_data[3]);
    burstData.accel[0] = adis_accel_scale(raw_data[4]);
    burstData.accel[1] = adis_accel_scale(raw_data[5]);
    burstData.accel[2] = adis_accel_scale(raw_data[6]);
    burstData.temp = adis_temp_scale(raw_data[7]);
    burstData.timestamp = raw_data[8];
}

/**
 * @brief Reads full 32-bit gyroscope data for a single axis
 * @param device Pointer to the ADIS IMU device instance
 * @param low_reg Address of the lower register (e.g. ADIS_X_GYRO_LOW)
 * @param high_reg Address of the higher register (e.g. ADIS_X_GYRO_OUT)
 * @return 32-bit gyroscope reading
 */
int32_t imu_read_gyro_32bit() {
    int32_t high_word = (int32_t)adis_read_register(HIGH_REG);
    int32_t low_word = (int32_t)adis_read_register(LOW_REG);
    return (high_word << 16) | (low_word & 0xFFFF);
}

/**
 * @brief Reads full 32-bit accelerometer data for a single axis
 * @param device Pointer to the ADIS IMU device instance 
 * @param low_reg Address of the lower register (e.g. ADIS_X_ACCL_LOW)
 * @param high_reg Address of the higher register (e.g. ADIS_X_ACCL_OUT)
 * @return 32-bit accelerometer reading
 */
int32_t imu_read_accel_32bit() {
    int32_t high_word = (int32_t)adis_read_register(HIGH_REG);
    int32_t low_word = (int32_t)adis_read_register(LOW_REG);
    return (high_word << 16) | (low_word & 0xFFFF);
}

/**
 * @brief Reads 32-bit gyroscope data for all axes
 * @param device Pointer to the ADIS IMU device instance
 * @param gyro_readings Array to store the gyroscope readings (x, y, z)
 * @note The gyroscope readings are in degrees per second with full 32-bit precision
 */
void imu_read_gyro_32bit_all() {
    int32_t raw_x = adis_read_gyro_32bit(ADIS_X_GYRO_LOW, ADIS_X_GYRO_OUT);
    int32_t raw_y = adis_read_gyro_32bit(ADIS_Y_GYRO_LOW, ADIS_Y_GYRO_OUT);
    int32_t raw_z = adis_read_gyro_32bit(ADIS_Z_GYRO_LOW, ADIS_Z_GYRO_OUT);
    adis_data.x_gyro_out = (float32_t)raw_x * 0.1f / 65536.0f;  
    adis_data.y_gyro_out = (float32_t)raw_y * 0.1f / 65536.0f;
    adis_data.z_gyro_out = (float32_t)raw_z * 0.1f / 65536.0f;
}

/**
 * @brief Reads 32-bit accelerometer data for all axes
 * @param device Pointer to the ADIS IMU device instance
 * @param accel_readings Array to store the accelerometer readings (x, y, z)
 * @note The accelerometer readings are in g-forces with full 32-bit precision
 */
void imu_read_accel_32bit_all() {
    int32_t raw_x = adis_read_accel_32bit(ADIS_X_ACCL_LOW, ADIS_X_ACCL_OUT);
    int32_t raw_y = adis_read_accel_32bit(ADIS_Y_ACCL_LOW, ADIS_Y_ACCL_OUT);
    int32_t raw_z = adis_read_accel_32bit(ADIS_Z_ACCL_LOW, ADIS_Z_ACCL_OUT);
    adis_data.x_accl_out = (float32_t)raw_x * 0.01225f / 65536.0f; 
    adis_data.y_accl_out = (float32_t)raw_y * 0.01225f / 65536.0f;
    adis_data.z_accl_out = (float32_t)raw_z * 0.01225f / 65536.0f;
}

/**
 * @brief Reads 16-bit delta angle data from the ADIS IMU
 * @param device Pointer to the ADIS IMU device instance
 * @param delta_angle Array to store the delta angle readings (x, y, z)
 * @note Delta angle readings are in degrees
 */
void imu_read_delta_angle() {
    // Read high words only for 16-bit precision
    int16_t x = adis_read_register(ADIS_X_DELTANG_OUT);
    int16_t y = adis_read_register(ADIS_Y_DELTANG_OUT);
    int16_t z = adis_read_register(ADIS_Z_DELTANG_OUT);
    
    // Scale by ΔθMAX/2^15 where ΔθMAX = ±2160°
    adis_data.x_delta_ang_out = (float32_t)x * (2160.0f / 32768.0f);
    adis_data.y_delta_ang_out = (float32_t)y * (2160.0f / 32768.0f);
    adis_data.z_delta_ang_out = (float32_t)z * (2160.0f / 32768.0f);
}

/**
 * @brief Reads 16-bit delta velocity data from the ADIS IMU
 * @param device Pointer to the ADIS IMU device instance
 * @param delta_vel Array to store the delta velocity readings (x, y, z)
 * @note Delta velocity readings are in m/sec
 */
void imu_read_delta_vel() {
    // Read high words only for 16-bit precision
    int16_t x = adis_read_register(ADIS_X_DELTVEL_OUT);
    int16_t y = adis_read_register(ADIS_Y_DELTVEL_OUT);
    int16_t z = adis_read_register(ADIS_Z_DELTVEL_OUT);
    
    // Scale by ±400 m/sec range over 16 bits
    adis_data.x_delta_vel_out = (float32_t)x * (400.0f / 32768.0f);
    adis_data.y_delta_vel_out = (float32_t)y * (400.0f / 32768.0f);
    adis_data.z_delta_vel_out = (float32_t)z * (400.0f / 32768.0f);
}

/**
 * @brief Reads 32-bit delta angle data
 * @param device Pointer to the ADIS IMU device instance
 * @param low_reg Address of the lower register
 * @param high_reg Address of the higher register
 * @return 32-bit delta angle reading
 */
int32_t imu_read_delta_angle_32bit() {
    int32_t high_word = (int32_t)adis_read_register(HIGH_REG);
    int32_t low_word = (int32_t)adis_read_register(LOW_REG);
    return (high_word << 16) | (low_word & 0xFFFF);
}

/**
 * @brief Reads 32-bit delta velocity data
 * @param device Pointer to the ADIS IMU device instance
 * @param low_reg Address of the lower register
 * @param high_reg Address of the higher register
 * @return 32-bit delta velocity reading
 */
int32_t imu_read_delta_vel_32bit() {
    int32_t high_word = (int32_t)adis_read_register(HIGH_REG);
    int32_t low_word = (int32_t)adis_read_register(LOW_REG);
    return (high_word << 16) | (low_word & 0xFFFF);
}

/**
 * @brief Reads full 32-bit precision delta angle data for all axes
 * @param device Pointer to the ADIS IMU device instance
 * @param delta_angle Array to store the delta angle readings (x, y, z)
 * @note Delta angle readings are in degrees with full 32-bit precision
 */
void imu_read_delta_angle_32bit_all() {
    int32_t x = adis_read_delta_angle_32bit(ADIS_X_DELTANG_LOW, ADIS_X_DELTANG_OUT);
    int32_t y = adis_read_delta_angle_32bit(ADIS_Y_DELTANG_LOW, ADIS_Y_DELTANG_OUT);
    int32_t z = adis_read_delta_angle_32bit(ADIS_Z_DELTANG_LOW, ADIS_Z_DELTANG_OUT);

    // Scale by ΔθMAX/2^31 where ΔθMAX = ±2160°
    adis_data.x_delta_ang_out = (float32_t)x * (2160.0f / 2147483648.0f);
    adis_data.y_delta_ang_out = (float32_t)y * (2160.0f / 2147483648.0f);
    adis_data.z_delta_ang_out = (float32_t)z * (2160.0f / 2147483648.0f);
}

/**
 * @brief Reads full 32-bit precision delta velocity data for all axes
 * @param device Pointer to the ADIS IMU device instance
 * @param delta_vel Array to store the delta velocity readings (x, y, z)
 * @note Delta velocity readings are in m/sec with full 32-bit precision
 */
void imu_read_delta_vel_32bit_all() {
    int32_t x = adis_read_delta_vel_32bit(ADIS_X_DELTVEL_LOW, ADIS_X_DELTVEL_OUT);
    int32_t y = adis_read_delta_vel_32bit(ADIS_Y_DELTVEL_LOW, ADIS_Y_DELTVEL_OUT);
    int32_t z = adis_read_delta_vel_32bit(ADIS_Z_DELTVEL_LOW, ADIS_Z_DELTVEL_OUT);
    adis_data.x_delta_vel_out = (float32_t)x * (400.0f / 2147483648.0f);
    adis_data.y_delta_vel_out = (float32_t)y * (400.0f / 2147483648.0f);
    adis_data.z_delta_vel_out = (float32_t)z * (400.0f / 2147483648.0f);
}
