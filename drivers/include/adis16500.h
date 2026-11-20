#ifndef ADIS16500_H
#define ADIS16500_H

#include "port.h"
#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"

// ADIS16500 driver, only deals with the IMU: configuring it and turning the raw bytes into numbers
// make an adis_device_t and an adis_sample_t, call adis_init(), then adis_get_sample() from a task

// all the registers are here so it's easy to add more later
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
} adis_regs;

typedef enum {
    ADIS_ERR_DMA_START,     // DMA transfer did not start
    ADIS_ERR_SPI,           // SPI error during a transfer
    ADIS_ERR_QUEUE_FULL,    // finished transfer could not be queued
    ADIS_ERR_CHECKSUM,      // burst checksum did not match
    ADIS_ERR_QUEUE_TIMEOUT, // adis_get_sample waited and got nothing
    ADIS_ERR_DATA_GAP,      // samples missed, counted from data_counter jumps
    ADIS_ERR_COUNT          // number of types above
} adis_err_t;

typedef struct {
    volatile uint32_t counts[ADIS_ERR_COUNT]; // one counter per error type
    volatile bool init_failed;                // fatal
    volatile bool imu_fault;                  // fatal, set from DIAG_STAT
    uint16_t last_data_counter;
    bool have_last_counter;
} adis_status_t;

typedef struct {
    float32_t gyro[3];     // deg/s, bias already subtracted
    float32_t accel[3];    // m/s^2
    float32_t temp;        // deg C
    uint16_t data_counter; // goes up by 1 on every imu measurement
    uint16_t status;       // DIAG_STAT, flag bits get raised for faults
    uint32_t timestamp;    // cpu cycle counter from when the transfer started
} adis_sample_t;

typedef struct {
    uint32_t sample_counter; // samples in the window so far, stops at buffer_size
    float32_t x_bias;
    float32_t y_bias;
    float32_t z_bias;
    float32_t x_buffer[255]; // 255 is the biggest window (buffer_size is a uint8_t)
    float32_t y_buffer[255];
    float32_t z_buffer[255];
    uint8_t write_index;
    uint8_t buffer_size;
    bool mode;
} adis_bias_t;

typedef struct {
    SPI_HandleTypeDef* spi_handle; // &hspi1
    GPIO_TypeDef* cs_pin_port;
    uint16_t cs_pin;
    adis_bias_t bias;
    adis_status_t status;
} adis_device_t;

// sets up the imu (32-bit burst, DR active high), returns false if it fails (wrong PROD_ID, MSC_CTRL write failed)
// takes about 310 ms because the imu needs that long to start up
bool adis_init(adis_device_t* device, SPI_HandleTypeDef* spi_handle, GPIO_TypeDef* cs_pin_port, uint16_t cs_pin);

// called by the generated isr.c
void adis16500_data_ready_callback(void);
void adis16500_spi_tx_rx_cplt_callback(void);
void adis16500_spi_error_callback(void);

// fatal means the data can't be trusted (init failed or DIAG_STAT shows a fault)
bool adis_has_fatal_error(adis_device_t* device);
uint32_t adis_get_error_count(adis_device_t* device, adis_err_t type);

// waits up to 10 ms for a sample, returns false if there wasn't a good one
bool adis_get_sample(adis_device_t* device, adis_sample_t* sample);

// sliding window gyro bias: bias = average of the last N raw gyro readings, subtracted from sample->gyro
// keep the imu still while the window fills, the bias is 0 until N samples are in
// turning it off or changing N resets the bias to 0
void adis_bias_mode(adis_device_t* device, uint8_t buffer_size_N, bool enable);

#endif