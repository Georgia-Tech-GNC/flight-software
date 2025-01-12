#ifndef __SENSORS_H__
#define __SENSORS_H__

#include "stm32h7xx_hal.h"
#include "ADIS16500.h"
#include "gps.h"
#include "MS5607.h"
#include "LIS3MDL.h"
#include "ring_buffer.h"

#include "spi.h"
#include "uart.h"
#include "i2c.h"
#include "system.h"
#include "DWT.h"

extern struct ADIS_Device imu_device;
extern struct lis3mdl_device mag_device;
extern MS5607StateTypeDef ms5607_state;

extern struct ublox_gnss_device gps;
__attribute__((section(".buffer"))) extern uint8_t uart4_rx_dma_buffer[1024];
extern uint16_t uart4_rx_dma_buffer_size;
extern struct ring_buffer uart4_rx_rb;
extern struct ring_buffer usart3_rx_rb;
extern uint8_t uart4_rx_rb_data[512];
extern struct ublox_gnss_cfg_val cfg[10];

// Sensor readings
typedef struct {
  uint8_t start_byte;
  float32_t accel_x;
  float32_t accel_y;
  float32_t accel_z;
  float32_t gyro_x;
  float32_t gyro_y;
  float32_t gyro_z;
  float32_t gps_x;
  float32_t gps_y;
  float32_t gps_z;
  float32_t accel_bias_x;
  float32_t accel_bias_y;
  float32_t accel_bias_z;
  float32_t gyro_bias_x;
  float32_t gyro_bias_y;
  float32_t gyro_bias_z;
  float32_t gps_offset_x; 
  float32_t gps_offset_y;
  float32_t gps_offset_z;
} Sensors;


void protocol_init(void);
void update_sensors(Sensors *sensors, UART_HandleTypeDef *huart);
void sensors_init(Sensors *sensors);


#endif