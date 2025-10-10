/**
 ******************************************************************************
 * @file    ADIS16500.h
 * @author  Kanav Chugh
 * @brief   ADIS16500 header driver file
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ADIS16500_H
#define _ADIS16500_H

#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include "stm32f4xx_hal_gpio.h"
//#include "DWT.h"

typedef __int32 int32_t;
typedef unsigned __int32 uint32_t;




void DWT_Init(void);
void delay_us(uint32_t microseconds);
int16_t imu_read_register(struct ADIS_Device *device, uint8_t addr);
void imu_write_register(struct ADIS_Device *device, uint8_t addr, uint16_t value);
void imu_read_gyro(struct ADIS_Device *device, float32_t gyro_readings[3]);
void imu_read_accel(struct ADIS_Device *device, float32_t accel_readings[3]);


float32_t imu_accel_scale(int16_t raw_data);
float32_t imu_gyro_scale(int16_t raw_data);
float32_t imu_temp_scale(int16_t raw_data);
uint8_t imu_burst_read(struct ADIS_Device *device, uint16_t *burst_data);
void imu_parse_burst(uint16_t *raw_data, struct ADIS_BurstData *parsed_data);
void imu_hardware_reset(struct ADIS_Device *device, GPIO_TypeDef* reset_pin, uint16_t reset_port, uint32_t delay_ms);
int32_t imu_read_gyro_32bit(struct ADIS_Device *device, uint8_t low_reg, uint8_t high_reg);
int32_t imu_read_accel_32bit(struct ADIS_Device *device, uint8_t low_reg, uint8_t high_reg);
void imu_read_gyro_32bit_all(struct ADIS_Device *device, float32_t gyro_readings[3]);
void imu_read_accel_32bit_all(struct ADIS_Device *device, float32_t accel_readings[3]);
void imu_read_delta_angle(struct ADIS_Device *device, float32_t delta_angle[3]);
void imu_read_delta_vel(struct ADIS_Device *device, float32_t delta_vel[3]);
int32_t imu_read_delta_angle_32bit(struct ADIS_Device *device, uint8_t low_reg, uint8_t high_reg);
int32_t imu_read_delta_vel_32bit(struct ADIS_Device *device, uint8_t low_reg, uint8_t high_reg);
void imu_read_delta_angle_32bit_all(struct ADIS_Device *device, float32_t delta_angle[3]);
void imu_read_delta_vel_32bit_all(struct ADIS_Device *device, float32_t delta_vel[3]);
void imu_read_specific(enum Determinants determinant, struct ADIS_Device *device, float32_t readings[3], uint8_t low_reg, uint8_t high_reg);
#endif /* _ADIS16500_H */;