/**
 * @file data_handling.c
 * @author Albert Zheng, Kanav Chugh
 * @brief Source file for logging, transmitting, and receiving serial data using DMA
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include "data_handling.h"

static uint8_t serial_buffer_a[81];
static uint8_t serial_buffer_b[81];
static uint8_t sensors_buffer_a[37];
static uint8_t sensors_buffer_b[37];
static volatile bool buffer_a_in_use = false;
static volatile bool transmit_complete = true;

/**
 * @brief Log serial data and sensor readings using DMA
 * @param serial_data Pointer to the SerialData structure containing the serial data
 * @param sensors Pointer to the Sensors structure containing sensor readings
 * @param huart UART handle to send data through
 * @details Uses double buffering and DMA for efficient transmission
 */
void log_data(SerialData *serial_data, Sensors *sensors, UART_HandleTypeDef* huart) {
    // Wait if previous transfer is still in progress
    if (!transmit_complete) {
        return;
    }

    // Select the inactive buffer pair
    uint8_t *current_serial_buffer = buffer_a_in_use ? serial_buffer_b : serial_buffer_a;
    uint8_t *current_sensors_buffer = buffer_a_in_use ? sensors_buffer_b : sensors_buffer_a;
    buffer_a_in_use = !buffer_a_in_use;

    // Prepare sensor data
    float32_t compensated_values[9];
    compensated_values[0] = sensors->accel_x + sensors->accel_bias_x;
    compensated_values[1] = sensors->accel_y - sensors->accel_bias_y;
    compensated_values[2] = sensors->accel_z - sensors->accel_bias_z;
    compensated_values[3] = sensors->gyro_x - sensors->gyro_bias_x;
    compensated_values[4] = sensors->gyro_y - sensors->gyro_bias_y;
    compensated_values[5] = sensors->gyro_z - sensors->gyro_bias_z;
    compensated_values[6] = sensors->gps_x;
    compensated_values[7] = sensors->gps_y;
    compensated_values[8] = sensors->gps_z;

    // Fill sensors buffer
    memcpy(&current_sensors_buffer[0], &sensors->start_byte, sizeof(uint8_t));
    for (int i = 0; i < 9; i++) {
        memcpy(&current_sensors_buffer[1 + i * 4], &compensated_values[i], 4);
    }

    // Fill serial buffer
    int offset = 0;
    memcpy(&current_serial_buffer[offset], &serial_data->state, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    
    // Position data
    memcpy(&current_serial_buffer[offset], &serial_data->pos_x, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->pos_y, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->pos_z, sizeof(float32_t));
    offset += sizeof(float32_t);

    // Velocity data
    memcpy(&current_serial_buffer[offset], &serial_data->vel_x, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->vel_y, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->vel_z, sizeof(float32_t));
    offset += sizeof(float32_t);

    // Quaternion data
    memcpy(&current_serial_buffer[offset], &serial_data->q0, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->q1, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->q2, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->q3, sizeof(float32_t));
    offset += sizeof(float32_t);

    // Angular velocity data
    memcpy(&current_serial_buffer[offset], &serial_data->wx, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->wy, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->wz, sizeof(float32_t));
    offset += sizeof(float32_t);

    // Covariance data
    memcpy(&current_serial_buffer[offset], &serial_data->P_1, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->P_2, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->P_3, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->P_4, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->P_5, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->P_6, sizeof(float32_t));
    //Time stamp
    offset += sizeof(float32_t);
    memcpy(&current_serial_buffer[offset], &serial_data->t, sizeof(float32_t));
    transmit_complete = false;
    HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(huart, current_sensors_buffer, sizeof(sensors_buffer_a));
    if (result == HAL_OK) {
        result = HAL_UART_Transmit_DMA(huart, current_serial_buffer, sizeof(serial_buffer_a));
    } else {
        transmit_complete = true; 
    }
}

/**
 * @brief DMA transfer complete callback
 * @param huart Pointer to UART handle
 * @details Handles buffer switching and triggers next transmission if data is pending
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    transmit_complete = true;
}