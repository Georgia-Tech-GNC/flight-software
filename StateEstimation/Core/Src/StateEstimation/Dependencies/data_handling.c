/**
 * @file data_handling.c
 * @author Albert Zheng, Kanav Chugh
 * @brief Source file for logging, transmitting, and receiving serial data
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/


#include "data_handling.h"



/**
 * @brief Log serial data and sensor readings
 * 
 * This function logs the SerialData and Sensors structures for further analysis or debugging.
 * It packetizes the data and sends it over UART.
 * 
 * @param serial_data Pointer to the SerialData structure containing the serial data.
 * @param sensors Pointer to the Sensors structure containing sensor readings.
 */
void log_data(SerialData *serial_data, Sensors *sensors, UART_HandleTypeDef* huart) {
    uint8_t serial_buffer[81];
    uint8_t sensors_buffer[37];
    memcpy(&sensors_buffer[0], &sensors->start_byte, sizeof(uint8_t));
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
    for (int i = 0; i < 9; i++) {
        memcpy(&sensors_buffer[1 + i * 4], &compensated_values[i], 4);
    }
    int offset = 0;
    memcpy(&serial_buffer[offset], &serial_data->state, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    memcpy(&serial_buffer[offset], &serial_data->pos_x, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->pos_y, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->pos_z, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->vel_x, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->vel_y, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->vel_z, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->q0, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->q1, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->q2, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->q3, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->wx, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->wy, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->wz, sizeof(float32_t));

    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->P_1, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->P_2, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->P_3, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->P_4, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->P_5, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->P_6, sizeof(float32_t));
    offset += sizeof(float32_t);
    memcpy(&serial_buffer[offset], &serial_data->t, sizeof(float32_t));
    HAL_StatusTypeDef result = HAL_UART_Transmit(huart, sensors_buffer, sizeof(sensors_buffer), HAL_MAX_DELAY);
    result = HAL_UART_Transmit(huart, serial_buffer, sizeof(serial_buffer), HAL_MAX_DELAY);
    if (result != HAL_OK) {
        printf("UART Transmission failed!");
    }
}

/**
 * @brief Reads from sensor drivers and applies compensation (bias/offsets)
 * 
 * This function updates the biases and offsets for the sensor readings. It sets the
 * initial values of the sensor readings and applies any necessary compensation.
 * 
 * @param sensors Pointer to the Sensors structure representing sensor readings.
 */
void update_biases(Sensors* sensors) {
    sensors->accel_x = 0;
    sensors->accel_y = 0;
    sensors->accel_z = 0;
    sensors->gyro_x = 0;
    sensors->gyro_y = 0;
    sensors->gyro_z = 0;
    sensors->gps_x = 0;
    sensors->gps_y = 0;
    sensors->gps_z = 0;
}