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
    uint8_t serial_buffer[sizeof(SerialData)];
    uint8_t sensors_buffer[sizeof(Sensors)];
    memcpy(serial_buffer, serial_data, sizeof(SerialData));
    memcpy(sensors_buffer, sensors, sizeof(Sensors));
    HAL_StatusTypeDef result = HAL_UART_Transmit(huart, serial_buffer, sizeof(serial_buffer), HAL_MAX_DELAY);
    result = HAL_UART_Transmit(huart, sensors_buffer, sizeof(sensors_buffer), HAL_MAX_DELAY);
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