/**
 * @file data_handling.c
 * @author Albert Zheng
 * @brief Source file for logging, transmitting, and receiving serial data
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/


#include "../Inc/data_handling.h"

void send_serial_data(SerialData *serial_data) {

    // ensure UART is properly initialized
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        printf("UART initialization failed!");
    }

    // Create serial data packet
    SerialDataPacket packet;
    packet.data = *serial_data;

    // Transmit packet
    HAL_StatusTypeDef result = HAL_UART_Transmit(&huart2, packet.bytes, sizeof(serial_data));
    
    if (result != HAL_OK) {
        printf("UART Transmission failed!")
    }
}

void log_data(SerialData *serial_data, Sensors *sensors) {

    // ensure UART is properly initialized
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        printf("UART initialization failed!");
    }

    // Create serial data packet
    SerialDataPacket packet;
    packet.data = *serial_data;

    // Transmit packet
    HAL_StatusTypeDef result = HAL_UART_Transmit(&huart2, packet.bytes, sizeof(serial_data));
    HAL_StatusTypeDef result = HAL_UART_Transmit(&huart2, packet.bytes, sizeof(sensors));
    
    if (result != HAL_OK) {
        printf("UART Transmission failed!")
    }
    
}

/**
 * @brief Reads from sensor drivers and applies compensation (bias/offsets)
*/
void read_compensated_sensors(Sensors* sensors) {

  // read drivers
  sensors->accelerometer_x = //TODO
  sensors->accelerometer_y = //TODO
  sensors->accelerometer_z = //TODO
  sensors->gyro_x = //TODO
  sensors->gyro_y = //TODO
  sensors->gyro_z = //TODO
  sensors->gps_x = //TODO
  sensors->gps_y = //TODO
  sensors->gps_z = //TODO
  sensors->baro = //TODO
  sensors->magneto_x = //TODO
  sensors->magneto_y = //TODO
  sensors->magneto_z = //TODO

}