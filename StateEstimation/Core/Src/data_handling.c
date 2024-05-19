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
    SerialData packet;
    packet.data = *serial_data;

    // Transmit packet
    HAL_StatusTypeDef result = HAL_UART_Transmit(&huart2, packet.bytes, sizeof(serial_data));
    
    if (result != HAL_OK) {
        printf("UART Transmission failed!")
    }
}

void log_data(LoggedData *logged_data) {

    //TODO
    
}