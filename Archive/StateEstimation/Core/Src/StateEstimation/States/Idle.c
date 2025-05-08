/**
 * @file Idle.c
 * @author Albert Zheng
 * @brief Header file for idle state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include "States/Idle.h"
#include "stm32h7xx_hal.h"
#include "gen_constants.h"

void run_idle(UART_HandleTypeDef *huart) {
    static char signal_received[3];
    static uint8_t receive_index = 0;
    uint8_t temp;
    if (HAL_UART_Receive(huart, &temp, 1, 10) == HAL_OK) {
        if (receive_index < 2) {
            signal_received[receive_index++] = temp;
        }
        HAL_UART_Transmit(huart, &temp, 1, HAL_MAX_DELAY);
        if (receive_index == 2) {
            signal_received[2] = '\0';
            if (strcmp(signal_received, "GO") == 0) {
                HAL_UART_Transmit(huart, "\r\nStarting EKF...\r\n", sizeof("\r\nStarting EKF...\r\n") - 1, HAL_MAX_DELAY);
                rocket_state = GROUND;
                ready_message_printed = 0; 
            }
            receive_index = 0;  
        }
    }
}

