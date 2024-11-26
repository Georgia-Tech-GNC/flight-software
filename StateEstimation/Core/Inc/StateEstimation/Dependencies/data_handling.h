/**
 * @file data_handling.h
 * @author Albert Zheng, Kanav Chugh
 * @brief Header file for logging, transmitting, and receiving serial data
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef DATA_HANDLING_H
#define DATA_HANDLING_H

#include "sensors.h"
#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include <string.h>
#include <stdio.h>

// Data packet to send to Main MCU
typedef struct {
  uint8_t state; // current state machine state
  float32_t pos_x; // ecef frame
  float32_t pos_y;
  float32_t pos_z;
  float32_t vel_x; // ecef frame
  float32_t vel_y;
  float32_t vel_z;
  float32_t q0; // world to body
  float32_t q1;
  float32_t q2;
  float32_t q3;
  float32_t wx; // body frame
  float32_t wy;
  float32_t wz;
  float32_t P_1; // body frame
  float32_t P_2;
  float32_t P_3;
  float32_t P_4; // body frame
  float32_t P_5;
  float32_t P_6;
  float32_t t;
} SerialData;


//void send_data(SerialData* serial_data, UART_HandleTypeDef* huart);

//void receive_data(SerialData* serial_data, UART_HandleTypeDef* huart);

void log_data(SerialData* serial_data, Sensors* sensors, UART_HandleTypeDef* huart);

void update_biases(Sensors* sensors);

#endif