/**
 * @file Ground.h
 * @author Albert Zheng
 * @brief Header file for ground state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include "ground_ekf.h"
#include "data_handling.h"
#include <stdbool.h>

#ifndef __GROUND_H__
#define __GROUND_H__

void state_transition_ground(GroundExtKalmanFilter *gekf);

void state_transition_jacob_ground(GroundExtKalmanFilter *gekf);

void observation_ground(GroundExtKalmanFilter *gekf);

void observation_jacob_ground(GroundExtKalmanFilter *gekf);

void get_ground_attitude(GroundExtKalmanFilter *gekf);

uint8_t check_gekf_convergence(GroundExtKalmanFilter *gekf, UART_HandleTypeDef *huart);

void run_ground(GroundExtKalmanFilter *gekf, Sensors *sensors, SerialData *serial_data, UART_HandleTypeDef *huart);

void print_P_n(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart);

#endif