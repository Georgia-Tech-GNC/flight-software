/**
 * @file ekf.h
 * @author Patrick Barry, Kanav Chugh
 * @brief This contains the function definitions for the EKF that governs position/velocity
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef __EKF2_H__
#define __EKF2_H__

#include "arm_math.h"
#include "sensors.h"
#include "gen_constants.h"
#include <sys/time.h>
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdio.h>
#include "state_est_helpers.h"

#define MAX_EKF_DIM 6
#define MAX_EKF_MEAS 6 


typedef struct {
    uint16_t nx, nu, nz;
    float32_t time_step;

    arm_matrix_instance_f32 dfdx, G, Q, R, dhdx, K_n;
    arm_matrix_instance_f32 x_prev, x_n, x_next, P_prev, P_n, P_next, f, h, z;
    arm_matrix_instance_f32 temp1, temp2;

    float32_t dfdx_data[MAX_EKF_DIM * MAX_EKF_DIM];
    float32_t G_data[MAX_EKF_DIM * MAX_EKF_DIM];
    float32_t Q_data[MAX_EKF_DIM * MAX_EKF_DIM];
    float32_t R_data[MAX_EKF_MEAS * MAX_EKF_MEAS];
    float32_t dhdx_data[MAX_EKF_MEAS * MAX_EKF_DIM];
    float32_t K_n_data[MAX_EKF_DIM * MAX_EKF_MEAS];
    float32_t x_prev_data[MAX_EKF_DIM];
    float32_t x_n_data[MAX_EKF_DIM];
    float32_t x_next_data[MAX_EKF_DIM];
    float32_t P_prev_data[MAX_EKF_DIM * MAX_EKF_DIM];
    float32_t P_n_data[MAX_EKF_DIM * MAX_EKF_DIM];
    float32_t P_next_data[MAX_EKF_DIM * MAX_EKF_DIM];
    float32_t f_data[MAX_EKF_DIM];
    float32_t h_data[MAX_EKF_MEAS];
    float32_t z_data[MAX_EKF_MEAS];
    float32_t temp1_data[MAX_EKF_DIM * MAX_EKF_DIM];
    float32_t temp2_data[MAX_EKF_DIM * MAX_EKF_DIM];

    float32_t gps[3];
    float32_t gps_origin[3];
    float32_t accel_offset[3];
    float32_t accelerometer[3];
    float32_t gyro[3];
    float32_t magneto[3];
    float32_t gps_flat[3];
    float32_t barometer;
} GroundExtKalmanFilter;



void GPS2FlatGround(Sensors *sensors, GroundExtKalmanFilter *ekf, uint8_t ground);
void initialize_ekf_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart, Sensors *sensors, uint16_t nz);
void observation_function_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void observation_jacobian_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
arm_status kalman_gain_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void update_state_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void update_covariance_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void state_transition_function_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void state_transition_jacobian_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void predict_state_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void predict_covariance_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void make_measurement_ground(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void acknowledge_time_passed_ground(GroundExtKalmanFilter *ekf);
void update_ekf_ground(GroundExtKalmanFilter *ekf, Sensors* sensors);


#endif