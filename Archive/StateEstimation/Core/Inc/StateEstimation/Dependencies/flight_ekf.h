/**
 * @file ekf.h
 * @author Patrick Barry, Kanav Chugh
 * @brief This contains the function definitions for the EKF that governs position/velocity
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef __FLIGHT_EKF_H__
#define __FLIGHT_EKF_H__

#include "arm_math.h"
#include "sensors.h"
#include "gen_constants.h"
#include "attitude.h"
#include <sys/time.h>
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdio.h>
#include "state_est_helpers.h"

#define MAX_EKF_DIM 6
#define MAX_FLIGHT_MEAS 3


typedef struct {
    uint16_t nx, nu, nz;
    float32_t time_step;

    arm_matrix_instance_f32 dfdx, G, Q, R, dhdx, K_n;
    arm_matrix_instance_f32 x_prev, x_n, x_next, P_prev, P_n, P_next, f, h, z;
    arm_matrix_instance_f32 temp1, temp2;

    float32_t dfdx_data[MAX_EKF_DIM * MAX_EKF_DIM];
    //float32_t G_data[MAX_EKF_DIM * MAX_EKF_DIM];
    float32_t Q_data[MAX_EKF_DIM * MAX_EKF_DIM];
    float32_t R_data[MAX_FLIGHT_MEAS * MAX_FLIGHT_MEAS];
    float32_t dhdx_data[MAX_FLIGHT_MEAS * MAX_EKF_DIM];
    float32_t K_n_data[MAX_EKF_DIM * MAX_FLIGHT_MEAS];
    //float32_t x_prev_data[MAX_EKF_DIM];
    float32_t x_n_data[MAX_EKF_DIM];
    //float32_t x_next_data[MAX_EKF_DIM];
    //float32_t P_prev_data[MAX_EKF_DIM * MAX_EKF_DIM];
    float32_t P_n_data[MAX_EKF_DIM * MAX_EKF_DIM];
    //float32_t P_next_data[MAX_EKF_DIM * MAX_EKF_DIM];
    float32_t f_data[MAX_EKF_DIM];
    float32_t h_data[MAX_FLIGHT_MEAS];
    float32_t z_data[MAX_FLIGHT_MEAS];
    float32_t temp1_data[MAX_EKF_DIM * MAX_EKF_DIM];
    float32_t temp2_data[MAX_EKF_DIM * MAX_EKF_DIM];

    float32_t c[3];

    float32_t gps[3];
    float32_t gps_origin[3];
    float32_t accel_offset[3];
    float32_t accelerometer[3];
    float32_t gyro[3];
    float32_t magneto[3];
    float32_t gps_flat[3];
    float32_t launch_gps[3];
    float32_t launch_accel[3]; 
    float32_t launch_gyro[3];
    float32_t barometer;
} ExtKalmanFilter;

void GPS2Flat(Sensors *sensors, ExtKalmanFilter *ekf, uint8_t ground);
void initialize_ekf(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart, Sensors *sensors, uint16_t nz);
void print_matrix(const char* name, arm_matrix_instance_f32* mat, UART_HandleTypeDef *huart);
void observation_function(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void observation_jacobian(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
arm_status kalman_gain(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void update_state(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void update_covariance(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void state_transition_function(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, UART_HandleTypeDef *huart);
void state_transition_jacobian(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, UART_HandleTypeDef *huart);
void predict_state(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void predict_covariance(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
void make_measurement(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart);
//void acknowledge_time_passed(ExtKalmanFilter *ekf);
void run_ekf(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, Sensors *sensors, UART_HandleTypeDef *huart, int ekf_initialized);
void update_ekf(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, Sensors* sensors);
void predict_step(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, UART_HandleTypeDef *huart);
void update_step(ExtKalmanFilter *ekf, UART_HandleTypeDef *huart);

#endif