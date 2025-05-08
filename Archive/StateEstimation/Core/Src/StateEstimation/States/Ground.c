/**
 * @file ground_ekf.c
 * @author Albert Zheng
 * @brief Source file for ground EKF
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "States/Ground.h"
#include "gen_constants.h"

void state_transition_ground(GroundExtKalmanFilter *gekf) {
    // Calculates next state for ground EKF

    float32_t f_new_data[10];

    // accel bias
    f_new_data[0] = gekf->x_n.pData[0];
    f_new_data[1] = gekf->x_n.pData[1];
    f_new_data[2] = gekf->x_n.pData[2];

    // gyro bias
    f_new_data[3] = gekf->x_n.pData[3];
    f_new_data[4] = gekf->x_n.pData[4];
    f_new_data[5] = gekf->x_n.pData[5];

    arm_matrix_instance_f32 f_new = {6, 1, f_new_data};
    gekf->f = f_new;
}

void state_transition_jacob_ground(GroundExtKalmanFilter *gekf) {
    // Calculates state transition jacobian for ground EKF
    float32_t dfdx_new_data[36];
    arm_matrix_instance_f32 dfdx_new;
    arm_mat_identity_f32(&dfdx_new, 6, dfdx_new_data); //TODO: needs updating for later usage
    gekf->dfdx = dfdx_new;
}

void observation_ground(GroundExtKalmanFilter *gekf) {
    // Calculates state-to-measurement relation for ground EKF
    float32_t h_new_data[6];

    h_new_data[0] = -9.81 + gekf->x_n.pData[0];
    h_new_data[1] = gekf->x_n.pData[1],
    h_new_data[2] = gekf->x_n.pData[2],
    h_new_data[3] = gekf->x_n.pData[3];
    h_new_data[4] = gekf->x_n.pData[4];
    h_new_data[5] = gekf->x_n.pData[5];

    arm_matrix_instance_f32 h_new = {6, 1, h_new_data};
    gekf->h = h_new;


}

void observation_jacob_ground(GroundExtKalmanFilter *gekf) {
    float32_t dhdx_new_data[36] = {
        1.0, 0,   0,   0,   0,   0,
        0,   1.0, 0,   0,   0,   0,
        0,   0,   1.0, 0,   0,   0,
        0,   0,   0,   1.0, 0,   0,
        0,   0,   0,   0,   1.0, 0,
        0,   0,   0,   0,   0,   1.0
    };
    arm_matrix_instance_f32 dhdx_new;
    arm_mat_init_f32(&dhdx_new, 6,6, dhdx_new_data); 
    gekf->dhdx = dhdx_new;
}

uint8_t check_gekf_convergence(GroundExtKalmanFilter *gekf, UART_HandleTypeDef *huart) {
    uint8_t converged = 1;
    char debug_buffer[64];
    uint8_t len;
    for (uint8_t i = 0; i < 6; i++) {
        float32_t diag_element = gekf->P_n.pData[i + i * 6];
        if (diag_element > 0.1) {
            converged = 0;
            len = snprintf(debug_buffer, sizeof(debug_buffer), 
                           "State %d not converged: %f\r\n", i, diag_element);
            HAL_UART_Transmit(huart, (uint8_t*)debug_buffer, len, HAL_MAX_DELAY);
        }
    }
    return converged;
}


void print_P_n(GroundExtKalmanFilter *ekf, UART_HandleTypeDef *huart) {
    char buffer[100];
    int len = snprintf(buffer, sizeof(buffer), "P_n matrix:\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    for (int i = 0; i < ekf->nx; i++) {
        len = snprintf(buffer, sizeof(buffer), "[");
        HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        for (int j = 0; j < ekf->nx; j++) {
            len = snprintf(buffer, sizeof(buffer), "%.6f ", ekf->P_n.pData[i * ekf->nx + j]);
            HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        }
        len = snprintf(buffer, sizeof(buffer), "]\r\n");
        HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    }
}


void run_ground(GroundExtKalmanFilter* gekf, Sensors* sensors, SerialData *serial_data, UART_HandleTypeDef *huart) {
    serial_data->state = GROUND;
    serial_data->pos_x = 0.0;
    serial_data->pos_y = 0.0;
    serial_data->pos_z = 0.0;
    serial_data->vel_x = 0.0;
    serial_data->vel_y = 0.0;
    serial_data->vel_z = 0.0;
    serial_data->q0 = 1.0;
    serial_data->q1 = 0.0;
    serial_data->q2 = 0.0;
    serial_data->q3 = 0.0;
    serial_data->wx = 0.0;
    serial_data->wy = 0.0;
    serial_data->wz = 0.0;
    serial_data->P_1 = gekf->P_n.pData[0 + 0 * 6];
    serial_data->P_2 = gekf->P_n.pData[1 + 1 * 6];
    serial_data->P_3 = gekf->P_n.pData[2 + 2 * 6];
    serial_data->P_4 = gekf->P_n.pData[3 + 3 * 6];
    serial_data->P_5 = gekf->P_n.pData[4 + 4 * 6];
    serial_data->P_6 = gekf->P_n.pData[5 + 5 * 6];

    make_measurement_ground(gekf, huart);
    GPS2FlatGround(sensors, gekf, 1);
    observation_ground(gekf);
    //observation_jacob_ground(gekf);
    kalman_gain_ground(gekf, huart);
    update_state_ground(gekf, huart);
    update_covariance_ground(gekf, huart);

    state_transition_ground(gekf);
    //state_transition_jacob_ground(gekf);
    //predict_state(gekf, huart);
    predict_covariance_ground(gekf, huart);
    acknowledge_time_passed_ground(gekf);
    if (check_gekf_convergence(gekf, huart)) {
        sensors->accel_bias_x = gekf->x_n.pData[0];
        sensors->accel_bias_y = gekf->x_n.pData[1];
        sensors->accel_bias_z = gekf->x_n.pData[2];
        sensors->gyro_bias_x = gekf->x_n.pData[3];
        sensors->gyro_bias_y = gekf->x_n.pData[4];
        sensors->gyro_bias_z = gekf->x_n.pData[5];
        rocket_state = ARMED;
    }
}

