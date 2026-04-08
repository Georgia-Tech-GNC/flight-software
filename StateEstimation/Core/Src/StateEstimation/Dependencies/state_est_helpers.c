/**
 * @file state_est_helpers.c
 * @author Patrick Barry 
 * @brief Source file for helper functions for the state estimation MCU
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/
#include <stdio.h>
#include "state_est_helpers.h"
#include "arm_math.h"

uint16_t state_machine;
uint32_t global_time;
float32_t fast_ascent_start_time;
float32_t global_time_seconds;
uint32_t prev_global_time;
int16_t first_iter;
int16_t first_slow_ascent_iter;
float32_t startTOV;
int16_t activatedTOV;
float32_t prev_alt;
uint8_t iterations;
uint8_t has_not_run_fast_ascent;
uint8_t first_time;
uint8_t start;
uint32_t global_time_step;
uint8_t ready_message_printed;
uint8_t gekf_initialize;
uint8_t fekf_initialize;

/**
 * @brief Function that linearly interpolates the distance from the IMU to the center of mass so that Coriolis effects may be subtracted
 *  from accelerometer measurements
 * @param seconds_since_launch, the number of seconds since launch
 * @param launch_has_occurred (0 if launch has not occurred, 1 if launch has occurred)
 * @return Returns a 3-element array that represents the vector in the body frame from the center of mass to the IMU
 * @note Requires knowledge of the CoM-->IMU vector at empty and full motor.
*/
float32_t* com_to_imu(float32_t seconds_since_launch, int launch_has_occurred){
    float32_t x_dist;
    if (launch_has_occurred) {
        if (seconds_since_launch >= BURN_TIME) {
            x_dist = COM_DIST_END;
        } else {
            x_dist = (seconds_since_launch / BURN_TIME) * (COM_DIST_START - COM_DIST_END); //linear interpolation
        }
    } else {
        x_dist = COM_DIST_START;
    }
    //float32_t imu_distances[3] = {x_dist, COM_TO_IMU_Y, COM_TO_IMU_Z};
    return (float32_t[3]){x_dist, COM_TO_IMU_Y, COM_TO_IMU_Z};
}

/**
 * @brief This function converts barometric pressure readings to altitude based on an atmospheric model.
 * @param pressure, a pressure reading in ??? units
 * @return altitude, the "pressure altitude." Useful if barometer is utilized
 * @note Based on ??? atmosphere model (consult Albert Zheng for questions)
*/
float32_t pressure2altitude(float32_t pressure) {
    return (float32_t) (44330 * (1.0 - pow( (pressure/100) / 1013.25, 0.1903))); // (sealvlhpa = 1013.25)
}


void arm_mat_identity_f32(arm_matrix_instance_f32* matrix, uint16_t size, float32_t* data) {
    // Allocate memory for the matrix data

    // Initialize the matrix instance
    arm_mat_init_f32(matrix, size, size, data);

    // Set the matrix elements to form an identity matrix
    for (uint16_t i = 0; i < size; i++) {
        for (uint16_t j = 0; j < size; j++) {
            data[i * size + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void state_machine_init(void) {
    state_machine = IDLE;
    global_time = HAL_GetTick();
    fast_ascent_start_time = 0.0f;
    global_time_seconds = (float32_t) global_time / 1000.0f;
    prev_global_time = global_time;
    first_iter = 1;
    startTOV = 0.0f;
    activatedTOV = 0; 
    prev_alt = 0.0f;
    start = HAL_GetTick();
    has_not_run_fast_ascent = 1;
    first_time = 1;
    ready_message_printed = 0;
    gekf_initialize = 1;
    fekf_initialize = 1;
    iterations = 0;
}


void check_for_nan(const char* name, arm_matrix_instance_f32* mat, UART_HandleTypeDef *huart) {
    char buffer[100];
    for (int i = 0; i < mat->numRows * mat->numCols; i++) {
        if (isnan(mat->pData[i]) || isinf(mat->pData[i])) {
            int row = i / mat->numCols;
            int col = i % mat->numCols;
            int len = snprintf(buffer, sizeof(buffer), "NaN or Inf found in %s at [%d][%d]\r\n", name, row, col);
            HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        }
    }
}

// Function to print a matrix
void print_matrix(const char* name, arm_matrix_instance_f32* mat, UART_HandleTypeDef *huart) {
    char buffer[256];
    int len;
    len = snprintf(buffer, sizeof(buffer), "%s (%dx%d):\r\n", name, mat->numRows, mat->numCols);
    HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    for (int i = 0; i < mat->numRows; i++) {
        for (int j = 0; j < mat->numCols; j++) {
            len = snprintf(buffer, sizeof(buffer), "%.4e ", mat->pData[i * mat->numCols + j]);
            HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        }
        HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

