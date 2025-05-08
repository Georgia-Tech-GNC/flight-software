/**
 * @file state_est_helpers.c
 * @author Kanav Chugh, Patrick Barry 
 * @brief Source file for helper functions for the state estimation MCU
 * 
 * Copyright 2025 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/
#include <stdio.h>
#include "state_est_helpers.h"
#include "arm_math.h"
#include "main.h"

// Global variables
uint16_t rocket_state;
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

SerialData serial_data;
Sensors sensors;
GroundExtKalmanFilter gekf;
ExtKalmanFilter fekf;
RocketAttitude rocket_atd;
uint8_t signal_received[2];
float32_t launch_time_stamp;
uint8_t launched;

static StateMachine state_machine;

/**
 * @brief Calculates center of mass to IMU vector
 * @param seconds_since_launch Time since launch in seconds
 * @param launch_has_occurred Flag indicating if launch has occurred
 * @return Pointer to 3-element array containing COM to IMU vector [x, y, z]
 */
float32_t* com_to_imu(float32_t seconds_since_launch, int launch_has_occurred) {
    float32_t x_dist;
    if (launch_has_occurred) {
        if (seconds_since_launch >= BURN_TIME) {
            x_dist = COM_DIST_END;
        } else {
            x_dist = (seconds_since_launch / BURN_TIME) * (COM_DIST_START - COM_DIST_END);
        }
    } else {
        x_dist = COM_DIST_START;
    }
    return (float32_t[3]){x_dist, COM_TO_IMU_Y, COM_TO_IMU_Z};
}

/**
 * @brief Converts pressure to altitude using barometric formula
 * @param pressure Pressure reading in mbar
 * @return Calculated altitude in meters
 */
float32_t pressure2altitude(float32_t pressure) {
    return (float32_t) (44330 * (1.0 - pow((pressure/100) / 1013.25, 0.1903)));
}

/**
 * @brief Initializes a square identity matrix
 * @param matrix Pointer to the matrix instance to initialize
 * @param size Dimension of the square matrix
 * @param data Pointer to the data array to store matrix values
 */
void arm_mat_identity_f32(arm_matrix_instance_f32* matrix, uint16_t size, float32_t* data) {
    arm_mat_init_f32(matrix, size, size, data);
    for (uint16_t i = 0; i < size; i++) {
        for (uint16_t j = 0; j < size; j++) {
            data[i * size + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

/**
 * @brief Checks matrix for NaN or Inf values and prints warnings
 * @param name Name of the matrix for identification in output
 * @param mat Pointer to the matrix to check
 * @param huart Pointer to UART handle for debug output
 */
void check_for_nan(const char* name, arm_matrix_instance_f32* mat, UART_HandleTypeDef *huart) {
    char buffer[100];
    for (int i = 0; i < mat->numRows * mat->numCols; i++) {
        if (isnan(mat->pData[i]) || isinf(mat->pData[i])) {
            int row = i / mat->numCols;
            int col = i % mat->numCols;
            int len = snprintf(buffer, sizeof(buffer), "NaN or Inf found in %s at [%d][%d]\r\n", 
                             name, row, col);
            HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        }
    }
}

/**
 * @brief Prints matrix contents to UART for debugging
 * @param name Name of the matrix for identification in output
 * @param mat Pointer to the matrix to print
 * @param huart Pointer to UART handle for debug output
 */
void print_matrix(const char* name, arm_matrix_instance_f32* mat, UART_HandleTypeDef *huart) {
    char buffer[256];
    int len;
    len = snprintf(buffer, sizeof(buffer), "%s (%dx%d):\r\n", name, mat->numRows, mat->numCols);
    HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    
    for (int i = 0; i < mat->numRows; i++) {
        for (int j = 0; j < mat->numCols; j++) {
            len = snprintf(buffer, sizeof(buffer), "%.4e ", 
                         mat->pData[i * mat->numCols + j]);
            HAL_UART_Transmit(huart, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        }
        HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

/**
 * @brief Initialize state machine and related subsystems
 * @details Initializes state handlers, variables, EKF systems, and UART communications
 */
void state_machine_init(void) {
    state_machine.stateHandlers[IDLE] = handle_idle;
    state_machine.stateHandlers[GROUND] = handle_ground;
    state_machine.stateHandlers[ARMED] = handle_armed;
    state_machine.stateHandlers[FASTASCENT] = handle_fast_ascent;
    state_machine.stateHandlers[SLOWASCENT] = handle_slow_ascent;
    state_machine.stateHandlers[FREEFALL] = handle_freefall;
    state_machine.stateHandlers[LANDED] = handle_landed;
    
    state_machine.currentState = IDLE;
    rocket_state = IDLE;
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

    uint8_t tmp;
    launched = 1;
    while(HAL_TIMEOUT != HAL_UART_Receive(&huart2, &tmp, 1, 10));
    HAL_UART_Receive_IT(&huart2, signal_received, 2);
    initialize_ekf(&fekf, &huart3, &sensors, 3);  
    initialize_ekf_ground(&gekf, &huart3, &sensors, 6); 
    HAL_Delay(500);
}

/**
 * @brief Main state machine execution function
 * @details Updates sensors, runs current state handler, updates timing, and logs data
 */
void state_machine_run(void) {
    update_sensors(&sensors, &huart3);
    state_machine.currentState = rocket_state;
    if (state_machine.stateHandlers[state_machine.currentState] != NULL) {
        state_machine.stateHandlers[state_machine.currentState]();
    }
    global_time = HAL_GetTick();
    global_time_seconds = global_time / 1000.0f;
    if (rocket_state > ARMED) {
      serial_data.t = (global_time - launch_time_stamp) / 1000.0f;
    }
    log_data(&serial_data, &sensors, &huart2);
}

/**
 * @brief Transitions state machine to a new state
 * @param newState The state to transition to
 */
void transition_state(RocketState newState) {
    state_machine.currentState = newState;
    rocket_state = newState; 
}

/**
 * @brief Handle IDLE state operations
 * @details Displays ready message, processes GPS and sensor data
 */
void handle_idle(void) {
    if (!ready_message_printed) {
        HAL_UART_Transmit(&huart3, "Ready to run EKF. Type 'GO' to start.\r\n", 
                          sizeof("Ready to run EKF. Type 'GO' to start.\r\n"), HAL_MAX_DELAY);
        ready_message_printed = 1;
    }
    
    char debug[128];
    int debug_len = sprintf(debug, "GPS: lat=%f, lon=%f, height=%f\r\n", 
                      sensors.gps_x, sensors.gps_y, sensors.gps_z);
    
    GPS2FlatGround(&sensors, &gekf, 1);
    debug_len = sprintf(debug, "FLAT: x=%f, y=%f, z=%f\r\n",
                       gekf.gps_flat[0], gekf.gps_flat[1], gekf.gps_flat[2]);
    
    debug_len = sprintf(debug, "ACCEL: x=%f, y=%f, z=%f\r\n",
                       sensors.accel_x, sensors.accel_y, sensors.accel_z);
    
    debug_len = sprintf(debug, "GYRO: x=%f, y=%f, z=%f\r\n",
                       sensors.gyro_x, sensors.gyro_y, sensors.gyro_z);
    
    run_idle(&huart3);
}

/**
 * @brief Handle GROUND state operations
 * @details Initializes and updates ground EKF, runs ground operations
 */
void handle_ground(void) {
    if (gekf_initialize) {
        initialize_ekf_ground(&gekf, &huart3, &sensors, 6);
        gekf_initialize = 0;
    }
    run_ground(&gekf, &sensors, &serial_data, &huart3);
}

/**
 * @brief Handles operations in ARMED state
 * @details Initializes flight EKF and rocket attitude, monitors for launch conditions
 */
void handle_armed(void) {
    if (fekf_initialize) {
        initialize_ekf(&fekf, &huart3, &sensors, 3);
        initialize_rocket_attitude(&rocket_atd, 1, 0, 0, 0); 
        fekf_initialize = 0;
    }
    
    GPS2Flat(&sensors, &fekf, 0);
    fekf.launch_gps[0] = fekf.gps_flat[0];
    fekf.launch_gps[1] = fekf.gps_flat[1];
    fekf.launch_gps[2] = fekf.gps_flat[2];
    char debug[128];
    int debug_len = sprintf(debug, "GPS: lat=%f, lon=%f, height=%f\r\n", 
                      sensors.gps_x, sensors.gps_y, sensors.gps_z);
    HAL_UART_Transmit(&huart3, debug, debug_len, HAL_MAX_DELAY);
    debug_len = sprintf(debug, "FLAT: x=%f, y=%f, z=%f\r\n",
                       fekf.gps_flat[0], fekf.gps_flat[1], fekf.gps_flat[2]);
    HAL_UART_Transmit(&huart3, debug, debug_len, HAL_MAX_DELAY);
    debug_len = sprintf(debug, "ACCEL: x=%f, y=%f, z=%f\r\n",
                       sensors.accel_x, sensors.accel_y, sensors.accel_z);
    HAL_UART_Transmit(&huart3, debug, debug_len, HAL_MAX_DELAY);
    debug_len = sprintf(debug, "GYRO: x=%f, y=%f, z=%f\r\n",
                       sensors.gyro_x, sensors.gyro_y, sensors.gyro_z);
    
    if (fekf.accelerometer[0] > 4.9) {
        char debug_buffer[256];
        int len = snprintf(debug_buffer, sizeof(debug_buffer), "Transitioning to FASTASCENT\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)debug_buffer, len, HAL_MAX_DELAY);
        transition_state(FASTASCENT);
    }
}

/**
 * @brief Handles operations in FASTASCENT state
 * @details Processes initial launch phase and fast ascent calculations
 */
void handle_fast_ascent(void) {
    if (launched) {
        launch_time_stamp = HAL_GetTick();
        launched = 0;
    }
    run_fast_ascent(&fekf, &rocket_atd, &sensors, &serial_data, &huart3);
}

/**
 * @brief Handles operations in SLOWASCENT state
 * @details Processes slow ascent phase calculations and monitoring
 */
void handle_slow_ascent(void) {
    HAL_UART_Transmit(&huart3, "Slow Ascent\r\n", 
            sizeof("Slow Ascent\r\n"), HAL_MAX_DELAY);
    run_slow_ascent(&fekf, &rocket_atd, &sensors, &serial_data, &huart3);
}

/**
 * @brief Handles operations in FREEFALL state
 * @details Processes freefall phase calculations and monitoring
 */
void handle_freefall(void) {
    HAL_UART_Transmit(&huart3, "Freefall\r\n", 
            sizeof("Freefall\r\n"), HAL_MAX_DELAY);
    run_freefall(&fekf, &rocket_atd, &sensors, &serial_data, &huart3);
}

/**
 * @brief Handles operations in LANDED state
 * @details Sets final state parameters and indicates landing completion
 */
void handle_landed(void) {
    serial_data.state = LANDED;
    serial_data.vel_x = 0.0;
    serial_data.vel_y = 0.0;
    serial_data.vel_z = 0.0;
    HAL_UART_Transmit(&huart3, "Landed\r\n", 
            sizeof("Landed\r\n"), HAL_MAX_DELAY);
}

