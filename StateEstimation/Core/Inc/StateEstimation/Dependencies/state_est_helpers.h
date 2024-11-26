/**
 * @file state_est_helpers.h
 * @author Patrick Barry 
 * @brief This contains the function definitions for helper functions for the state estimation MCU
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/
#ifndef __STATE_EST_HELPERS_H__
#define __STATE_EST_HELPERS_H__


#include "stdbool.h"
#include "arm_math.h"
#include "stm32h7xx_hal.h"

typedef enum {
    IDLE,
    GROUND,
    ARMED,
    FASTASCENT,
    SLOWASCENT,
    FREEFALL,
    LANDED,
    NUM_STATES 
} State;


// GPS params
#define WGS84_A 6378137.0   // Semi-major axis of WGS 84 ellipsoid (meters)
#define WGS84_B 6356752.3 // semi-minor axis

#define BURN_TIME 4.5
#define COM_DIST_START 1.0
#define COM_DIST_END 0.2
#define COM_TO_IMU_Y 0.01
#define COM_TO_IMU_Z 0.005

typedef void (*state_func)(void);

float32_t *com_to_imu(float32_t seconds_since_launch, int launch_has_occurred);

float32_t pressure2altitude (float32_t pressure);

void arm_mat_identity_f32(arm_matrix_instance_f32* matrix, uint16_t size, float32_t* data);

void check_for_nan(const char* name, arm_matrix_instance_f32* mat, UART_HandleTypeDef *huart);

void print_matrix(const char* name, arm_matrix_instance_f32* mat, UART_HandleTypeDef *huart);

void state_machine_init(void);

#endif