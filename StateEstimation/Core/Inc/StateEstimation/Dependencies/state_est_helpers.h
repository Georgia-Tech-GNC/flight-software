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



// State machine enumeration
typedef enum {
    IDLE,
    GROUND,
    ARMED,
    FASTASCENT,
    SLOWASCENT,
    FREEFALL,
    LANDED
} RocketState;


// GPS params
#define WGS84_A 6378137.0   // Semi-major axis of WGS 84 ellipsoid (meters)
#define WGS84_B 6356752.3   // semi-minor axis

#define BURN_TIME 4.5
#define COM_DIST_START 1.0
#define COM_DIST_END 0.2
#define COM_TO_IMU_Y 0.01
#define COM_TO_IMU_Z 0.005

// Function pointer type for state handlers
typedef void (*StateHandler)(void);

// State machine structure
typedef struct {
    RocketState currentState;
    StateHandler stateHandlers[7]; 
} StateMachine;

// External variable declarations
extern uint16_t rocket_state;
extern uint32_t global_time;
extern float32_t fast_ascent_start_time;
extern float32_t global_time_seconds;
extern uint32_t prev_global_time;
extern int16_t first_iter;
extern float32_t startTOV;
extern int16_t activatedTOV;
extern float32_t prev_alt;
extern uint8_t iterations;
extern uint8_t has_not_run_fast_ascent;
extern uint8_t first_time;
extern uint8_t start;
extern uint32_t global_time_step;

// Helper function declarations
float32_t *com_to_imu(float32_t seconds_since_launch, int launch_has_occurred);
float32_t pressure2altitude(float32_t pressure);
void arm_mat_identity_f32(arm_matrix_instance_f32* matrix, uint16_t size, float32_t* data);
void check_for_nan(const char* name, arm_matrix_instance_f32* mat, UART_HandleTypeDef *huart);
void print_matrix(const char* name, arm_matrix_instance_f32* mat, UART_HandleTypeDef *huart);

void handle_idle(void);
void handle_ground(void);
void handle_armed(void);
void handle_fast_ascent(void);
void handle_slow_ascent(void);
void handle_freefall(void);
void handle_landed(void);

// State machine function declarations
void state_machine_init(void);
void state_machine_run(void);
void transition_state(RocketState newState);


// State handler declarations

#endif /* __STATE_EST_HELPERS_H__ */