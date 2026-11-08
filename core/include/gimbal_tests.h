#ifndef GIMBAL_TESTS_H
#define GIMBAL_TESTS_H

#include "servo.h"
#include "encoder.h"

// FreeRTOS configuration
#define TEST_STACK_SIZE 512
#define TEST_SAMPLE_MS 10 // MS to check the encoder

// Test #1 and #2: Doublet response
#define DOUBLET_HOLD_MS 1500 // MS To hold position
#define DOUBLET_CYCLES 5 // Num of cycles
#define SMALL_STEP_DEG 2.0f 
#define THETA_MAX_DEG 45.0f // Max angle servo (one side)

// Test #3: Staircase response
#define STAIRCASE_STEP_DEG 0.5f
#define STAIRCASE_HOLD_MS 500 // MS to hold poisiton
#define STAIRCASE_MAX_DEG 5.0f // MAX angle of staircase

// Test #4: Sinusoidal frequency sweep
#define SWEEP_DURATION_S 20.0f
#define SWEEP_START_HZ 0.5f
#define SWEEP_END_HZ 15.0f
#define SWEEP_AMPLITUDE_DEG 2.0f

typedef struct {
    servo_t *servo;
    encoder_t *encoder;
} gimbal_test_context_t;

void gimbal_test_1_task(void *params);
void gimbal_test_2_task(void *params);
void gimbal_test_3_task(void *params);
void gimbal_test_4_task(void *params);

#endif
