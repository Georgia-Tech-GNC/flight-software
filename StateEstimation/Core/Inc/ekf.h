/**
 * @file ekf.h
 * @author Patrick Barry
 * @brief This contains the function definitions for the EKF that governs position/velocity
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef __EKF2_H__
#define __EKF2_H__

// #include "arm_math.h"
#include "../../Drivers/CMSIS/DSP/Include/arm_math.h"
#include <sys/time.h>


typedef struct ExtKalmanFilter {
    uint16_t nx;
    uint16_t nu;
    uint16_t nz;

    arm_matrix_instance_f32 x_n; //current state x_n,n (nx by 1)
    arm_matrix_instance_f32 u; //control input (nu by 1)

    arm_matrix_instance_f32 G; //control matrix G (nu by nu)

    arm_matrix_instance_f32 z; //measurement z_n (nz by 1)
    arm_matrix_instance_f32 f;
    arm_matrix_instance_f32 h;

    arm_matrix_instance_f32 x_prev; //previously predicted state x_n,n-1 (nx by 1)
    arm_matrix_instance_f32 P_prev; //previously predicted estimate covariance P_n,n-1 (nx by nx)

    arm_matrix_instance_f32 x_next; //predicted future state x_n,n+1 (nx by 1)
    arm_matrix_instance_f32 P_next; //predicted future estimate covariance P_n,n+1 (nx by nx)

    arm_matrix_instance_f32 K_n; //Kalman gain K_n (nx by nz)

    arm_matrix_instance_f32 dfdx; //Linearized state transition function (nx by nx)
    arm_matrix_instance_f32 dhdx; //Linearized observation function (nz by nx)

    arm_matrix_instance_f32 P_n; //estimate covariance P_n,n (nx by nx)
    arm_matrix_instance_f32 Q; //process noise covariance Q (nx by nx)

    arm_matrix_instance_f32 R; //measurement covariance R (nz by nz)

    float32_t gps[3]; //GPS position reading relative to starting location (x, y, z)
    float32_t accelerometer[3]; //Accelerometer readings (x, y, z)
    float32_t gyro[3];
    float32_t magneto[3];
    float32_t barometer; //barometer altitude reading

    float32_t time_step; //time in seconds
    int64_t prev_time_millis; //previous milliseconds

} ExtKalmanFilter;

arm_status initialize_ekf(ExtKalmanFilter *ekf, uint16_t num_states, uint16_t num_inputs, uint16_t num_measurements, 
float32_t *dfdx_f32, float32_t *dhdx_f32, float32_t *G_f32, float32_t *Q_f32, float32_t *K_f32, float32_t *R_f32,
float32_t *x_prev, float32_t *P_prev, float32_t *x_init, float32_t *P_init, float32_t *x_next, float32_t *P_next,
float32_t *f_f32, float32_t *h_f32, float32_t *z_f32, float32_t *state_stddevs);

int64_t currentTimeMillis();
void update_time_step(ExtKalmanFilter *ekf);
void observation_function(ExtKalmanFilter *ekf);
void observation_jacobian(ExtKalmanFilter *ekf);
void kalman_gain(ExtKalmanFilter *ekf);
void update_state(ExtKalmanFilter *ekf);
void update_covariance(ExtKalmanFilter *ekf);
void state_transition_function(ExtKalmanFilter *ekf);
void state_transition_jacobian(ExtKalmanFilter *ekf);
void predict_state(ExtKalmanFilter *ekf);
void predict_covariance(ExtKalmanFilter *ekf);
void make_measurement(ExtKalmanFilter *ekf);
void acknowledge_time_passed(ExtKalmanFilter *ekf);
void run_ekf(ExtKalmanFilter *ekf, float *GPS_sensor, float *IMU_sensor);

#endif