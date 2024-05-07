/**
 * @file ekf.h
 * @author Patrick Barry, Ishan Swali
 * @brief This contains the function definitions for the EKF that governs position/velocity
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef __EKF_H__
#define __EKF_H__

const int nx = 6;
const int nz = 3;
const int nu = 0;
typedef struct {

    float time_step;
    //Core variables
    float x_n[nx]; //Curent state x_n,n
    float u; //Control input (nu by 1)
    float G; //Control matrix G (nu by nu)

    float z[nz]; //Measurement z_n (nz by 1)

    float x_prev[nx]; //Previously predicted state x_n,n-1 (nx by 1)
    float P_prev[nx][nx]; //Previously predicted estimate covariance P_n,n-1 (nx by nx)

    float x_next[nx]; //Predicted future state x_n,n-1 (nx by 1)
    float P_next[nx][nx]; //Predicted future estimate covariance P_n,n-1 (nx by nx)

    //Intermediate Variables
    float K_n[nx][nz]; //Kalman gain Kn (nx by nz)

    float f[nx]; //State transition function (nx by 1)
    float dfdx[nx][nx]; //Linearized state transition function (nx by nx)

    float h[nz]; //Observation function (nz by 1)
    float dhdx[nz][nx]; //Linearized observation function (nz by nx)

    float P_n[nx][nx]; //Current estimate covariance P_n,n (nx by nx)

    float Q[nx][nx]; //Process noise covariance Q (nx by nx)

    float R[nz][nz]; //Measurement covariance R (nz by nz)

    float gps[nz]; //GPS reading relative to starting location (x, y, z)
    float accelerometer[nz]; //Accelerometer readings (x, y, z)
    float barometer; //Barometer altitude reading

    int64_t prev_time_millis;
} ekf;

void initialize_ekf(ekf *ekf);

int64_t currentTimeMillis();

void update_time_step(ekf *ekf);

void observation_function(ekf *ekf);

void observation_jacobian(ekf *ekf);

void kalman_gain(ekf *ekf);

void update_state(ekf *ekf);

void update_covariance(ekf *ekf);

void state_transition_function(ekf *ekf);

void state_transition_jacobian(ekf *ekf);

void predict_state(ekf *ekf);

void predict_covariance(ekf *ekf);

void correct_accelerometer_coriolis(ekf *ekf, float *com_to_imu, float wx, float wy, float wz, float wx_dot, float wy_dot, float wz_dot);

void acknowledge_time_passed(ekf *ekf);

float *run_ekf(ekf *ekf, float *com_to_imu, float wx, float wy, float wz, float wx_dot, float wy_dot, float wz_dot, float *GPS_sensor, float *IMU_sensor);

#endif
