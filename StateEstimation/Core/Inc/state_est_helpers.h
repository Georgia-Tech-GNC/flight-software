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


const float burn_time = 12.0; //motor burn time, in seconds
const float com_dist_start = 1.0; //distance in meters along the rocket's axis from center of mass with full tank to the IMU
const float com_dist_end = 0.2; //distance in meters along the rocket's axis from center of mass with empty tank to the IMU

const float com_to_imu_y = 0.01; //y-distance from center of mass to IMU in meters
const float com_to_imu_z = 0.005; //z-distance from center of mass to IMU in meters

float *com_to_imu(float seconds_since_launch, int launch_has_occurred);
    //launch_has_occurred = 1 if launch has occurred, =0 if launch has not occurred.

float *multiply_matrices(float *pA, int rowsA, int colsA, float *pB, int rowsB, int colsB);
float *transpose_matrix(float *matA);
float *inverse_matrix(float *matA);
float *add_matrices(float *matA, float *matB);
float *subtract_matrices(float *matA, float *matB);




#endif
