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

float *com_to_imu(float seconds_since_launch, int launch_has_occurred){

    float x_dist;

    if (launch_has_occurred == 1){
        if (seconds_since_launch >= burn_time){
            x_dist = com_dist_end;
        }
        else{
            x_dist = (seconds_since_launch/burn_time)*(com_dist_start - com_dist_end); //linear interpolation
        }
    }
    else{
        x_dist = com_dist_start;
    }

    float imu_distances[3] = {x_dist, com_to_imu_y, com_to_imu_z};
    return imu_distances;

}

float *run_fast_ascent(ExtKalmanFilter *ekf, rocket_attitude *rocket_atd, float *GPS_data, float *accel_data, float *gyro_data){

    

    float wx = gyro_data[0];
    float wy = gyro_data[1];
    float wz = gyro_data[2];

    run_attitude_estimation(&rocket_atd, wx, wy, wz);
    run_ekf(&ekf, GPS_data, accel_data);

    float phi = rocket_atd->phi;
    float theta = rocket_atd->theta;
    float psi = rocket_atd->psi;

    float x = ekf->x_n.pData[0];
    float vx = ekf->x_n.pData[1];
    float y = ekf->x_n.pData[2];
    float vy = ekf->x_n.pData[3];
    float z = ekf->x_n.pData[4];
    float vz = ekf->x_n.pData[5];

    float state_vector[9] = {x, vx, y, vy, z, vz, phi, theta, psi};
    return state_vector;

}