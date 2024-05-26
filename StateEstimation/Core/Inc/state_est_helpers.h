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

#include "ekf.h";
#include "attitude.h";

// GPS params
#define WGS84_A 6378137.0   // Semi-major axis of WGS 84 ellipsoid (meters)
#define WGS84_B 6356752.3 // semi-minor axis

const int state_vec_length = 9;

const float burn_time = 12.0; //motor burn time, in seconds
const float com_dist_start = 1.0; //distance in meters along the rocket's axis from center of mass with full tank to the IMU
const float com_dist_end = 0.2; //distance in meters along the rocket's axis from center of mass with empty tank to the IMU

const float com_to_imu_y = 0.01; //y-distance from center of mass to IMU in meters
const float com_to_imu_z = 0.005; //z-distance from center of mass to IMU in meters

typedef struct SensorComps {

    float accelo_bias_x;
    float accelo_bias_y;
    float accelo_bias_z;
    float accelo_bias_rate_x;
    float accelo_bias_rate_y;
    float accelo_bias_rate_z;
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
    float gyro_bias_rate_x;
    float gyro_bias_rate_y;
    float gyro_bias_rate_z;
    float gps_offset_x;
    float gps_offset_y;
    float gps_offset_z;
    float baro_offset;

} SensorComps;

float *com_to_imu(float seconds_since_launch, int launch_has_occurred);
    //launch_has_occurred = 1 if launch has occurred, =0 if launch has not occurred.

float pressure2altitude (float pressure);



#endif