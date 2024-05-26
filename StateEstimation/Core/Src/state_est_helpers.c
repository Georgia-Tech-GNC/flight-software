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

float pressure2altitude (float pressure) {

    float altitude = 44330 * (1.0 - pow( (pressure/100) / 1013.25, 0.1903)); // (sealvlhpa = 1013.25)
    return altitude;
}