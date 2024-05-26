/**
 * @file SlowAscent.c
 * @author Patrick Barry
 * @brief Source file for slow ascent state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "../../Inc/ekf.h"
#include "../../Inc/attitude.h"
#include "../../Inc/States/Ground.h"

float *run_slow_ascent(ExtKalmanFilter *ekf, rocket_attitude *rocket_atd, float *GPS_data, float *accel_data, float *gyro_data){

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

    //TODO: State transition check to free fall

    return state_vector;

}