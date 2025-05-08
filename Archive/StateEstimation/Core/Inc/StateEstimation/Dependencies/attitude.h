/**
 * @file attitude.h
 * @author Patrick Barry, Kanav Chugh
 * @brief This contains the definitions of functions needed for in-flight attitude estimation.
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * The materials provided are for the use of the students.
 * Copyrighted course materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include <stdint.h>
#include <sys/time.h>
#include <math.h>
#include "arm_math.h"

typedef struct { 
    /*Assumes q_current is knownf rom ground calibration*/
    float32_t q_current_s; 
    float32_t q_current_x;
    float32_t q_current_y;
    float32_t q_current_z;

    float32_t gyro_x;
    float32_t gyro_y;
    float32_t gyro_z;

    float32_t q_delt_s;
    float32_t q_delt_x;
    float32_t q_delt_y;
    float32_t q_delt_z;

    float32_t time_step;

    float32_t phi;
    float32_t theta;
    float32_t psi;
} RocketAttitude;

void initialize_rocket_attitude(RocketAttitude *rocket_atd, float32_t qs, float32_t qx, float32_t qy, float32_t qz);
void set_gyro(RocketAttitude *rocket_atd, float32_t* readings);
void gyro_to_rotation_quat(RocketAttitude *rocket_atd);
void quat_update(RocketAttitude *rocket_atd);
void quat_to_euler_angs(RocketAttitude *rocket_atd);
void run_attitude_estimation(RocketAttitude *rocket_atd, float32_t* w);


#endif