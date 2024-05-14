/**
 * @file attitude.h
 * @author Patrick Barry
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

typedef struct { 
    /*Assumes q_current is knownf rom ground calibration*/
    float q_current_s; 
    float q_current_x;
    float q_current_y;
    float q_current_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float q_delt_s;
    float q_delt_x;
    float q_delt_y;
    float q_delt_z;

    float time_step;
    float prev_time_millis;

    float phi;
    float theta;
    float psi;
} rocket_attitude;

int64_t currentTimeMillis_att();
void update_time_step(rocket_attitude *rocket_atd);
void initialize_rocket_attitude(rocket_attitude *rocket_atd, float qs, float qx, float qy, float qz);
void set_gyro_x(rocket_attitude *rocket_atd, float wx);
void set_gyro_y(rocket_attitude *rocket_atd, float wy);
void set_gyro_z(rocket_attitude *rocket_atd, float wz);
void gyro_to_rotation_quat(rocket_attitude *rocket_atd);
void quat_update(rocket_attitude *rocket_atd);
void quat_to_euler_angs(rocket_attitude *rocket_atd);
void run_attitude_estimation(rocket_attitude *rocket_atd, float wx, float wy, float wz);


#endif