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

#define DT 0.02f //Should be the amount of time between gyro measurements. May be variable.

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
} rocket_attitude;


void set_gyro_x(rocket_attitude *rocket_atd);
void set_gyro_y(rocket_attitude *rocket_atd);
void set_gyro_z(rocket_attitude *rocket_atd);
void gyro_to_rotation_quat(rocket_attitude *rocket_atd);
void quat_update(rocket_attitude *rocket_atd);
 
#endif