/**
 * @file attitude.c
 * @author Patrick Barry
 * @brief Source file for in-flight attitude estimation
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * The materials provided are for the use of the students.
 * Copyrighted course materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/
#include <math.h>
#include "attitude.h"

int64_t currentTimeMillis_att(){
    struct timeval time;
    gettimeofday(&time, NULL);
    int64_t s1 = (int64_t)(time.tv_sec) * 1000;
    int64_t s2 = (time.tv_usec / 1000);
    return s1 + s2;
}

void update_time_step(rocket_attitude *rocket_atd){

    int64_t curr_time_millis = currentTimeMillis_att();
    float dt_millis = (float)(curr_time_millis - rocket_atd->prev_time_millis);
    rocket_atd->time_step = dt_millis / 1000;
    rocket_atd->prev_time_millis = curr_time_millis;

}

void initialize_rocket_attitude(rocket_attitude *rocket_atd, float qs, float qx, float qy, float qz){

    rocket_atd->q_current_s = qs;
    rocket_atd->q_current_x = qx;
    rocket_atd->q_current_y = qy;
    rocket_atd->q_current_z = qz;

    rocket_atd->time_step = 0.02;

    rocket_atd->q_delt_s = 0.0;
    rocket_atd->q_delt_x = 0.0;
    rocket_atd->q_delt_y = 0.0;
    rocket_atd->q_delt_z = 0.0;

    rocket_atd->gyro_x = 0.0;
    rocket_atd->gyro_y = 0.0;
    rocket_atd->gyro_z = 0.0;

    rocket_atd->prev_time_millis = currentTimeMillis_att();
}
/**
 * @brief Set the value of the gyro_x measurement that the attitude update system is using
 * 
 * This function simply retrieves the latest 'angular rate about the x-axis' measurement from the ADIS16500 IMU
 *  and sets the gyro_x value of the attitude estimation struct to the value of this measurement.
 * @param rocket_atd (struct that attitude estimation system is built out of)
*/
void set_gyro_x(rocket_attitude *rocket_atd, float wx) { //Need to hook this up with IMU driver
    rocket_atd->gyro_x = wx;
}

/**
 * @brief Set the value of the gyro_y measurement that the attitude update system is using
 * 
 * This function simply retrieves the latest 'angular rate about the y-axis' measurement from the ADIS16500 IMU
 * and sets the gyro_y value of the attitude estimation struct to the value of this measurement.
 * @param rocket_atd (struct that attitude estimation system is built out of)
*/
void set_gyro_y(rocket_attitude *rocket_atd, float wy){ //Need to hook this up with IMU driver
    rocket_atd->gyro_y = wy;
}

/**
 * @brief Set the value of the gyro_z measurement that the attitude update system is using
 * 
 * This function simply retrieves the latest 'angular rate about the z-axis' measurement from the ADIS16500 IMU
 * and sets the gyro_z value of the attitude estimation struct to the value of this measurement.
 * @param rocket_atd (struct that attitude estimation system is built out of)
*/
void set_gyro_z(rocket_attitude *rocket_atd, float wz){ //Need to hook this up with IMU driver
    rocket_atd->gyro_z = wz;
}

/**
 * @brief Take gyro measurement data and convert it into an instantaneous rotation quaternion.
 * 
 * This function uses the attitude estimation struct's values for wx, wy, and wz (angular rates in the body frame, rad/s)
 * and converts them to an instantaneous rotation quaternion that may be later used to update the attitude of the rocket.
 * The instantaneous rotation quaternion is four elements, q_delt_{s, x, y, z} and is stored in the attitude estimation struct.
 * @param rocket_atd (struct that attitude estimation system is built out of)
 * @note If all gyro measurements are zero, which is technically a possible outcome while in flight, this would result in this
 * function failing due to division by zero. As a result, if all gyro measurements are zero, 0.01 is added to make the norm of 
 * the vector w = [wx, wy, wz] a nonzero value. 
*/
void gyro_to_rotation_quat(rocket_attitude *rocket_atd){ 
    float omega[] = {rocket_atd->gyro_x, rocket_atd->gyro_y, rocket_atd->gyro_z}; //Create a vector omega = [wx, wy, wz]
    float norm = sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]); //Calculate the norm of this vector

    /*Little bit more similar to C drivers, but option way to do the if statement */
    omega[0] = norm == 0 ? omega[0] + 0.01 : omega[0]; //If the norm is zero, add 0.01 to each component
    omega[1] = norm == 0 ? omega[2] + 0.01 : omega[1];
    omega[2] = norm == 0 ? omega[2] + 0.01 : omega[2];

    norm = sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]); //Re-calculate the norm
    /*
    if (norm == 0){
        omega[0] += 0.01;
        omega[1] += 0.01;
        omega[2] += 0.01;
    }
    */

    float axis[] = {omega[0], omega[1], omega[2]}; //Find the axis of instantaneous rotation based off of  gyro measurements.
    axis[0] /= norm;
    axis[1] /= norm;
    axis[2] /= norm;

    float angle = rocket_atd->time_step*norm; //Find the angle that the rocket rotates about the axis of instantaneous rotation.

    rocket_atd->q_delt_s = cos(angle/2.0f); //Definition of quaternion elements... simply forming the instantaneous rotation quat from axis-angle representation
    rocket_atd->q_delt_x = axis[0]*sin(angle/2.0f);
    rocket_atd->q_delt_y = axis[0]*sin(angle/2.0f);
    rocket_atd->q_delt_z = axis[0]*sin(angle/2.0f);

}

/**
 * @brief This function updates the quaternion representing the rocket's attitude. The rocket's attitude,
 * which is represented by the four-element quaternion in the attitude estimation struct q_current{s, x, y z},
 * represents the rotation from the North East Down (NED) frame to the rocket's body frame.
 * 
 * @param rocket_atd (struct that attitude estimation system is built out of)
*/
void quat_update(rocket_attitude *rocket_atd){

    float q_new_s = rocket_atd->q_current_s*rocket_atd->q_delt_s //Find the value of the scalar component q_s for the rocket's attitude quaternion.
                    - rocket_atd->q_current_x*rocket_atd->q_delt_x
                    - rocket_atd->q_current_y*rocket_atd->q_delt_y
                    - rocket_atd->q_current_z*rocket_atd->q_delt_z;

    float q_new_x = rocket_atd->q_current_s*rocket_atd->q_delt_x //Find the value of q_x for the rocket's attitude quaternion.
                    + rocket_atd->q_current_x*rocket_atd->q_delt_s
                    + rocket_atd->q_current_y*rocket_atd->q_delt_z
                    - rocket_atd->q_current_z*rocket_atd->q_delt_y;

    float q_new_y = rocket_atd->q_current_s*rocket_atd->q_delt_y //FInd the value of q_y for the rocket's attitude quaternion.
                    + rocket_atd->q_current_y*rocket_atd->q_delt_s
                    + rocket_atd->q_current_z*rocket_atd->q_delt_x
    -                rocket_atd->q_current_x*rocket_atd->q_delt_z;

    float q_new_z = rocket_atd->q_current_s*rocket_atd->q_delt_z //Find the value of q_z for the rocket's attitude quaternion.
                    + rocket_atd->q_current_z*rocket_atd->q_delt_s
                    + rocket_atd->q_current_x*rocket_atd->q_delt_y
                    - rocket_atd->q_current_y*rocket_atd->q_delt_x;

    rocket_atd->q_current_s = q_new_s; //Update the values in the attitude estimation struct to reflect the new attitude quaternion.
    rocket_atd->q_current_x = q_new_x;
    rocket_atd->q_current_y = q_new_y;
    rocket_atd->q_current_z = q_new_z;

}

void quat_to_euler_angs(rocket_attitude *rocket_atd){

    float qs = rocket_atd->q_current_s;
    float qx = rocket_atd->q_current_x;
    float qy = rocket_atd->q_current_y;
    float qz = rocket_atd->q_current_z;

    float C11 = qs*qs + qx*qx - qy*qy - qz*qz;
    float C12 = 2.0*(qx*qy+qz*qs);
    float C13 = 2.0*(qx*qz-qy*qs);
    float C21 = 2.0*(qx*qy-qz*qs);
    float C22 = qs*qs - qx*qx + qy*qy - qz*qz;
    float C23 = 2.0*(qy*qz + qx*qs);
    float C31 = 2.0*(qx*qz + qy*qs);
    float C32 = 2.0*(qy*qz - qx*qs);
    float C33 = qs*qs - qx*qx - qy*qy + qz*qz;

    rocket_atd->phi = (float)atan2((double)C23, (double)C33);
    rocket_atd->theta = -(float)asin((double)C13);
    rocket_atd->psi = (float)atan2((double)C12, (double)C11);

}

void run_attitude_estimation(rocket_attitude *rocket_atd, float wx, float wy, float wz){

    set_gyro_x(rocket_atd, wx);
    set_gyro_y(rocket_atd, wy);
    set_gyro_z(rocket_atd, wz);

    update_time_step(rocket_atd);

    gyro_to_rotation_quat(rocket_atd);
    quat_update(rocket_atd);
    quat_to_euler_angs(rocket_atd);
}