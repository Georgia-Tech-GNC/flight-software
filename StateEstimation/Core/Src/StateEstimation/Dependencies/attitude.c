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

#include "attitude.h"

void initialize_rocket_attitude(RocketAttitude *rocket_atd, float32_t qs, float32_t qx, float32_t qy, float32_t qz){
    rocket_atd->q_current_s = qs;
    rocket_atd->q_current_x = qx;
    rocket_atd->q_current_y = qy;
    rocket_atd->q_current_z = qz;
    rocket_atd->q_delt_s = 0.0;
    rocket_atd->q_delt_x = 0.0;
    rocket_atd->q_delt_y = 0.0;
    rocket_atd->q_delt_z = 0.0;
    rocket_atd->gyro_x = 0.0;
    rocket_atd->gyro_y = 0.0;
    rocket_atd->gyro_z = 0.0;
}
/**
 * @brief Set the value of the gyro measurement that the attitude update system is using
 * 
 * This function simply retrieves the latest 'angular rate about the x, y, and z axis' measurement from the ADIS16500 IMU
 *  and sets the gyro value of the attitude estimation struct to the value of this measurement.
 * @param rocket_atd (struct that attitude estimation system is built out of)
*/
void set_gyro(RocketAttitude *rocket_atd, float32_t* readings) {
    rocket_atd->gyro_x = readings[0];
    rocket_atd->gyro_y = readings[1];
    rocket_atd->gyro_z = readings[2];
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
void gyro_to_rotation_quat(RocketAttitude *rocket_atd){ 
    float32_t omega[] = {rocket_atd->gyro_x, rocket_atd->gyro_y, rocket_atd->gyro_z}; //Create a vector omega = [wx, wy, wz]
    float32_t norm = sqrt(omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]); //Calculate the norm of this vector

    omega[0] = norm == 0 ? omega[0] + 0.01 : omega[0];
    omega[1] = norm == 0 ? omega[1] + 0.01 : omega[1];
    omega[2] = norm == 0 ? omega[2] + 0.01 : omega[2];

    norm = sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]); //Re-calculate the norm
    /*
    if (norm == 0){
        omega[0] += 0.01;
        omega[1] += 0.01;
        omega[2] += 0.01;
    }
    */

    float32_t axis[] = {omega[0], omega[1], omega[2]}; //Find the axis of instantaneous rotation based off of  gyro measurements.
    axis[0] /= norm;
    axis[1] /= norm;
    axis[2] /= norm;

    float32_t angle = rocket_atd->time_step * norm; //Find the angle that the rocket rotates about the axis of instantaneous rotation.

    rocket_atd->q_delt_s = cos(angle / 2.0f); //Definition of quaternion elements... simply forming the instantaneous rotation quat from axis-angle representation
    rocket_atd->q_delt_x = axis[0] * sin(angle/2.0f);
    rocket_atd->q_delt_y = axis[1] * sin(angle/2.0f);
    rocket_atd->q_delt_z = axis[2] * sin(angle/2.0f);

}

/**
 * @brief This function updates the quaternion representing the rocket's attitude. The rocket's attitude,
 * which is represented by the four-element quaternion in the attitude estimation struct q_current{s, x, y z},
 * represents the rotation from the North East Down (NED) frame to the rocket's body frame.
 * 
 * @param rocket_atd (struct that attitude estimation system is built out of)
*/
void quat_update(RocketAttitude *rocket_atd){

    float32_t q_new_s = rocket_atd->q_current_s * rocket_atd->q_delt_s //Find the value of the scalar component q_s for the rocket's attitude quaternion.
                    - rocket_atd->q_current_x * rocket_atd->q_delt_x
                    - rocket_atd->q_current_y * rocket_atd->q_delt_y
                    - rocket_atd->q_current_z * rocket_atd->q_delt_z;

    float32_t q_new_x = rocket_atd->q_current_s * rocket_atd->q_delt_x //Find the value of q_x for the rocket's attitude quaternion.
                    + rocket_atd->q_current_x * rocket_atd->q_delt_s
                    + rocket_atd->q_current_y * rocket_atd->q_delt_z
                    - rocket_atd->q_current_z * rocket_atd->q_delt_y;

    float32_t q_new_y = rocket_atd->q_current_s * rocket_atd->q_delt_y //FInd the value of q_y for the rocket's attitude quaternion.
                    + rocket_atd->q_current_y * rocket_atd->q_delt_s
                    + rocket_atd->q_current_z * rocket_atd->q_delt_x
    -                rocket_atd->q_current_x * rocket_atd->q_delt_z;

    float32_t q_new_z = rocket_atd->q_current_s * rocket_atd->q_delt_z //Find the value of q_z for the rocket's attitude quaternion.
                    + rocket_atd->q_current_z * rocket_atd->q_delt_s
                    + rocket_atd->q_current_x * rocket_atd->q_delt_y
                    - rocket_atd->q_current_y * rocket_atd->q_delt_x;
    float32_t norm = sqrt(q_new_s * q_new_s + q_new_x * q_new_x + q_new_y * q_new_y + q_new_z * q_new_z);
    rocket_atd->q_current_s = q_new_s / norm; //Update the values in the attitude estimation struct to reflect the new attitude quaternion.
    rocket_atd->q_current_x = q_new_x / norm;
    rocket_atd->q_current_y = q_new_y / norm;
    rocket_atd->q_current_z = q_new_z / norm;

}
/**
 * @brief This function, when called, updates the euler angles in the rocket_atd struct based on the current quaternion in the rocket_atd struct
 * @param rocket_atd (rocket attitude struct)
 * @return None
 * @note This is not strictly necessary as our present control algorithm uses quaternion attitude representation. However, it may be helpful
 * if controls need to be based off of Euler angles or for debugging. 
*/
void quat_to_euler_angs(RocketAttitude *rocket_atd){

    float32_t qs = rocket_atd->q_current_s;
    float32_t qx = rocket_atd->q_current_x;
    float32_t qy = rocket_atd->q_current_y;
    float32_t qz = rocket_atd->q_current_z;

    float32_t C11 = qs*qs + qx*qx - qy*qy - qz*qz;
    float32_t C12 = 2.0*(qx*qy+qz*qs);
    float32_t C13 = 2.0*(qx*qz-qy*qs);
    //float32_tC21 = 2.0*(qx*qy-qz*qs); 
    //float32_tC22 = qs*qs - qx*qx + qy*qy - qz*qz;
    float32_t C23 = 2.0*(qy*qz + qx*qs);
    //float32_tC31 = 2.0*(qx*qz + qy*qs);
    //float32_tC32 = 2.0*(qy*qz - qx*qs);
    float32_t C33 = qs*qs - qx*qx - qy*qy + qz*qz;

    rocket_atd->phi = (float) atan2((double)C23, (double)C33);
    rocket_atd->theta = -(float) asin((double)C13);
    rocket_atd->psi = (float) atan2((double)C12, (double)C11);

}

/**
 * @brief This function runs one loop of attitude estimation, taking in gyro measurements and updating the rocket quaternion
 * @param rocket_atd (rocket attitude struct) and gyro measurements wx, wy, wz
 * @return
 * @note
*/
void run_attitude_estimation(RocketAttitude *rocket_atd, float32_t *w){
    set_gyro(rocket_atd, w);
    gyro_to_rotation_quat(rocket_atd);
    quat_update(rocket_atd);
    quat_to_euler_angs(rocket_atd); //Not necessary to include right now but if not too slow then may still be included
}