/**
 * @file FreeFall.c
 * @author Patrick Barry
 * @brief Source file for free fall state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include "States/FreeFall.h"
#include "gen_constants.h"

/**
 * @brief This function contains all of the operations that occur specifically during freefall, when the rocket is falling to the ground.
 * @param ekf, the EKF struct; rocket_atd, the rocket attitude struct; sensors, a struct of most recent sensor measurements; serial_data, the data logging/transmitting struct
 * @note Transmitting data to the main MCU and logging data are not unique freefall operations so are not included here.
*/
void run_freefall(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, Sensors *sensors, SerialData *serial_data, UART_HandleTypeDef *huart){

    if (first_iter) {
        activatedTOV = 0;
        first_iter = 0;
    }

    //float32_t GPS_data[3] = {sensors->gps_x, sensors->gps_y, sensors->gps_z};
    //float32_t accel_data[3] = {sensors->accel_x, sensors->accel_y, sensors->accel_z};
    float32_t gyro_data[3] = {sensors->gyro_x, sensors->gyro_y, sensors->gyro_z};
    run_attitude_estimation(rocket_atd, gyro_data);
    run_ekf(ekf, rocket_atd, sensors, huart, 1);

    //float32_tphi = rocket_atd->phi;
    //float32_ttheta = rocket_atd->theta;
    //float32_tpsi = rocket_atd->psi;

    serial_data->state = FREEFALL; //Idle = 0, Ground = 1, Fast Ascent = 2, Slow Ascent = 3, Freefall = 4, Landed = 5
    serial_data->pos_x = ekf->x_n.pData[0];
    serial_data->pos_y = ekf->x_n.pData[2];
    serial_data->pos_z = ekf->x_n.pData[4];
    serial_data->vel_x = ekf->x_n.pData[1];
    serial_data->vel_y = ekf->x_n.pData[3];
    serial_data->vel_z = ekf->x_n.pData[5];
    serial_data->q0 = rocket_atd->q_current_s;  
    serial_data->q1 = rocket_atd->q_current_x;
    serial_data->q2 = rocket_atd->q_current_y;
    serial_data->q3 = rocket_atd->q_current_z;
    serial_data->wx = sensors->gyro_x;
    serial_data->wy = sensors->gyro_y;
    serial_data->wz = sensors->gyro_z;


    //TODO: State transition check to landed
    // TOV - time of validity
    if (ekf->x_n.pData[1] > -0.2 && ekf->x_n.pData[1] < 0.2){ //Check if global x velocity is within bound. Could also check y and z velocities
        if (activatedTOV) {
            float32_t TOV = global_time_seconds - startTOV;
            if (TOV > 3.0) {
                // Switch states
                rocket_state = LANDED;
                first_iter = 1;
            }

        } else {
            startTOV = global_time_seconds;
            activatedTOV = 1;
        }
    } else {
        startTOV = global_time_seconds;
        activatedTOV = 0;
    }


}