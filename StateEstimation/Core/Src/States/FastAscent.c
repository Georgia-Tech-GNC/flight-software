/**
 * @file FastAscent.c
 * @author Patrick Barry
 * @brief Source file for fast ascent state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include "../../Inc/States/FastAscent.h"

/**
 * @brief This function contains all of the operations that occur specifically during fast ascent, when the motor is propelling the rocket upward.
 * @param ekf, the EKF struct; rocket_atd, the rocket attitude struct; sensors, a struct of most recent sensor measurements; serial_data, the data logging/transmitting struct
 * @return
 * @note Transmitting data to the main MCU and logging data are not unique fast ascent operations so are not included here.
*/
void run_fast_ascent(ExtKalmanFilter *ekf, rocket_attitude *rocket_atd, Sensors *sensors, SerialData *serial_data){

    if (first_iter) {
        static float fast_ascent_start_time = (float)(GlobalTime)/1000.0;
        first_iter = 0;
    }

    float GPS_data[3] = {sensors->gps_x, sensors->gps_y, sensors->gps_z};
    float accel_data[3] = {sensors->accelerometer_x, sensors->accelerometer_y, sensors->accelerometer_z};

    run_attitude_estimation(rocket_atd, sensors->gyro_x, sensors->gyro_y, sensors->gyro_z);
    run_ekf(ekf, GPS_data, accel_data);

    //float phi = rocket_atd->phi;
    //float theta = rocket_atd->theta;
    //float psi = rocket_atd->psi;



    serial_data->state = 2; //Idle = 0, Ground = 1, Fast Ascent = 2, Slow Ascent = 3, Freefall = 4, Landed = 5
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


    //TODO: State transition check to slow ascent
    // can't detect filtered acceleration
    float burn_time = 10;
    if (GlobalTimeSeconds - fast_ascent_start_time > burn_time){
        // Switch states
        STATE_MACHINE = SLOWASCENT;
        first_iter = 1;
    }
    

}