/**
 * @file FastAscent.c
 * @author Patrick Barry, Kanav Chugh
 * @brief Source file for fast ascent state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include "States/FastAscent.h"
#include "gen_constants.h"

/**
 * @brief This function contains all of the operations that occur specifically during fast ascent, when the motor is propelling the rocket upward.
 * @param ekf, the EKF struct; rocket_atd, the rocket attitude struct; sensors, a struct of most recent sensor measurements; serial_data, the data logging/transmitting struct
 * @return
 * @note Transmitting data to the main MCU and logging data are not unique fast ascent operations so are not included here.
*/
void run_fast_ascent(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, Sensors *sensors, SerialData *serial_data, UART_HandleTypeDef *huart) {

    if (first_iter) {
        fast_ascent_start_time = (float32_t) (global_time) / 1000.0f;
        first_iter = 0;
    }
    //run_attitude_estimation(rocket_atd, ekf->gyro);
    //run_ekf(ekf, rocket_atd, sensors, huart, 1);
    serial_data->state = FASTASCENT;
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
    if (global_time_seconds - fast_ascent_start_time > BURN_TIME) {
        rocket_state = SLOWASCENT;
        first_iter = 1;
    } else {
        rocket_state = FASTASCENT;
        first_iter = 0;
    }
}
