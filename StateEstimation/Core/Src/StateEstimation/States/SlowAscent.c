/**
 * @file SlowAscent.c
 * @author Patrick Barry
 * @brief Source file for slow ascent state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include "States/SlowAscent.h"
#include "state_est_helpers.h"
#include "gen_constants.h"
/**
 * @brief This function contains all of the operations that occur specifically during slow ascent, when the rocket has no more thrust but is still moving upward.
 * @param ekf, the EKF struct; rocket_atd, the rocket attitude struct; sensors, a struct of most recent sensor measurements; serial_data, the data logging/transmitting struct
 * @return
 * @note Transmitting data to the main MCU and logging data are not unique slow ascent operations so are not included here.
*/
void run_slow_ascent(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, Sensors *sensors, SerialData *serial_data, UART_HandleTypeDef *huart) {
    if (first_iter) {
        activatedTOV = 0;
        first_iter = 0;
    }
    float32_t gyro_data[3] = {sensors->gyro_x, sensors->gyro_y, sensors->gyro_z};
    //run_attitude_estimation(rocket_atd, gyro_data);
   // run_ekf(ekf, rocket_atd, sensors, huart, 1);
    serial_data->state = SLOWASCENT;
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
    serial_data->wx = sensors->gyro_x - sensors->gyro_bias_x;
    serial_data->wy = sensors->gyro_y - sensors->gyro_bias_y;
    serial_data->wz = sensors->gyro_z - sensors->gyro_bias_z;
    prev_alt = serial_data->pos_z;
    int counter = 0;
    int num_loops_before_check = 20;
    if (global_time_seconds - fast_ascent_start_time > 11.0) {
        rocket_state = FREEFALL;
        first_iter = 1;
    } else {
        rocket_state = SLOWASCENT;
        first_iter = 0;
    }
    if ((ekf->x_n.pData[0] < prev_alt) && (counter % num_loops_before_check == 0)){ 
        if (activatedTOV) {
            float32_t TOV = global_time_seconds - startTOV;
            if (TOV > 3.0) {
                rocket_state = FREEFALL;
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
    counter++;
    if (counter % num_loops_before_check == 0) { 
        prev_alt = ekf->x_n.pData[0];
    }

}