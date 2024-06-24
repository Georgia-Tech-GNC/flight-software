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

#include "../../Inc/States/SlowAscent.h"

/**
 * @brief This function contains all of the operations that occur specifically during slow ascent, when the rocket has no more thrust but is still moving upward.
 * @param ekf, the EKF struct; rocket_atd, the rocket attitude struct; sensors, a struct of most recent sensor measurements; serial_data, the data logging/transmitting struct
 * @return
 * @note Transmitting data to the main MCU and logging data are not unique slow ascent operations so are not included here.
*/
void run_slow_ascent(ExtKalmanFilter *ekf, rocket_attitude *rocket_atd, Sensors *sensors, SerialData *serial_data){

    if (first_iter) {
<<<<<<< HEAD
        static int activatedTOV = 0;
        float prevAlt;
=======
        static int activatedTOV = 0; // activated time of validity
        static float prevAlt;
>>>>>>> 70d328d11b7681ae405b01e8fe40bd24ff6d0163
        first_iter = 0;
    }

    float gps_world_x, gps_world_y, gps_world_z;   
    GPS2World(sensors->gps_x,sensors->gps_y,sensors->gps_z,&gps_world_x,&gps_world_y,&gps_world_z);

    float GPS_data[3] = {sensors->gps_x, sensors->gps_y, sensors->gps_z};
    float accel_data[3] = {sensors->accelerometer_x, sensors->accelerometer_y, sensors->accelerometer_z}; //TODO: Remove acceleration biases

    run_attitude_estimation(rocket_atd, sensors->gyro_x, sensors->gyro_y, sensors->gyro_z);
    run_ekf(&ekf, GPS_data, accel_data);

    //float phi = rocket_atd->phi;
    //float theta = rocket_atd->theta;
    //float psi = rocket_atd->psi;

    serial_data->state = 3; //Idle = 0, Ground = 1, Fast Ascent = 2, Slow Ascent = 3, Freefall = 4, Landed = 5
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

    // TOV - time of validity
    int counter = 0;
    int num_loops_before_check = 10; //TODO: Tune this value so that the condition below is checked every 1.0 seconds
    if ((ekf->x_n.pData[0] < prevAlt) && (counter % num_loops_before_check == 0)){ 
        if (activatedTOV) {

            float TOV = GlobalTimeSeconds - startTOV;

            // detect if rocket has been falling for 3 seconds straight
            // note: this assumes altitude is posn of cg of rocket (EKF needs to account for GPS lever arm)
            if (TOV > 3.0) {
                // Switch states
                STATE_MACHINE = FREEFALL;
                first_iter = 1;
            }

        } else {
            static float startTOV = GlobalTimeSeconds;
            static int activatedTOV = 1;
        }
    } else {
        startTOV = GlobalTimeSeconds;
        activatedTOV = 0;
    }
<<<<<<< HEAD
    counter = counter + 1;
    if (counter % num_loops_before_check == 0){ 
        float prevAlt = ekf->x_n.pData[0];
    }
=======
    
    static float prevAlt = ekf->x_n.pData[0];
    
>>>>>>> 70d328d11b7681ae405b01e8fe40bd24ff6d0163

}