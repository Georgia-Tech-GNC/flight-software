/**
 * @file FastAscent.c
 * @author Patrick Barry
 * @brief Source file for fast ascent state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/



void run_fast_ascent(ExtKalmanFilter *ekf, rocket_attitude *rocket_atd, Sensors *sensors, SerialData *serial_data){

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
    serial_data->q1 = rocket_atd->q_current_x
    serial_data->q2 = rocket_atd->q_current_y;
    serial_data->q3 = rocket_atd->q_current_z
    serial_data->wx = sensors->gyro_x;
    serial_data->wy = sensors->gyro_y;
    serial_data->wz = sensors->gyro_z;


    //TODO: State transition check to slow ascent

}