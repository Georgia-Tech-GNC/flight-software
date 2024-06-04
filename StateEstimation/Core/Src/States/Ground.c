/**
 * @file ground_ekf.c
 * @author Albert Zheng
 * @brief Source file for ground EKF
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "../../Inc/ekf.h"
#include "../../Inc/States/Ground.h"
#include "../../Inc/data_handling.h"

void state_transition_ground(ExtKalmanFilter *gekf) {
    // Calculates next state for ground EKF

    float f_new_data[gekf->nx];

    // accelo bias
    f_new_data[0] = gekf->x_n.pData[0];
    f_new_data[1] = gekf->x_n.pData[1];
    f_new_data[2] = gekf->x_n.pData[2];

    // gyro bias
    f_new_data[3] = gekf->x_n.pData[3];
    f_new_data[4] = gekf->x_n.pData[4];
    f_new_data[5] = gekf->x_n.pData[5];

    //gps offset
    f_new_data[6] = gekf->x_n.pData[6];
    f_new_data[7] = gekf->x_n.pData[7];
    f_new_data[8] = gekf->x_n.pData[8];

    // baro offset
    f_new_data[9] = gekf->x_n.pData[9];


    arm_matrix_instance_f32 f_new = {gekf->nx, 1, f_new_data};
    gekf->f = f_new;


}

void state_transition_jacob_ground(ExtKalmanFilter *gekf) {
    // Calculates state transition jacobian for ground EKF

    // TODO may need to update as gnd sims progresses
    float dfdx_new_data[gekf->nx][gekf->nx] = {1,0,0,0,0,0,0,0,0,0,
                                               0,1,0,0,0,0,0,0,0,0,
                                               0,0,1,0,0,0,0,0,0,0,
                                               0,0,0,1,0,0,0,0,0,0,
                                               0,0,0,0,1,0,0,0,0,0,
                                               0,0,0,0,0,1,0,0,0,0,
                                               0,0,0,0,0,0,1,0,0,0,
                                               0,0,0,0,0,0,0,1,0,0,
                                               0,0,0,0,0,0,0,0,1,0,
                                               0,0,0,0,0,0,0,0,0,1};

    arm_matrix_instance_f32 dfdx_new = {gekf->nx,gekf->nx,dfdx_new_data};
    gekf->dfdx = dfdx_new;


}

void observation_ground(ExtKalmanFilter *gekf) {
    // Calculates state-to-measurement relation for ground EKF


    float h_new_data[gekf->nz];

    h_new_data[0] = gekf->x_n.pData[0];
    h_new_data[1] = gekf->x_n.pData[1],
    h_new_data[2] = 9.81 + gekf->x_n.pData[2],
    h_new_data[3] = gekf->x_n.pData[3];
    h_new_data[4] = gekf->x_n.pData[4];
    h_new_data[5] = gekf->x_n.pData[5];

    h_new_data[6] = gekf->x_n.pData[6];
    h_new_data[7] = gekf->x_n.pData[7];
    h_new_data[8] = gekf->x_n.pData[8];
    h_new_data[9] = gekf->x_n.pData[9];

    arm_matrix_instance_f32 h_new = {gekf->nz, 1, h_new_data};
    gekf->h = h_new;


}

void observation_jacob_ground(ExtKalmanFilter *gekf) {
    // Calculates observation jacobian for ground EKF

    float dhdx_new_data[gekf->nz][gekf->nx] = {1,0,0,0,0,0,0,0,0,0,
                                               0,1,0,0,0,0,0,0,0,0,
                                               0,0,1,0,0,0,0,0,0,0,
                                               0,0,0,1,0,0,0,0,0,0,
                                               0,0,0,0,1,0,0,0,0,0,
                                               0,0,0,0,0,1,0,0,0,0,
                                               0,0,0,0,0,0,1,0,0,0,
                                               0,0,0,0,0,0,0,1,0,0,
                                               0,0,0,0,0,0,0,0,1,0,
                                               0,0,0,0,0,0,0,0,0,1};

    arm_matrix_instance_f32 dhdx_new = {gekf->nz,gekf->nx,dhdx_new_data};
    gekf->dhdx = dhdx_new;

}

int check_gekf_convergence(ExtKalmanFilter *gekf) {

    for (int i=0; i<=gekf->nx; i++) {

        if (gekf->P_n.pData[i][i] > 0.1) {

            return 0;

        }
    }

    return 1;
}

void GPS2ECEF(float lat, float lon, float alt, float* x, float* y, float* z) {
    // Converts from WGS84 to ECEF

    // Convert latitude and longitude from degrees to radians
    double coslat = cos(lat * (M_PI / 180.0));
    double sinlat = sin(lat * (M_PI / 180.0));
    double coslon = cos(lon * (M_PI / 180.0));
    double sinlon = sin(lon * (M_PI / 180.0));

    double e = sqrt(1 - (WGS84_B**2/WGS84_A**2))
    double N = WGS84_A / sqrt(1.0 - e * e * sinlat * sinlat);

    *x = (N + alt) * coslat * coslon;
    *y = (N + alt) * coslat * sinlon;
    *z = (N * (1.0 - e**2) + alt) * sinlat;
}

void run_ground(ExtKalmanFilter* gekf, Sensors* sensors, SerialData *serial_data) {

    // Loop
    while (STATE_MACHINE == GROUND) {

        update_time_step(gekf);

        // adis_get_data(imu_data); // read from imu

        // double mag_reading[3]; // read from mag
        // int status = lis3mdl_read_mag(lis_mag,mag_reading);

        // MS5607Update(); // read from baro

        // Read sensor data
        // sensors->accelerometer_x = imu_data->x_accl_out;
        // sensors->accelerometer_y = imu_data->y_accl_out;
        // sensors->accelerometer_z = imu_data->z_accl_out;
        // sensors->gyro_x = imu_data->x_gyro_out;
        // sensors->gyro_y = imu_data->y_gyro_out;
        // sensors->gyro_z = imu_data->z_gyro_out;
        // sensors->magneto_x = mag_reading[0];
        // sensors->magneto_y = mag_reading[1];
        // sensors->magneto_z = mag_reading[2];
        // sensors->baro = pressure2altitude(float(reading_baro->pressure));
        // GPS2World(sensors->gps_x,sensors->gps_y,sensors->gps_z,&gps_world_x,&gps_world_y,&gps_world_z);

        // Read sensors
        read_sensors(sensors);
        float gps_world_x, gps_world_y, gps_world_z;
        GPS2World(sensors->gps_x,sensors->gps_y,sensors->gps_z,&gps_world_x,&gps_world_y,&gps_world_z);

        //Measurement function
        gekf->z[0] = sensors->accelerometer_x;
        gekf->z[1] = sensors->accelerometer_y;
        gekf->z[2] = sensors->accelerometer_z;
        gekf->z[3] = sensors->gyro_x;
        gekf->z[4] = sensors->gyro_y;
        gekf->z[5] = sensors->gyro_z;
        gekf->z[6] = gps_world_x;
        gekf->z[7] = gps_world_y;
        gekf->z[8] = gps_world_z;
        gekf->z[9] = sensors->baro;

        //Update
        observation_function(gekf);
        observation_jacobian(gekf);
        kalman_gain(gekf);
        update_state(gekf);
        update_covariance(gekf);

        //Predict
        state_transition_function(gekf);
        state_transition_jacobian(gekf);
        predict_state(gekf);
        predict_covariance(gekf);

        acknowledge_time_passed(gekf);


        int gekf_has_converged = check_gekf_convergence(gekf);

        // Update SerialData packet
        serial_data->state = STATE_MACHINE;
        serial_data->pos_x = 0.0;
        serial_data->pos_y = 0.0;
        serial_data->pos_z = 0.0;
        serial_data->vel_x = 0.0;
        serial_data->vel_y = 0.0;
        serial_data->vel_z = 0.0;
        serial_data->q0 = 1.0;
        serial_data->q1 = 0.0;
        serial_data->q2 = 0.0;
        serial_data->q3 = 0.0;
        serial_data->wx = 0.0;
        serial_data->wy = 0.0;
        serial_data->wz = 0.0;
        

        // State transition conditions
        if (gekf_has_converged && signal_received) {

            // Upload sensor compensations
            sensors->accel_bias_x = gekf->x_n.pData[0];
            sensors->accel_bias_y = gekf->x_n.pData[1];
            sensors->accel_bias_z = gekf->x_n.pData[2];
            sensors->gyro_bias_x = gekf->x_n.pData[3];
            sensors->gyro_bias_y = gekf->x_n.pData[4];
            sensors->gyro_bias_z = gekf->x_n.pData[5];
            sensors->gps_offset_x = gekf->x_n.pData[6];
            sensors->gps_offset_y = gekf->x_n.pData[7];
            sensors->gps_offset_z = gekf->x_n.pData[8];
            sensors->baro_offset = gekf->x_n.pData[9];

            // Switch states
            STATE_MACHINE = FASTASCENT;
            first_iter = 0;
        }

    }





}