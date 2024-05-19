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
#include <lapacke.h>
#include <sys/time.h>

#include "../../Inc/Utils/ekf.h"
#include "../../Inc/States/Ground.h"

void state_transition_ground(ExtKalmanFilter *gekf) {
    // Calculates next state for ground EKF

    float f_new_data[gekf->nx];

    // accelo bias
    f_new_data[0] = gekf->x_n.pData[0] + gekf->time_step * gekf->x_n.pData[3];
    f_new_data[1] = gekf->x_n.pData[1] + gekf->time_step * gekf->x_n.pData[4];
    f_new_data[2] = gekf->x_n.pData[2] + gekf->time_step * gekf->x_n.pData[5];

    // accelo bias rate 
    f_new_data[3] = gekf->x_n.pData[3];
    f_new_data[4] = gekf->x_n.pData[4];
    f_new_data[5] = gekf->x_n.pData[5];

    // gyro bias
    f_new_data[6] = gekf->x_n.pData[6] + gekf->time_step * gekf->x_n.pData[9];
    f_new_data[7] = gekf->x_n.pData[7] + gekf->time_step * gekf->x_n.pData[10];
    f_new_data[8] = gekf->x_n.pData[8] + gekf->time_step * gekf->x_n.pData[11];

    // gyro bias rate
    f_new_data[9] = gekf->x_n.pData[9];
    f_new_data[10] = gekf->x_n.pData[10];
    f_new_data[11] = gekf->x_n.pData[11];

    //magneto bias
    f_new_data[12] = gekf->x_n.pData[12] + gekf->time_step * gekf->x_n.pData[15];
    f_new_data[13] = gekf->x_n.pData[13] + gekf->time_step * gekf->x_n.pData[16];
    f_new_data[14] = gekf->x_n.pData[14] + gekf->time_step * gekf->x_n.pData[17];

    // magneto bias rate
    f_new_data[15] = gekf->x_n.pData[15];
    f_new_data[16] = gekf->x_n.pData[16];
    f_new_data[17] = gekf->x_n.pData[17];

    // baro bias
    f_new_data[18] = gekf->x_n.pData[18];

    arm_matrix_instance_f32 f_new = {gekf->nx, 1, f_new_data};
    gekf->f = f_new;


}

void state_transition_jacob_ground(ExtKalmanFilter *gekf) {
    // Calculates state transition jacobian for ground EKF

    // TODO may need to update as gnd sims progresses
    float dfdx_new_data[gekf->nx][gekf->nx] = {{1, 0, 0, gekf.time_step,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  0},
                                                {0, 1, 0,  0, gekf.time_step,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  0},
                                                {0, 0, 1,  0,  0, gekf.time_step, 0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  0},
                                                {0, 0, 0,  1,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  0},
                                                {0, 0, 0,  0,  1,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  0},
                                                {0, 0, 0,  0,  0,  1, 0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  0},
                                                {0, 0, 0,  0,  0,  0, 1, 0, 0, gekf.time_step,  0,  0, 0, 0, 0,  0,  0,  0},
                                                {0, 0, 0,  0,  0,  0, 0, 1, 0,  0, gekf.time_step,  0, 0, 0, 0,  0,  0,  0},
                                                {0, 0, 0,  0,  0,  0, 0, 0, 1,  0,  0, gekf.time_step, 0, 0, 0,  0,  0,  0},
                                                {0, 0, 0,  0,  0,  0, 0, 0, 0,  1,  0,  0, 0, 0, 0,  0,  0,  0},
                                                {0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  1,  0, 0, 0, 0,  0,  0,  0},
                                                {0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  1, 0, 0, 0,  0,  0,  0},
                                                {0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  0, 1, 0, 0, gekf.time_step,  0,  0},
                                                {0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  0, 0, 1, 0,  0, gekf.time_step,  0},
                                                {0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 1,  0,  0, gekf.time_step},
                                                {0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0,  1,  0,  0},
                                                {0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  1,  0},
                                                {0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0,  0,  0,  1}};

    arm_matrix_instance_f32 dfdx_new = {gekf->nx,gekf->nx,dfdx_new_data};
    gekf->dfdx = dfdx_new;


}

void observation_ground(ExtKalmanFilter *gekf) {
    // Calculates state-to-measurement relation for ground EKF


    float h_new_data[gekf->nz];

    h_new_data[0] = gekf->x_n.pData[0];
    h_new_data[1] = gekf->x_n.pData[1],
    h_new_data[2] = 9.81 + gekf->x_n.pData[2],
    h_new_data[3] = gekf->x_n.pData[6];
    h_new_data[4] = gekf->x_n.pData[7];
    h_new_data[5] = gekf->x_n.pData[8];
    h_new_data[6] = gekf->mx0 + gekf->x_n.pData[12];
    h_new_data[7] = gekf->my0 + gekf->x_n.pData[13];
    h_new_data[8] = gekf->mz0 + gekf->x_n.pData[14];
    h_new_data[9] = gekf->x_n.pData[18];

    arm_matrix_instance_f32 h_new = {gekf->nz, 1, h_new_data};
    gekf->h = h_new;


}

void observation_jacob_ground(ExtKalmanFilter *gekf) {
    // Calculates observation jacobian for ground EKF

    float dhdx_new_data[gekf->nz][gekf->nx] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0};

    arm_matrix_instance_f32 dhdx_new = {gekf->nx,gekf->nx,dhdx_new_data};
    gekf->dhdx = dhdx_new;

}

int check_gekf_convergence(ExtKalmanFilter *gekf) {

    for (int i=0; i<=gekf->nx; i++) {

        if (gekf->P_n.pData[i][i] > 0.01) {

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


void get_ground_attitude(ExtKalmanFilter *gekf) {



}

void run_ground() {

    // Setup
    if (first_iter) {
        static ExtKalmanFilter* gekf;
        static float ground_quat[4];
    }

    // Loop
    while (STATE_MACHINE == GROUND) {

        update_time_step(gekf);

        adis_get_data(imu_data); // read from imu

        double mag_reading[3]; // read from mag
        int status = lis3mdl_read_mag(lis_mag,mag_reading);

        MS5607Update(); // read from baro

        float gps_reading = ?;
        float gps_ecef_x, gps_ecef_y, gps_ecef_z;
        GPS2ECEF(gps_reading[0],gps_reading[1],gps_reading[2],&gps_ecef_x,&gps_ecef_y,&gps_ecef_z);

        // Read sensor data
        gekf->accelerometer[0] = imu_data->x_accl_out;
        gekf->accelerometer[1] = imu_data->y_accl_out;
        gekf->accelerometer[2] = imu_data->z_accl_out;
        gekf->gyro[0] = imu_data->x_gyro_out;
        gekf->gyro[1] = imu_data->y_gyro_out;
        gekf->gyro[2] = imu_data->z_gyro_out;
        gekf->magneto[0] = mag_reading[0];
        gekf->magneto[1] = mag_reading[1];
        gekf->magneto[2] = mag_reading[2];
        gekf->baro = pressure2altitude(float(reading_baro->pressure));
        gekf->gps[0] = gps_ecef_x;
        gekf->gps[1] = gps_ecef_y;
        gekf->gps[2] = gps_ecef_z;

        //Measurement function
        gekf->z[0] = gekf->accelerometer[0];
        gekf->z[1] = gekf->accelerometer[1];
        gekf->z[2] = gekf->accelerometer[2];
        gekf->z[3] = gekf->gyro[0];
        gekf->z[4] = gekf->gyro[1];
        gekf->z[5] = gekf->gyro[2];
        gekf->z[6] = gekf->magneto[0];
        gekf->z[7] = gekf->magneto[1];
        gekf->z[8] = gekf->magneto[2];
        gekf->z[9] = gekf->gps[0];
        gekf->z[10] = gekf->gps[1];
        gekf->z[11] = gekf->gps[2];

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

        // Calculate initial attitude using fused magneto-accelo readings
        get_init_attitude(gekf,ground_quat);

        int gekf_has_converged = check_gekf_convergence(gekf);

        // Check for arming signal
        
        //Wait for signal to start sensor calibration.
        char signal_received[] = "NO";
        while (1){
            signal_received = ;//TODO: HAL and receiving through UART

            if (signal_received == "GO"){
            break;
            }
        }

        //TODO: Initialize sensors
        //TODO: Write an EKF to do the sensor calibration.

        //Ground calibration is now complete. Send Xbee signal to ground station that calibration is complete and rocket is ready to be launched.
        char message_for_launch_readiness[] = "GOFORLAUNCH";
        //TODO: Line of code that transmits through HAL UART to controls MCU that vehicle is launch ready.


        // Update SerialData packet
        serial_data->state = STATE_MACHINE;
        serial_data->posx = 0.0;
        serial_data->posy = 0.0;
        serial_data->posz = 0.0;
        serial_data->velx = 0.0;
        serial_data->vely = 0.0;
        serial_data->velz = 0.0;
        serial_data->q0 = ground_quat.pData[0];
        serial_data->q1 = ground_quat.pData[1];
        serial_data->q2 = ground_quat.pData[2];
        serial_data->q3 = ground_quat.pData[3];
        serial_data->wx = 0.0;
        serial_data->wy = 0.0;
        serial_data->wz = 0.0;
        serial_data->t = GlobalTime;

        // Update LoggedData packet
        logged_data->state = STATE_MACHINE;
        logged_data->posx = 0.0;
        logged_data->posy = 0.0;
        logged_data->posz = 0.0;
        logged_data->velx = 0.0;
        logged_data->vely = 0.0;
        logged_data->velz = 0.0;
        logged_data->q0 = ground_quat.pData[0];
        logged_data->q1 = ground_quat.pData[1];
        logged_data->q2 = ground_quat.pData[2];
        logged_data->q3 = ground_quat.pData[3];
        logged_data->wx = 0.0;
        logged_data->wy = 0.0;
        logged_data->wz = 0.0;
        logged_data->t = GlobalTime;
        logged_data->accelerometerx = gekf->accelerometer[0];
        logged_data->accelerometery = gekf->accelerometer[1];
        logged_data->accelerometerz = gekf->accelerometer[2];
        logged_data->gyrox = gekf->gyro[0];
        logged_data->gyroy = gekf->gyro[1];
        logged_data->gyroz = gekf->gyro[2];
        logged_data->magnetox = gekf->magneto[0];
        logged_data->magnetoy = gekf->magneto[1];
        logged_data->magnetoz = gekf->magneto[2];
        logged_data->gpsx = gekf->gps[0];
        logged_data->gpsy = gekf->gps[1];
        logged_data->gpsz = gekf->gps[2];
        //TODO log to flash chip/sd card

        // State transition conditions
        if (gekf_has_converged && signal_received) {

            // Upload sensor compensations
            sensor_comps->accelo_bias_x = gekf->x_n.pData[0];
            sensor_comps->accelo_bias_y = gekf->x_n.pData[1];
            sensor_comps->accelo_bias_z = gekf->x_n.pData[2];
            sensor_comps->accelo_bias_rate_x = gekf->x_n.pData[3];
            sensor_comps->accelo_bias_rate_y = gekf->x_n.pData[4];
            sensor_comps->accelo_bias_rate_z = gekf->x_n.pData[5];
            sensor_comps->gyro_bias_x = gekf->x_n.pData[6];
            sensor_comps->gyro_bias_y = gekf->x_n.pData[7];
            sensor_comps->gyro_bias_z = gekf->x_n.pData[8];
            sensor_comps->gyro_bias_rate_x = gekf->x_n.pData[9];
            sensor_comps->gyro_bias_rate_y = gekf->x_n.pData[10];
            sensor_comps->gyro_bias_rate_z = gekf->x_n.pData[11];
            sensor_comps->baro_offset = gekf->x_n.pData[12];
            sensor_comps->gps_offset_x = gekf->x_n.pData[13];
            sensor_comps->gps_offset_y = gekf->x_n.pData[14];
            sensor_comps->gps_offset_z = gekf->x_n.pData[15];

            // Switch states
            STATE_MACHINE = FASTSACENT;
            first_iter = 0;
        }

    }





}