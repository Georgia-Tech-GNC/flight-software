/**
 * @file attitude.c
 * @author Patrick Barry, Ishan Swali
 * @brief Source file for in-flight EKF for position, velocity
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
#include "ekf.h"

void initialize_ekf(ekf *ekf){

    ekf->time_step = 0.02; //TODO: implement variable time step
    ekf->x_n = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ekf->u = 0;
    ekf->G = 0;
    ekf->z = {0.0, 0.0, 0.0};

    ekf->x_prev = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ekf->P_prev = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    ekf->n_next = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ekf->P_next = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    ekf->K_n = {{0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0}};

    ekf->f = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ekf->dfdx = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    ekf->h = {0.0, 0.0, 0.0};
    ekf->dhdx = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    ekf->P_n = {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};

    ekf->Q = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 2.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 2.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0, 0.0, 2.0}}; 

    ekf->R = {{1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}}; //Dagonal should be GPS stdev squared

    ekf->gps = {0.0, 0.0, 0.0};
    ekf->accelerometer = {0.0, 0.0, 0.0};
    ekf->barometer = 0.0;

    ekf->prev_time_millis = currentTimeMillis();
}

int64_t currentTimeMillis(){
    struct timeval time;
    gettimeofday(&time, NULL);
    int64_t s1 = (int64_t)(time.tv_sec) * 1000;
    int64_t s2 = (time.tv_usec / 1000);
    return s1 + s2;
}

void update_time_step(ekf *ekf){

    int64_t curr_time_millis = currentTimeMillis();
    float dt_millis = (float)(curr_time_millis - ekf->prev_time_millis);
    ekf->time_step = dt_millis / 1000;
    ekf->prev_time_millis = curr_time_millis;

}

void observation_function(ekf *ekf){
    float h1 = ekf->x_n[0]; //x position
    float h2 = ekf->x_n[2]; //y position
    float h3 = ekf->x_n[4]; //z position

    ekf->h[0] = h1;
    ekf->h[1] = h2;
    ekf->h[2] = h3; 

}

void observation_jacobian(ekf *ekf){
    //x position, velocity
    ekf->dfdx[0][0] = 1.0;
    ekf->dfdx[0][1] = ekf->time_step;
    ekf->dfdx[1][0] = 0.0;
    ekf->dfdx[1][1] = 1.0;

    //y position, velocity
    ekf->dfdx[2][2] = 1.0;
    ekf->dfdx[2][3] = ekf->time_step;
    ekf->dfdx[3][2] = 0.0;
    ekf->dfdx[3][3] = 1.0;

    //z position, velocity
    ekf->dfdx[4][4] = 1.0;
    ekf->dfdx[4][5] = ekf->time_step;
    ekf->dfdx[5][4] = 0.0;
    ekf->dfdx[5][5] = 1.0;
}

void kalman_gain(ekf *ekf){
    double *temp1 = malloc(nx * nz * sizeof(float));
    double *temp2 = malloc(nx * nz * sizeof(float));
    double *temp3 = malloc(nx * nz * sizeof(float));

    for (int i = 0; i < nx; i++){
        for (int j = 0; j < nz; j++){
            temp1[j*nx + i] = ekf->dhdx[i*nz + j];
        }
    }

    cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, nz, nx, nx, 1.0, ekf->dhdx, nx, ekf->P_prev, nx, 0.0, temp2, nx);

    cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, nz, nz, nx, 1.0, temp2, nx, temp1, nx, 0.0, temp3, nz);

    for (int i = 0; i < nz; i++) {
        for (int j = 0; j < nz; j++) {
            temp3[i * nz + j] += ekf->R[i * nz + j];
        }
    }

    int ipiv[nz];
    int info = LAPACKE_dgetrf(LAPACK_ROW_MAJOR, nz, nz, temp3, nz, ipiv);
    if (info == 0) {
        info = LAPACKE_dgetri(LAPACK_ROW_MAJOR, nz, temp3, nz, ipiv);
        if (info == 0) {
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, nx, nz, nz, 1.0, ekf->P_prev, nx, temp1, nx, 0.0, temp2, nz);
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, nx, nz, nz, 1.0, temp2, nz, temp3, nz, 0.0, K_n, nz);
        } else {
            printf("Error: LAPACKE_dgetri failed\n");
        }
    } else {
        printf("Error: LAPACKE_dgetrf failed\n");
    }

    free(temp1);
    free(temp2);
    free(temp3);
}

void update_state(ekf *ekf){

    for (int i = 0; i < nx; i++){
        ekf->x_n[i] = ekf->x_n[i] + ekf->K_n[i]*(ekf->z[i] - ekf->h[i]);
    }
}

void update_covariance(ekf *ekf){
    // Temporary variables for matrix operations
    double *P_n_temp = (double *)malloc(ekf->nx * ekf->nx * sizeof(double));
    double *temp1 = (double *)malloc(ekf->nx * ekf->nx * sizeof(double));
    double *temp2 = (double *)malloc(ekf->nx * ekf->nx * sizeof(double));
    
    // Compute (eye(ekf.nx) - ekf.K_n * ekf.dhdx)
    for (int i = 0; i < ekf->nx; i++) {
        for (int j = 0; j < ekf->nx; j++) {
            temp1[i * ekf->nx + j] = (i == j) ? 1.0 : 0.0;
            for (int k = 0; k < ekf->nx; k++) {
                temp1[i * ekf->nx + j] -= ekf->K_n[i * ekf->nx + k] * ekf->dhdx[k * ekf->nx + j];
            }
        }
    }
    
    // Compute (eye(ekf.nx) - ekf.K_n * ekf.dhdx)' and store it in temp2
    for (int i = 0; i < ekf->nx; i++) {
        for (int j = 0; j < ekf->nx; j++) {
            temp2[i * ekf->nx + j] = temp1[j * ekf->nx + i];
        }
    }
    
    // Compute P_n_temp = (eye(ekf.nx) - ekf.K_n * ekf.dhdx) * ekf.P_prev
    for (int i = 0; i < ekf->nx; i++) {
        for (int j = 0; j < ekf->nx; j++) {
            P_n_temp[i * ekf->nx + j] = 0.0;
            for (int k = 0; k < ekf->nx; k++) {
                P_n_temp[i * ekf->nx + j] += temp1[i * ekf->nx + k] * ekf->P_prev[k * ekf->nx + j];
            }
        }
    }
    
    // Compute P_n_temp = (eye(ekf.nx) - ekf.K_n * ekf.dhdx) * ekf.P_prev * (eye(ekf.nx) - ekf.K_n * ekf.dhdx)'
    for (int i = 0; i < ekf->nx; i++) {
        for (int j = 0; j < ekf->nx; j++) {
            ekf->P_n[i * ekf->nx + j] = 0.0;
            for (int k = 0; k < ekf->nx; k++) {
                ekf->P_n[i * ekf->nx + j] += P_n_temp[i * ekf->nx + k] * temp2[k * ekf->nx + j];
            }
        }
    }

    // Add ekf.K_n * ekf.R * ekf.K_n' to P_n
    for (int i = 0; i < ekf->nx; i++) {
        for (int j = 0; j < ekf->nx; j++) {
            temp1[i * ekf->nx + j] = 0.0;
            for (int k = 0; k < ekf->nx; k++) {
                temp1[i * ekf->nx + j] += ekf->K_n[i * ekf->nx + k] * ekf->R[k * ekf->nx + j];
            }
        }
    }
    
    for (int i = 0; i < ekf->nx; i++) {
        for (int j = 0; j < ekf->nx; j++) {
            ekf->P_n[i * ekf->nx + j] += temp1[i * ekf->nx + j];
        }
    }
    
    free(P_n_temp);
    free(temp1);
    free(temp2);
}

void state_transition_function(ekf *ekf){

    //x
    float f1 = ekf->x_n[0] + ekf->x_n[1]*ekf->time_step + 0.5*(ekf->time_step*ekf->time_step)*ekf->accelerometer[0];
    float f2 = ekf->x_n[1] + ekf->accelerometer[0]*ekf->time_step;

    //y
    float f3 = ekf->x_n[2] + ekf->x_n[3]*ekf->time_step + 0.5*(ekf->time_step*ekf->time_step)*ekf->accelerometer[1];
    float f4 = ekf->x_n[3] + ekf->accelerometer[1]*ekf->time_step;

    //z
    float f5 = ekf->x_n[4] + ekf->x_n[5]*ekf->time_step + 0.5*(ekf->time_step*ekf->time_step)*ekf->accelerometer[2];
    float f6 = ekf->x_n[5] + ekf->accelerometer[2]*ekf->time_step;

    ekf->f[0] = f1;
    ekf->f[1] = f2;
    ekf->f[2] = f3;
    ekf->f[3] = f4;
    ekf->f[4] = f5;
    ekf->f[5] = f6;
}

void state_transition_jacobian(ekf *ekf){

    ekf->dfdx[0][0] = 1.0;
    ekf->dfdx[0][1] = ekf->time_step;
    ekf->dfdx[1][0] = 0.0;
    ekf->dfdx[1][1] = 1.0;

    ekf->dfdx[2][2] = 1.0;
    ekf->dfdx[2][3] = ekf->time_step;
    ekf->dfdx[3][2] = 0.0;
    ekf->dfdx[3][3] = 1.0;

    ekf->dfdx[4][4] = 1.0;
    ekf->dfdx[4][5] = ekf->time_step;
    ekf->dfdx[5][4] = 0.0;
    ekf->dfdx[5][5] = 1.0;
}

void predict_state(ekf *ekf){

    for(int i = 0; i < nx; i++){
        ekf.x_next[i] = ekf->f[i] + ekf->G*ekf->u;
    }
}

void predict_covariance(ekf *ekf){

    // Temporary variables for matrix operations
    double *temp1 = (double *)malloc(ekf->nx * ekf->nx * sizeof(double));
    double *temp2 = (double *)malloc(ekf->nx * ekf->nx * sizeof(double));
    
    // Compute ekf.dfdx * ekf.P_n
    for (int i = 0; i < ekf->nx; i++) {
        for (int j = 0; j < ekf->nx; j++) {
            temp1[i * ekf->nx + j] = 0.0;
            for (int k = 0; k < ekf->nx; k++) {
                temp1[i * ekf->nx + j] += ekf->dfdx[i * ekf->nx + k] * ekf->P_n[k * ekf->nx + j];
            }
        }
    }
    
    // Compute ekf.dfdx' and store it in temp2
    for (int i = 0; i < ekf->nx; i++) {
        for (int j = 0; j < ekf->nx; j++) {
            temp2[i * ekf->nx + j] = ekf->dfdx[j * ekf->nx + i];
        }
    }
    
    // Compute P_next = ekf.dfdx * ekf.P_n * ekf.dfdx' + ekf.Q
    for (int i = 0; i < ekf->nx; i++) {
        for (int j = 0; j < ekf->nx; j++) {
            ekf->P_next[i * ekf->nx + j] = 0.0;
            for (int k = 0; k < ekf->nx; k++) {
                ekf->P_next[i * ekf->nx + j] += temp1[i * ekf->nx + k] * temp2[k * ekf->nx + j];
            }
            ekf->P_next[i * ekf->nx + j] += ekf->Q[i * ekf->nx + j];
        }
    }

    free(temp1);
    free(temp2);
}

void correct_accelerometer_coriolis(ekf *ekf, float* com_to_imu, float wx, float wy, float wz, float wx_dot, float wy_dot, float wz_dot){
    
    https://physics.stackexchange.com/questions/222947/calculating-acceleration-offset-by-center-of-gravity-c-g

    //com_to_imu[0] is x distance from center of mass to imu in meters
    //com_to_imu[1] is y distance from center of mass to imu in meters
    //com_to_imu[2] is z distance from center of mass to imu in meters

    //Correct x-accel
    ekf->accelerometer[0] = ekf->accelerometer[0] + (-(wy*wy) - (wz*wz))*com_to_imu[0] + (wx*wy - wz_dot*wz_dot)(com_to_imu[1]) + (wx*wz+wy_dot)*com_to_imu[2];

    //Correct y-accel
    ekf->accelerometer[1] = ekf->accelerometer[1] + (wx*wy+wz_dot)*com_to_imu[0] + (-(wx*wx) - (wz*wz))*com_to_imu[1] + (wy*wz - wx_dot)*com_to_imu[2];

    //Correct z-accel
    ekf->accelerometer[2] = ekf->accelerometer[2] + (wx*wz-wy_dot)*com_to_imu[0] + (wy*wz+wx_dot)*com_to_imu[1] + (-(wx*wx) - (wy*wy))*com_to_imu[2];
}

void acknowledge_time_passed(ekf *ekf){

    ekf->x_prev = ekf->x_next;
    ekf->P_prev = ekf->P_next;

}

float *run_ekf(ekf *ekf, float *com_to_imu, float wx, float wy, float wz, float wx_dot, float wy_dot, float wz_dot, float *GPS_sensor, float *IMU_sensor){

    //GPS_sensor: 3-element array with position relative to starting position
    //IMU_sensor: 3-element array with x, y, and z accelerations

    update_time_step(ekf);

    ekf->gps = GPS_sensor;
    ekf->accelerometer = IMU_sensor;

    //Measurement function
    ekf->z = ekf->gps;
    correct_accelerometer_coriolis(ekf, com_to_imu, wx, wy, wz, wx_dot, wy_dot, wz_dot);

    //Update
    observation_function(ekf);
    observation_jacobian(ekf);
    kalman_gain(ekf);
    update_state(ekf);
    update_covariance(ekf);

    //Predict
    state_transition_function(ekf);
    state_transition_jacobian(ekf);
    predict_state(ekf);
    predict_covariance(ekf);

    acknowledge_time_passed(ekf);
    return ekf->x_n;
}

