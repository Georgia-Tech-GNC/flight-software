/**
 * @file ekf.h
 * @author Patrick Barry, Ishan Swali
 * @brief This contains the function definitions for the EKF that governs position/velocity
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef __EKF_H__
#define __EKF_H__

const int nx = 6;
const int nz = 3;
const int nu = 0;
typedef struct {

    float time_step = 0.02; //TODO: change this to be accurate
    //Core variables
    float x_n[nx] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //Curent state x_n,n
    float u = 0; //Control input (nu by 1)
    float G = 0; //Control matrix G (nu by nu)

    float z[nz] = {0.0, 0.0, 0.0}; //Measurement z_n (nz by 1)

    float x_prev[nx] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //Previously predicted state x_n,n-1 (nx by 1)
    float P_prev[nx][nx] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        //Previously predicted estimate covariance P_n,n-1 (nx by nx)

    float x_next[nx] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //Predicted future state x_n,n-1 (nx by 1)
    float P_next[nx][nx] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        //Predicted future estimate covariance P_n,n-1 (nx by nx)

    //Intermediate Variables
    float K_n[nx][nz] = {{0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0}}; //Kalman gain Kn (nx by nz)

    float f[nx] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //State transition function (nx by 1)
    float dfdx[nx][nx] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; //Linearized state transition function (nx by nx)

    float h[nz] = {0.0, 0.0, 0.0}; //Observation function (nz by 1)
    float dhdx[nz][nx] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; //Linearized observation function (nz by nx)

    float P_n[nx][nx] = {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}}; //Current estimate covariance P_n,n (nx by nx)

    float Q[nx][nx] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 2.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 2.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 2.0}}; //Process noise covariance Q (nx by nx)

    float R[nz][nz] = {{1.0, 0.0, 0.0},
                        {0.0, 1.0, 0.0},
                        {0.0, 0.0, 1.0}}; //Measurement covariance R (nz by nz)
                        //Diagonal should be GPS standard deviation, squared

    float gps[nz] = {0.0, 0.0, 0.0}; //GPS reading relative to starting location (x, y, z)
    float accelerometer[nz] = {0.0, 0.0, 0.0}; //Accelerometer readings (x, y, z)
    float barometer = 0.0; //Barometer altitude reading


} ekf;

void observation_function(ekf *ekf);

void observation_jacobian(ekf *ekf);

void kalman_gain(ekf *ekf);

void update_state(ekf *ekf);

void update_covariance(ekf *ekf);

void state_transition_function(ekf *ekf);

void state_transition_jacobian(ekf *ekf);

void predict_state(ekf *ekf);

void predict_covariance(ekf *ekf);

void ackowledge_time_passed(ekf *ekf);


#endif
