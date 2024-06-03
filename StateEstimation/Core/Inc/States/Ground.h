/**
 * @file Ground.h
 * @author Albert Zheng
 * @brief Header file for ground state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef __GROUND_H__
    #define __GROUND_H__

void state_transition_ground(ExtKalmanFilter *gekf);

void state_transition_jacob_ground(ExtKalmanFilter *gekf);

void observation_ground(ExtKalmanFilter *gekf);

void observation_jacob_ground(ExtKalmanFilter *gekf);

void get_ground_attitude(ExtKalmanFilter *gekf);

void check_gekf_convergence(ExtKalmanFilter *gekf);

void GPS2ECEF(float* gps_reading, float* posn_ecef);

void run_ground(ExtKalmanFilter *gekf, Sensors *sensors, SerialData *serial_data);


#endif