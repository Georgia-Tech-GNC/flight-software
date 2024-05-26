/**
 * @file FreeFall.h
 * @author Patrick Barry
 * @brief Header file for free fall state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef __FREEFALL_H__
#define __FREEFALL_H__

float *run_freefall(ExtKalmanFilter *ekf, rocket_attitude *rocket_atd, float *GPS_data, float *accel_data, float *gyro_data);


#endif