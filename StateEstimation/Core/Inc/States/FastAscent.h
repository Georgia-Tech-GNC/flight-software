/**
 * @file FastAscent.h
 * @author Patrick Barry
 * @brief Header file for fast ascent state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef __FASTASCENT_H__
#define __FASTASCENT_H__

float *run_fast_ascent(ExtKalmanFilter *ekf, rocket_attitude *rocket_atd, float *GPS_data, float *accel_data, float *gyro_data);


#endif