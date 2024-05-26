/**
 * @file SlowAscent.h
 * @author Patrick Barry
 * @brief Header file for slow ascent state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef __SLOWASCENT_H__
#define __SLOWASCENT_H__

float *run_slow_ascent(ExtKalmanFilter *ekf, rocket_attitude *rocket_atd, float *GPS_data, float *accel_data, float *gyro_data);


#endif