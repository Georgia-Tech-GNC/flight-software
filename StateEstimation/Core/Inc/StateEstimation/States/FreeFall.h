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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "Ground.h"
#include "attitude.h"
#include "data_handling.h"

void run_freefall(ExtKalmanFilter *ekf, RocketAttitude *rocket_atd, Sensors *sensors, SerialData *serial_data, UART_HandleTypeDef *huart);

#endif