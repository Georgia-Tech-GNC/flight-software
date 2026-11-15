#ifndef ALTITUDE_KALMAN_FILTER_H
#define ALTITUDE_KALMAN_FILTER_H

#include "arm_math.h"
#include <stdbool.h>

typedef struct AltitudeKalmanFilter {
    float32_t altitude;
    float32_t vertical_velocity;
    float32_t covariance[2][2];
    bool initialized;
} altitude_kalman_filter_t;

void altitude_kalman_filter_init(
    altitude_kalman_filter_t *filter,
    float32_t altitude_meters
);

void altitude_kalman_filter_predict(
    altitude_kalman_filter_t *filter,
    float32_t acceleration_z,
    float32_t dt_seconds
);

void altitude_kalman_filter_update(
    altitude_kalman_filter_t *filter,
    float32_t altitude_meters
);

#endif
