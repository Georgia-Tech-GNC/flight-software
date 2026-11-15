#include "altitude_kalman_filter.h"

#define BAROMETER_MEASUREMENT_VARIANCE 4.0f
#define ACCELERATION_NOISE_VARIANCE 4.0f

void altitude_kalman_filter_init(
    altitude_kalman_filter_t *filter,
    float32_t altitude_meters
) {
    filter->altitude = altitude_meters;
    filter->vertical_velocity = 0.0f;
    filter->covariance[0][0] = BAROMETER_MEASUREMENT_VARIANCE;
    filter->covariance[0][1] = 0.0f;
    filter->covariance[1][0] = 0.0f;
    filter->covariance[1][1] = 10.0f;
    filter->initialized = true;
}

void altitude_kalman_filter_predict(
    altitude_kalman_filter_t *filter,
    float32_t acceleration_z,
    float32_t dt_seconds
) {
    float32_t dt_squared = dt_seconds * dt_seconds;
    float32_t dt_cubed = dt_squared * dt_seconds;
    float32_t dt_fourth = dt_cubed * dt_seconds;
    float32_t old_p00 = filter->covariance[0][0];
    float32_t old_p01 = filter->covariance[0][1];
    float32_t old_p10 = filter->covariance[1][0];
    float32_t old_p11 = filter->covariance[1][1];

    filter->altitude +=
        filter->vertical_velocity * dt_seconds +
        0.5f * acceleration_z * dt_squared;
    filter->vertical_velocity += acceleration_z * dt_seconds;
    filter->covariance[0][0] =
        old_p00 + dt_seconds * (old_p01 + old_p10) +
        dt_squared * old_p11 +
        ACCELERATION_NOISE_VARIANCE * dt_fourth * 0.25f;
    filter->covariance[0][1] =
        old_p01 + dt_seconds * old_p11 +
        ACCELERATION_NOISE_VARIANCE * dt_cubed * 0.5f;
    filter->covariance[1][0] =
        old_p10 + dt_seconds * old_p11 +
        ACCELERATION_NOISE_VARIANCE * dt_cubed * 0.5f;
    filter->covariance[1][1] =
        old_p11 + ACCELERATION_NOISE_VARIANCE * dt_squared;
}

void altitude_kalman_filter_update(
    altitude_kalman_filter_t *filter,
    float32_t altitude_meters
) {
    if (!filter->initialized) {
        altitude_kalman_filter_init(filter, altitude_meters);
        return;
    }

    float32_t innovation = altitude_meters - filter->altitude;
    float32_t innovation_variance =
        filter->covariance[0][0] + BAROMETER_MEASUREMENT_VARIANCE;
    float32_t gain_altitude =
        filter->covariance[0][0] / innovation_variance;
    float32_t gain_velocity =
        filter->covariance[1][0] / innovation_variance;
    float32_t old_p00 = filter->covariance[0][0];
    float32_t old_p01 = filter->covariance[0][1];

    filter->altitude += gain_altitude * innovation;
    filter->vertical_velocity += gain_velocity * innovation;
    filter->covariance[0][0] = (1.0f - gain_altitude) * old_p00;
    filter->covariance[0][1] = (1.0f - gain_altitude) * old_p01;
    filter->covariance[1][0] =
        filter->covariance[1][0] - gain_velocity * old_p00;
    filter->covariance[1][1] =
        filter->covariance[1][1] - gain_velocity * old_p01;
}
