#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include "altitude_kalman_filter.h"
#include "arm_math.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct Vector3 {
    float32_t x;
    float32_t y;
    float32_t z;
} vector3_t;

typedef struct BarometerPacketReceive {
    float32_t barometer_altitude;
    bool barometer_valid;
} barometer_packet_receive_t;


typedef struct ImuPacketReceive {
    float32_t acceleration[3];
    float32_t angular_velocity[3];
    uint64_t timestep;  // Milliseconds on the IMU sample clock.
} imu_packet_receive_t;


typedef struct StateEstimate {
    float32_t orientation[4];
    vector3_t velocity;
    vector3_t position;
    vector3_t linear_acceleration;
    uint64_t last_imu_timestep;
    bool have_last_imu_timestep;
} state_estimate_t;

void state_estimation_init(
    state_estimate_t *state_estimate,
    altitude_kalman_filter_t *altitude_filter
);

bool process_imu_packet(
    state_estimate_t *state_estimate,
    altitude_kalman_filter_t *altitude_filter,
    const imu_packet_receive_t *packet
);

void process_barometer_packet(
    state_estimate_t *state_estimate,
    altitude_kalman_filter_t *altitude_filter,
    const barometer_packet_receive_t *packet
);

#endif
