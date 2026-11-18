#include "state_estimation.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "Line %d: %s\n", __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

static bool near(float actual, float expected) {
    return fabsf(actual - expected) < 0.0001f;
}

static void test_imu_timestamps(void) {
    state_estimate_t state;
    altitude_kalman_filter_t filter;
    state_estimation_init(&state, &filter);

    imu_packet_receive_t imu = {.acceleration = {2.0f, -4.0f, 6.0f}};
    CHECK(process_imu_packet(&state, &filter, &imu));
    CHECK(near(state.linear_acceleration.z, 6.0f));
    CHECK(near(state.position.z, 0.0f));

    imu.timestep = 1000;
    CHECK(process_imu_packet(&state, &filter, &imu));
    CHECK(near(state.position.x, 1.0f));
    CHECK(near(state.position.y, -2.0f));
    CHECK(near(state.position.z, 3.0f));
    CHECK(near(state.velocity.z, 6.0f));
    CHECK(!filter.initialized);

    CHECK(!process_imu_packet(&state, &filter, &imu));
    imu.timestep = 500;
    CHECK(!process_imu_packet(&state, &filter, &imu));
    CHECK(near(state.position.z, 3.0f));

    imu.timestep = 2000;
    CHECK(process_imu_packet(&state, &filter, &imu));
    CHECK(near(state.position.z, 12.0f));
    CHECK(near(state.velocity.z, 12.0f));
}

static void test_imu_rotation(void) {
    state_estimate_t state;
    altitude_kalman_filter_t filter;
    state_estimation_init(&state, &filter);

    imu_packet_receive_t imu = {
        .acceleration = {1.0f, 0.0f, 0.0f},
        .angular_velocity = {0.0f, 0.0f, 2.0f},
        .timestep = 10000,
    };
    CHECK(process_imu_packet(&state, &filter, &imu));
    CHECK(near(state.orientation[0], 1.0f));

    imu.timestep = 11000;
    CHECK(process_imu_packet(&state, &filter, &imu));
    CHECK(near(state.orientation[0], sqrtf(0.5f)));
    CHECK(near(state.orientation[3], sqrtf(0.5f)));
    CHECK(near(state.linear_acceleration.x, 0.0f));
    CHECK(near(state.linear_acceleration.y, 1.0f));
    CHECK(near(state.position.y, 0.5f));
}

static void test_barometer_prediction_and_correction(void) {
    state_estimate_t state;
    altitude_kalman_filter_t filter;
    state_estimation_init(&state, &filter);

    barometer_packet_receive_t barometer = {.barometer_altitude = 100.0f};
    process_barometer_packet(&state, &filter, &barometer);
    CHECK(!filter.initialized);

    barometer.barometer_valid = true;
    process_barometer_packet(&state, &filter, &barometer);
    CHECK(near(state.position.z, 100.0f));
    CHECK(!state.have_last_imu_timestep);

    imu_packet_receive_t imu = {.acceleration = {0.0f, 0.0f, 2.0f}};
    CHECK(process_imu_packet(&state, &filter, &imu));
    imu.timestep = 1000;
    CHECK(process_imu_packet(&state, &filter, &imu));
    CHECK(near(state.position.z, 101.0f));
    CHECK(near(state.velocity.z, 2.0f));

    barometer.barometer_altitude = NAN;
    process_barometer_packet(&state, &filter, &barometer);
    CHECK(near(filter.altitude, 101.0f));
    barometer.barometer_altitude = 120.0f;
    process_barometer_packet(&state, &filter, &barometer);
    CHECK(near(state.position.z, 116.0f));
    CHECK(near(state.velocity.z, 14.0f));
    CHECK(state.last_imu_timestep == 1000);
}

static void test_first_barometer_fix_after_imu(void) {
    state_estimate_t state;
    altitude_kalman_filter_t filter;
    state_estimation_init(&state, &filter);

    imu_packet_receive_t imu = {.acceleration = {0.0f, 0.0f, 2.0f}};
    CHECK(process_imu_packet(&state, &filter, &imu));
    imu.timestep = 1000;
    CHECK(process_imu_packet(&state, &filter, &imu));

    const barometer_packet_receive_t barometer = {
        .barometer_altitude = 100.0f,
        .barometer_valid = true,
    };
    process_barometer_packet(&state, &filter, &barometer);
    CHECK(near(state.position.z, 100.0f));
    CHECK(near(state.velocity.z, 2.0f));
    CHECK(near(filter.vertical_velocity, 2.0f));
}

int main(void) {
    test_imu_timestamps();
    test_imu_rotation();
    test_barometer_prediction_and_correction();
    test_first_barometer_fix_after_imu();
    return EXIT_SUCCESS;
}
