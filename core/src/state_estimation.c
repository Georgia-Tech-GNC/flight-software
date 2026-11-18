#include "state_estimation.h"
#include <math.h>
#include <string.h>

#define TIMESTEP_MILLISECONDS_TO_SECONDS 0.001f

static void normalize_quaternion(float32_t quaternion[4]) {
    float32_t norm;

    arm_quaternion_norm_f32(quaternion, &norm, 1);
    if (norm <= 0.0f) {
        quaternion[0] = 1.0f;
        quaternion[1] = 0.0f;
        quaternion[2] = 0.0f;
        quaternion[3] = 0.0f;
        return;
    }

    float32_t normalized[4];
    arm_quaternion_normalize_f32(quaternion, normalized, 1);
    (void)memcpy(quaternion, normalized, sizeof(normalized));
}

static vector3_t rotate_vector(
    const float32_t orientation[4],
    vector3_t vector
) {
    float32_t rotation_data[9];
    float32_t vector_data[3] = {vector.x, vector.y, vector.z};
    float32_t rotated_data[3];
    arm_matrix_instance_f32 rotation_matrix;

    arm_quaternion2rotation_f32(orientation, rotation_data, 1);
    arm_mat_init_f32(&rotation_matrix, 3, 3, rotation_data);
    arm_mat_vec_mult_f32(&rotation_matrix, vector_data, rotated_data);

    return (vector3_t){
        rotated_data[0],
        rotated_data[1],
        rotated_data[2],
    };
}

void state_estimation_init(
    state_estimate_t *state_estimate,
    altitude_kalman_filter_t *altitude_filter
) {
    *state_estimate = (state_estimate_t){.orientation = {1.0f}};
    *altitude_filter = (altitude_kalman_filter_t){0};
}

static void update_orientation_and_acceleration(
    state_estimate_t *state_estimate,
    const imu_packet_receive_t *packet,
    float32_t dt_seconds
) {
    vector3_t body_acceleration = {
        packet->acceleration[0],
        packet->acceleration[1],
        packet->acceleration[2],
    };
    float32_t angular_rate[4] = {
        0.0f,
        packet->angular_velocity[0],
        packet->angular_velocity[1],
        packet->angular_velocity[2],
    };
    float32_t derivative[4];
    arm_quaternion_product_single_f32(
        state_estimate->orientation,
        angular_rate,
        derivative
    );
    for (size_t index = 0; index < 4; index++) {
        state_estimate->orientation[index] +=
            0.5f * derivative[index] * dt_seconds;
    }
    normalize_quaternion(state_estimate->orientation);

    vector3_t world_acceleration =
        rotate_vector(state_estimate->orientation, body_acceleration);
    state_estimate->linear_acceleration = world_acceleration;
}

bool process_imu_packet(
    state_estimate_t *state_estimate,
    altitude_kalman_filter_t *altitude_filter,
    const imu_packet_receive_t *packet
) {
    if (!state_estimate->have_last_imu_timestep) {
        state_estimate->last_imu_timestep = packet->timestep;
        state_estimate->have_last_imu_timestep = true;
        update_orientation_and_acceleration(state_estimate, packet, 0.0f);
        return true;
    }
    if (packet->timestep <= state_estimate->last_imu_timestep) {
        return false;
    }
    float32_t dt_seconds =
        (float32_t)(packet->timestep - state_estimate->last_imu_timestep) *
        TIMESTEP_MILLISECONDS_TO_SECONDS;
    state_estimate->last_imu_timestep = packet->timestep;

    update_orientation_and_acceleration(state_estimate, packet, dt_seconds);

    vector3_t calculated_velocity = {
        state_estimate->linear_acceleration.x * dt_seconds + state_estimate->velocity.x,
        state_estimate->linear_acceleration.y * dt_seconds + state_estimate->velocity.y,
        state_estimate->linear_acceleration.z * dt_seconds + state_estimate->velocity.z,
    };
    state_estimate->position.x +=
        state_estimate->velocity.x * dt_seconds +
        0.5f * state_estimate->linear_acceleration.x * dt_seconds * dt_seconds;
    state_estimate->position.y +=
        state_estimate->velocity.y * dt_seconds +
        0.5f * state_estimate->linear_acceleration.y * dt_seconds * dt_seconds;
    state_estimate->velocity.x = calculated_velocity.x;
    state_estimate->velocity.y = calculated_velocity.y;

    if (altitude_filter->initialized) {
        altitude_kalman_filter_predict(
            altitude_filter,
            state_estimate->linear_acceleration.z,
            dt_seconds
        );
        state_estimate->position.z = altitude_filter->altitude;
        state_estimate->velocity.z = altitude_filter->vertical_velocity;
    } else {
        state_estimate->position.z +=
            state_estimate->velocity.z * dt_seconds +
            0.5f * state_estimate->linear_acceleration.z * dt_seconds * dt_seconds;
        state_estimate->velocity.z = calculated_velocity.z;
    }
    return true;
}

void process_barometer_packet(
    state_estimate_t *state_estimate,
    altitude_kalman_filter_t *altitude_filter,
    const barometer_packet_receive_t *packet
) {
    if (!packet->barometer_valid || !isfinite(packet->barometer_altitude)) {
        return;
    }

    if (!altitude_filter->initialized) {
        // Preserve the IMU velocity accumulated before the first altitude fix.
        altitude_kalman_filter_init(altitude_filter, packet->barometer_altitude);
        altitude_filter->vertical_velocity = state_estimate->velocity.z;
    } else {
        altitude_kalman_filter_update(altitude_filter, packet->barometer_altitude);
    }
    state_estimate->position.z = altitude_filter->altitude;
    state_estimate->velocity.z = altitude_filter->vertical_velocity;
}
