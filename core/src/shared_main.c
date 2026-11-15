/** @file shared_main.c
 * @brief General entry point for all targets
 */

#include "port.h"
#include "shared_main.h"
#include "altitude_kalman_filter.h"
#include "FreeRTOS.h"
#include "task.h"

#include "message_buffer.h"
#include <arm_math.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MainTaskStackSize 4096
#define MainTaskPriority (tskIDLE_PRIORITY + 1)
#define MessageBufferSize 256
#define MainTaskPeriodTicks pdMS_TO_TICKS(3)

#define GRAVITY_MPS2 9.81f
#define LAUNCH_ACCELERATION_MPS2 5.0f
#define TIMESTEP_MILLISECONDS_TO_SECONDS 0.001f

MessageBufferHandle_t command_buffer_handle;
MessageBufferHandle_t imu_buffer_handle;
MessageBufferHandle_t telemetry_buffer_handle;

static StaticMessageBuffer_t command_buffer_struct;
static uint8_t command_buffer_storage[MessageBufferSize];
static StaticMessageBuffer_t imu_buffer_struct;
static uint8_t imu_buffer_storage[MessageBufferSize];
static StaticMessageBuffer_t telemetry_buffer_struct;
static uint8_t telemetry_buffer_storage[MessageBufferSize];

StaticTask_t main_task_buffer;
StackType_t main_task_stack[MainTaskStackSize];
TickType_t last_fsm_change;



static state_estimate_t state_estimate = {
    .orientation = {1.0f, 0.0f, 0.0f, 0.0f},
};
static altitude_kalman_filter_t altitude_filter;
static uint64_t last_imu_timestep;
static bool have_last_imu_timestep;

typedef enum TelemetryCommand {
    ARM,
    DISARM,
    UNCONTROLL,
    PYRO_DEPLOY,
    ESTOP
} telemetry_command_t;

fsm_state_t FSM = GROUND;

static uint64_t get_time_from_startup(void) {
    return pdTICKS_TO_MS(xTaskGetTickCount());
}

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

static void process_imu_packet(const imu_packet_receive_t *packet) {
    if (!have_last_imu_timestep) {
        last_imu_timestep = packet->timestep;
        have_last_imu_timestep = true;
        if (packet->barometer_valid) {
            altitude_kalman_filter_update(
                &altitude_filter,
                packet->barometer_altitude
            );
            state_estimate.position.z = altitude_filter.altitude;
            state_estimate.velocity.z = altitude_filter.vertical_velocity;
        }
        return;
    }
    if (packet->timestep <= last_imu_timestep) {
        return;
    }
    float32_t dt_seconds =
        (float32_t)(packet->timestep - last_imu_timestep) *
        TIMESTEP_MILLISECONDS_TO_SECONDS;
    last_imu_timestep = packet->timestep;

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
        state_estimate.orientation,
        angular_rate,
        derivative
    );
    for (size_t index = 0; index < 4; index++) {
        state_estimate.orientation[index] +=
            0.5f * derivative[index] * dt_seconds;
    }
    normalize_quaternion(state_estimate.orientation);

    vector3_t world_acceleration =
        rotate_vector(state_estimate.orientation, body_acceleration);
    world_acceleration.z -= GRAVITY_MPS2;
    state_estimate.linear_acceleration = world_acceleration;

    vector3_t calculated_velocity = {
        world_acceleration.x * dt_seconds + state_estimate.velocity.x,
        world_acceleration.y * dt_seconds + state_estimate.velocity.y,
        world_acceleration.z * dt_seconds + state_estimate.velocity.z,
    };
    state_estimate.position.x +=
        state_estimate.velocity.x * dt_seconds +
        0.5f * world_acceleration.x * dt_seconds * dt_seconds;
    state_estimate.position.y +=
        state_estimate.velocity.y * dt_seconds +
        0.5f * world_acceleration.y * dt_seconds * dt_seconds;
    state_estimate.velocity.x = calculated_velocity.x;
    state_estimate.velocity.y = calculated_velocity.y;

    if (altitude_filter.initialized) {
        altitude_kalman_filter_predict(
            &altitude_filter,
            world_acceleration.z,
            dt_seconds
        );
        state_estimate.position.z = altitude_filter.altitude;
        state_estimate.velocity.z = altitude_filter.vertical_velocity;
    } else {
        state_estimate.position.z +=
            state_estimate.velocity.z * dt_seconds +
            0.5f * world_acceleration.z * dt_seconds * dt_seconds;
        state_estimate.velocity.z = calculated_velocity.z;
    }

    if (packet->barometer_valid) {
        altitude_kalman_filter_update(
            &altitude_filter,
            packet->barometer_altitude
        );
        state_estimate.position.z = altitude_filter.altitude;
        state_estimate.velocity.z = altitude_filter.vertical_velocity;
    }
}

static float get_acceleration(void) {
    return state_estimate.linear_acceleration.z;
}

static bool move_fsm_forward(void) {
    if (FSM < PYRO) {
        FSM++;
        last_fsm_change = xTaskGetTickCount();
        return true;
    }
    return false;
}

static void set_fsm_state(fsm_state_t state) {
    if (state <= EMERGENCY_STOP) {
        FSM = state;
        last_fsm_change = xTaskGetTickCount();
    }
}

static void send_telemetry(const imu_packet_receive_t *latest_imu) {
    telemetry_packet_send_t packet = {
        .state = FSM,
        .time_since_startup = get_time_from_startup(),
        .orientation = {
            state_estimate.orientation[0],
            state_estimate.orientation[1],
            state_estimate.orientation[2],
            state_estimate.orientation[3],
        },
        .velocity = {
            state_estimate.velocity.x,
            state_estimate.velocity.y,
            state_estimate.velocity.z,
        },
        .position = {
            state_estimate.position.x,
            state_estimate.position.y,
            state_estimate.position.z,
        },
    };

    if (latest_imu != NULL) {
        (void)memcpy(packet.imu_acceleration, latest_imu->acceleration, sizeof(packet.imu_acceleration));
        (void)memcpy(packet.imu_angular_velocity, latest_imu->angular_velocity, sizeof(packet.imu_angular_velocity));
    }

    xMessageBufferSend(telemetry_buffer_handle, &packet, sizeof(packet), 0);
}

static void process_commands(void) {
    telemetry_command_t command;
    while (xMessageBufferReceive(command_buffer_handle, &command, sizeof(command), 0) == sizeof(command)) {
        switch (command) {
            case ARM:
                set_fsm_state(ARMED);
                break;
            case DISARM:
                set_fsm_state(GROUND);
                break;
            case UNCONTROLL:
                set_fsm_state(UNCONTROLLED);
                break;
            case PYRO_DEPLOY:
                set_fsm_state(PYRO);
                break;
            case ESTOP:
                set_fsm_state(EMERGENCY_STOP);
                break;
        }
    }
}

static void process_fsm(const imu_packet_receive_t *latest_imu) {
    switch (FSM) {
        case GROUND:
            break;
        case ARMED:
            if (get_acceleration() > LAUNCH_ACCELERATION_MPS2) {
                move_fsm_forward();
            }
            break;
        case FAST_ASCENT:
            if (xTaskGetTickCount() - last_fsm_change >= pdMS_TO_TICKS(500)) {
                move_fsm_forward();
            }
            break;
        case CONTROLLED:
            break;
        case UNCONTROLLED:
            break;
        case PYRO:
            break;
        case EMERGENCY_STOP:
            break;
    }

    send_telemetry(latest_imu);
}

static void main_task(void *_params) {
    UNUSED(_params);
    imu_packet_receive_t latest_imu = {0};
    bool have_imu_sample = false;
    if (!altitude_filter.initialized) {
        altitude_kalman_filter_init(&altitude_filter,0.0f);
    }
    while (true) {
        process_commands();

        imu_packet_receive_t imu_packet;
        while (xMessageBufferReceive(imu_buffer_handle, &imu_packet, sizeof(imu_packet), MainTaskPeriodTicks) == sizeof(imu_packet)) {
            process_imu_packet(&imu_packet);
            latest_imu = imu_packet;
            have_imu_sample = true;
        }

        process_fsm(have_imu_sample ? &latest_imu : NULL);
    }
}

/** The main methods of all targets call this after peripheral initialization. */
[[noreturn]] void shared_main(void) {
    command_buffer_handle = xMessageBufferCreateStatic(
        MessageBufferSize, command_buffer_storage, &command_buffer_struct);
    imu_buffer_handle = xMessageBufferCreateStatic(
        MessageBufferSize, imu_buffer_storage, &imu_buffer_struct);
    telemetry_buffer_handle = xMessageBufferCreateStatic(
        MessageBufferSize, telemetry_buffer_storage, &telemetry_buffer_struct);

    TaskHandle_t main_task_handle = xTaskCreateStatic(
        main_task, "Main Task", MainTaskStackSize, NULL, MainTaskPriority,
        main_task_stack, &main_task_buffer);

    if (command_buffer_handle && imu_buffer_handle &&
        telemetry_buffer_handle && main_task_handle) {
        static uint8_t success_message[] =
            "Successfully created tasks, starting scheduler.\r\n";
        HAL_UART_Transmit(&debug_uart, success_message, sizeof(success_message) - 1, HAL_MAX_DELAY);
        vTaskStartScheduler();
    } else {
        static uint8_t failure_message[] = "Failed to create tasks\r\n";
        HAL_UART_Transmit(&debug_uart, failure_message, sizeof(failure_message) - 1, HAL_MAX_DELAY);
    }

    while (true) {
        __asm__ volatile ("");
    }
}
