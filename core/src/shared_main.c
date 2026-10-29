/** @file shared_main.c
 * @brief General entry point for all targets
 */

#include "port.h"
#include "shared_main.h"
#include "FreeRTOS.h"
#include "task.h"

#include "message_buffer.h"
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MainTaskStackSize 4096
#define MainTaskPriority (tskIDLE_PRIORITY + 1)
#define MessageBufferSize 256
#define MainTaskPeriodTicks pdMS_TO_TICKS(333)

#define GRAVITY_MPS2 9.81f
#define LAUNCH_ACCELERATION_MPS2 5.0f
#define BAROMETER_MEASUREMENT_VARIANCE 4.0f
#define ACCELERATION_NOISE_VARIANCE 4.0f

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

typedef struct {
    float x;
    float y;
    float z;
} Vector3;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct {
    Quaternion orientation;
    Vector3 velocity;
    Vector3 position;
    Vector3 linear_acceleration;
} State_Estimate;

typedef struct {
    float altitude;
    float vertical_velocity;
    float covariance[2][2];
    bool initialized;
} Altitude_Kalman_Filter;

static State_Estimate state_estimate = {
    .orientation = {1.0f, 0.0f, 0.0f, 0.0f},
};
static Altitude_Kalman_Filter altitude_filter;

typedef enum {
    ARM,
    DISARM,
    UNCONTROLL,
    PYRO_DEPLOY,
    ESTOP
} Telemetry_Command;

FSM_State FSM = GROUND;

static uint64_t get_time_from_startup(void) {
    return pdTICKS_TO_MS(xTaskGetTickCount());
}

static Quaternion quaternion_normalize(Quaternion quaternion) {
    float magnitude = sqrtf(quaternion.w * quaternion.w +
                            quaternion.x * quaternion.x +
                            quaternion.y * quaternion.y +
                            quaternion.z * quaternion.z);
    if (magnitude <= 0.0f) {
        return (Quaternion){1.0f, 0.0f, 0.0f, 0.0f};
    }

    return (Quaternion){
        quaternion.w / magnitude,
        quaternion.x / magnitude,
        quaternion.y / magnitude,
        quaternion.z / magnitude,
    };
}

static Quaternion quaternion_multiply(Quaternion lhs, Quaternion rhs) {
    return (Quaternion){
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    };
}

static Vector3 rotate_vector(Quaternion orientation, Vector3 vector) {
    Quaternion vector_quaternion = {0.0f, vector.x, vector.y, vector.z};
    Quaternion inverse = {
        orientation.w, -orientation.x, -orientation.y, -orientation.z
    };
    Quaternion rotated = quaternion_multiply(
        quaternion_multiply(orientation, vector_quaternion), inverse);
    return (Vector3){rotated.x, rotated.y, rotated.z};
}

static void altitude_filter_predict(float acceleration_z, float dt_seconds) {
    float dt_squared = dt_seconds * dt_seconds;
    float dt_cubed = dt_squared * dt_seconds;
    float dt_fourth = dt_cubed * dt_seconds;
    float old_p00 = altitude_filter.covariance[0][0];
    float old_p01 = altitude_filter.covariance[0][1];
    float old_p10 = altitude_filter.covariance[1][0];
    float old_p11 = altitude_filter.covariance[1][1];

    altitude_filter.altitude +=
        altitude_filter.vertical_velocity * dt_seconds +
        0.5f * acceleration_z * dt_squared;
    altitude_filter.vertical_velocity += acceleration_z * dt_seconds;
    altitude_filter.covariance[0][0] =
        old_p00 + dt_seconds * (old_p01 + old_p10) +
        dt_squared * old_p11 +
        ACCELERATION_NOISE_VARIANCE * dt_fourth * 0.25f;
    altitude_filter.covariance[0][1] =
        old_p01 + dt_seconds * old_p11 +
        ACCELERATION_NOISE_VARIANCE * dt_cubed * 0.5f;
    altitude_filter.covariance[1][0] =
        old_p10 + dt_seconds * old_p11 +
        ACCELERATION_NOISE_VARIANCE * dt_cubed * 0.5f;
    altitude_filter.covariance[1][1] =
        old_p11 + ACCELERATION_NOISE_VARIANCE * dt_squared;
}

static void update_altitude_filter(float altitude_meters) {
    if (!altitude_filter.initialized) {
        altitude_filter.altitude = altitude_meters;
        altitude_filter.vertical_velocity = 0.0f;
        altitude_filter.covariance[0][0] = BAROMETER_MEASUREMENT_VARIANCE;
        altitude_filter.covariance[0][1] = 0.0f;
        altitude_filter.covariance[1][0] = 0.0f;
        altitude_filter.covariance[1][1] = 10.0f;
        altitude_filter.initialized = true;
    } else {
        float innovation = altitude_meters - altitude_filter.altitude;
        float innovation_variance =
            altitude_filter.covariance[0][0] +
            BAROMETER_MEASUREMENT_VARIANCE;
        float gain_altitude =
            altitude_filter.covariance[0][0] / innovation_variance;
        float gain_velocity =
            altitude_filter.covariance[1][0] / innovation_variance;
        float old_p00 = altitude_filter.covariance[0][0];
        float old_p01 = altitude_filter.covariance[0][1];

        altitude_filter.altitude += gain_altitude * innovation;
        altitude_filter.vertical_velocity += gain_velocity * innovation;
        altitude_filter.covariance[0][0] =
            (1.0f - gain_altitude) * old_p00;
        altitude_filter.covariance[0][1] =
            (1.0f - gain_altitude) * old_p01;
        altitude_filter.covariance[1][0] =
            altitude_filter.covariance[1][0] -
            gain_velocity * old_p00;
        altitude_filter.covariance[1][1] =
            altitude_filter.covariance[1][1] -
            gain_velocity * old_p01;
    }

    state_estimate.position.z = altitude_filter.altitude;
    state_estimate.velocity.z = altitude_filter.vertical_velocity;
}

static void process_imu_packet(const Imu_Packet_Receive *packet) {
    if (packet->dt_seconds <= 0.0f) {
        return;
    }

    Vector3 body_acceleration = {
        packet->acceleration[0],
        packet->acceleration[1],
        packet->acceleration[2],
    };
    Quaternion angular_rate = {
        0.0f,
        packet->angular_velocity[0],
        packet->angular_velocity[1],
        packet->angular_velocity[2],
    };
    Quaternion derivative = quaternion_multiply(
        state_estimate.orientation, angular_rate);

    state_estimate.orientation = quaternion_normalize((Quaternion){
        state_estimate.orientation.w + 0.5f * derivative.w *
            packet->dt_seconds,
        state_estimate.orientation.x + 0.5f * derivative.x *
            packet->dt_seconds,
        state_estimate.orientation.y + 0.5f * derivative.y *
            packet->dt_seconds,
        state_estimate.orientation.z + 0.5f * derivative.z *
            packet->dt_seconds,
    });

    Vector3 world_acceleration =
        rotate_vector(state_estimate.orientation, body_acceleration);
    world_acceleration.z -= GRAVITY_MPS2;
    state_estimate.linear_acceleration = world_acceleration;

    Vector3 imu_velocity = {
        packet->velocity[0],
        packet->velocity[1],
        packet->velocity[2],
    };
    state_estimate.position.x +=
        imu_velocity.x * packet->dt_seconds +
        0.5f * world_acceleration.x * packet->dt_seconds *
            packet->dt_seconds;
    state_estimate.position.y +=
        imu_velocity.y * packet->dt_seconds +
        0.5f * world_acceleration.y * packet->dt_seconds *
            packet->dt_seconds;
    state_estimate.velocity.x = imu_velocity.x;
    state_estimate.velocity.y = imu_velocity.y;

    if (altitude_filter.initialized) {
        altitude_filter_predict(world_acceleration.z, packet->dt_seconds);
        state_estimate.position.z = altitude_filter.altitude;
        state_estimate.velocity.z = altitude_filter.vertical_velocity;
    } else {
        state_estimate.position.z +=
            imu_velocity.z * packet->dt_seconds +
            0.5f * world_acceleration.z * packet->dt_seconds *
                packet->dt_seconds;
        state_estimate.velocity.z = imu_velocity.z;
    }

    if (packet->barometer_valid) {
        update_altitude_filter(packet->barometer_altitude);
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

static void set_fsm_state(FSM_State state) {
    if (state <= EMERGENCY_STOP) {
        FSM = state;
        last_fsm_change = xTaskGetTickCount();
    }
}

static void send_telemetry(const Imu_Packet_Receive *latest_imu) {
    Telemetry_Packet_Send packet = {
        .state = FSM,
        .time_since_startup = get_time_from_startup(),
        .orientation = {
            state_estimate.orientation.w,
            state_estimate.orientation.x,
            state_estimate.orientation.y,
            state_estimate.orientation.z,
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
        (void)memcpy(packet.imu_velocity, latest_imu->velocity, sizeof(packet.imu_velocity));
        (void)memcpy(packet.imu_acceleration, latest_imu->acceleration, sizeof(packet.imu_acceleration));
    }

    (void)xMessageBufferSend(telemetry_buffer_handle, &packet, sizeof(packet), 0);
}

static void process_commands(void) {
    Telemetry_Command command;
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

static void process_fsm(const Imu_Packet_Receive *latest_imu) {
    switch (FSM) {
        case GROUND:
            break;
        case ARMED:
            if (get_acceleration() > LAUNCH_ACCELERATION_MPS2) {
                (void)move_fsm_forward();
            }
            break;
        case FAST_ASCENT:
            if (xTaskGetTickCount() - last_fsm_change >=
                pdMS_TO_TICKS(500)) {
                (void)move_fsm_forward();
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
    TickType_t next_wake = xTaskGetTickCount();
    Imu_Packet_Receive latest_imu = {0};
    bool have_imu_sample = false;

    while (true) {
        process_commands();

        Imu_Packet_Receive imu_packet;
        while (xMessageBufferReceive(imu_buffer_handle, &imu_packet, sizeof(imu_packet), 0) == sizeof(imu_packet)) {
            process_imu_packet(&imu_packet);
            latest_imu = imu_packet;
            have_imu_sample = true;
        }

        process_fsm(have_imu_sample ? &latest_imu : NULL);
        vTaskDelayUntil(&next_wake, MainTaskPeriodTicks);
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
