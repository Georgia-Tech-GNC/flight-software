/** @file shared_main.c
 * @brief General entry point for all targets
 */

#include "port.h"
#include "shared_main.h"
#include "task.h"

#include <string.h>

#define MainTaskStackSize 4096
#define MainTaskPriority (tskIDLE_PRIORITY + 1)
#define MessageBufferSize 256
#define MainTaskPeriodTicks pdMS_TO_TICKS(3)

#define LAUNCH_ACCELERATION_MPS2 5.0f

MessageBufferHandle_t command_buffer_handle;
MessageBufferHandle_t imu_buffer_handle;
MessageBufferHandle_t barometer_buffer_handle;
MessageBufferHandle_t telemetry_buffer_handle;

static StaticMessageBuffer_t command_buffer_struct;
static uint8_t command_buffer_storage[MessageBufferSize];
static StaticMessageBuffer_t imu_buffer_struct;
static uint8_t imu_buffer_storage[MessageBufferSize];
static StaticMessageBuffer_t barometer_buffer_struct;
static uint8_t barometer_buffer_storage[MessageBufferSize];
static StaticMessageBuffer_t telemetry_buffer_struct;
static uint8_t telemetry_buffer_storage[MessageBufferSize];

StaticTask_t main_task_buffer;
StackType_t main_task_stack[MainTaskStackSize];
TickType_t last_fsm_change;



static state_estimate_t state_estimate;
static altitude_kalman_filter_t altitude_filter;

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


static float get_acceleration_z(void) {
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
            if (get_acceleration_z() > LAUNCH_ACCELERATION_MPS2) {
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
    state_estimation_init(&state_estimate, &altitude_filter);

    while (true) {
        process_commands();

        for (size_t i = 0; i < MessageBufferSize / sizeof(imu_packet_receive_t); i++) {
            imu_packet_receive_t packet;
            if (xMessageBufferReceive(imu_buffer_handle, &packet, sizeof(packet), 0) != sizeof(packet)) {
                break;
            }
            if (process_imu_packet(&state_estimate, &altitude_filter, &packet)) {
                latest_imu = packet;
                have_imu_sample = true;
            }
        }

        for (size_t i = 0; i < MessageBufferSize / sizeof(barometer_packet_receive_t); i++) {
            barometer_packet_receive_t packet;
            if (xMessageBufferReceive(barometer_buffer_handle, &packet, sizeof(packet), 0) != sizeof(packet)) {
                break;
            }
            process_barometer_packet(&state_estimate, &altitude_filter, &packet);
        }

        process_fsm(have_imu_sample ? &latest_imu : NULL);
        vTaskDelay(MainTaskPeriodTicks);
    }
}

/** The main methods of all targets call this after peripheral initialization. */
[[noreturn]] void shared_main(void) {
    command_buffer_handle = xMessageBufferCreateStatic(
        MessageBufferSize, command_buffer_storage, &command_buffer_struct);
    imu_buffer_handle = xMessageBufferCreateStatic(
        MessageBufferSize, imu_buffer_storage, &imu_buffer_struct);
    barometer_buffer_handle = xMessageBufferCreateStatic(
        MessageBufferSize, barometer_buffer_storage, &barometer_buffer_struct);
    telemetry_buffer_handle = xMessageBufferCreateStatic(
        MessageBufferSize, telemetry_buffer_storage, &telemetry_buffer_struct);

    TaskHandle_t main_task_handle = xTaskCreateStatic(
        main_task, "Main Task", MainTaskStackSize, NULL, MainTaskPriority,
        main_task_stack, &main_task_buffer);
    if (command_buffer_handle && imu_buffer_handle && barometer_buffer_handle &&
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
