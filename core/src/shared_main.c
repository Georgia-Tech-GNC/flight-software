/** @file shared_main.c
 * @brief General entry point for all targets
 */

#include "port.h"
#include "shared_main.h"
#include "FreeRTOS.h"
#include "projdefs.h"
#include "task.h"

#include "message_buffer.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stm32h7xx_hal_uart.h>
#include <sys/_intsup.h>
// Main Task variables
#define MainTaskStackSize 4096
#define MainTaskPriority (tskIDLE_PRIORITY + 1)
#define MessageBufferSize 256
MessageBufferHandle_t command_buffer_handle;
MessageBufferHandle_t telemetry_buffer_handle;
static StaticMessageBuffer_t command_buffer_struct;
static uint8_t command_buffer_storage[MessageBufferSize];
static StaticMessageBuffer_t telemetry_buffer_struct;
static uint8_t telemetry_buffer_storage[MessageBufferSize];
StaticTask_t main_task_buffer;
StackType_t main_task_stack[MainTaskStackSize];
TickType_t last_fsm_change;

// Get the time from startup in milliseconds
static uint64_t get_time_from_startup(void) {
    return pdTICKS_TO_MS(xTaskGetTickCount());
}

//enum for FSM
typedef enum  {
    GROUND,
    ARMED,
    FAST_ASCENT,
    CONTROLLED,
    UNCONTROLLED,
    PYRO,
    EMERGENCY_STOP,
} FSM_State;

const char *state_names[] = {
    [GROUND] = "GROUND",
    [ARMED] = "ARMED",
    [FAST_ASCENT] = "FAST_ASCENT",
    [CONTROLLED] = "CONTROLLED",
    [UNCONTROLLED] = "UNCONTROLLED",
    [PYRO] = "PYRO",
    [EMERGENCY_STOP] = "EMERGENCY_STOP",
};

//enum for telemetry commands
typedef enum {
    ARM,
    DISARM,
    UNCONTROLL,
    PYRO_DEPLOY,
    ESTOP
} Telemetry_Command;


struct Telemetry_Packet_Send {
    FSM_State state;
    uint64_t time_since_startup;

    //input values
    float imu_velocity[3];
    float imu_acceleration[3];

    // output data
    float orientation[4];
    float velocity[3];
    float position[3];
    float pyro;
    float servo1;
    float servo2;
};

struct Telemetry_Packets_Receive {
    uint8_t message_data[MainTaskStackSize][MessageBufferSize];
    size_t  message_lengths[MainTaskStackSize];
    uint32_t total_messages_read;
};


FSM_State FSM = GROUND;



static double get_angular_x() {
    // To be implemented
    return 0;
}

static double get_angular_y() {
    // To be implemented
    return 0;
}

static double get_angular_z() {
    // To be implemented
    return 0;
}

static int get_acceleration() {
    // To be implemented
    return 0;
}

static int get_altitude_barometer() {
    // To be implemented
    return 0;
}

// will instead be implemented through telemetry packet sent using a struct with state, control inputs and control outputs (PYRO channel, servo values)
static void send_telemetry(const char *state) {
    uint64_t time = get_time_from_startup();
    char message[MessageBufferSize];
    int message_length = snprintf(message, sizeof(message),
                                    "State: %s | MS Since Start: %" PRIu64
                                    " | Roll: %.2f | Pitch: %.2f | Yaw: %.2f"
                                    " | Altitude: %d | Acceleration: %d\r\n",
                                    state, time, get_angular_x(), get_angular_y(), get_angular_z(),
                                    get_altitude_barometer(), get_acceleration());
    if (message_length < 0) {
        return;
    }

    size_t message_size = (size_t)message_length;
    if (message_size >= sizeof(message)) {
        message_size = sizeof(message) - 1;
    }

    (void)xMessageBufferSend(telemetry_buffer_handle, message, message_size,
                                0);
    (void)HAL_UART_Transmit(&debug_uart, (uint8_t *)message,
                            (uint16_t)message_size, HAL_MAX_DELAY);
}

static bool move_fsm_forward() {
    if (FSM < PYRO) {
        FSM++;
        last_fsm_change = xTaskGetTickCount();
        return true;
    } else {
        return false;
    }
}
static void set_fsm_state(FSM_State state) {
    FSM = state;
    char message[MessageBufferSize];
    snprintf(message, sizeof(message), "Forced change to %s", state_names[FSM]);
    send_telemetry(message);
    last_fsm_change = xTaskGetTickCount();
}

// 333 Hz
static void main_task(void *_params) {
    float servo1_position = 0;
    float servo2_position = 0;
    UNUSED(_params);
    TickType_t next_wake = xTaskGetTickCount();

    while (true) {
        // read telemetry for new commands read the entire buffer and process data to get the values we want
        Telemetry_Command telemetry_command;
        if (xMessageBufferReceive(command_buffer_handle, &telemetry_command, sizeof(telemetry_command), pdMS_TO_TICKS(3)) > 0) {
            // process telemetry command
            switch (telemetry_command) {
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
        // process FSM (ground -> armed -> fast ascent -> controlled -> uncontrolled)
        // move servos accordingly
        // send telemetry (FSM state, timer, position, velocity, etc, control in and control out)
        // read previous telemetry packet and update values

        switch (FSM) {
            case GROUND:
                send_telemetry("GROUND");
                break;
            case ARMED:
                // check for launch signal (aka + acceleration)
                if (get_acceleration() > 0) {
                    // if detected change state to FAST_ASCENT
                    move_fsm_forward();
                    send_telemetry("ARMED -> FAST_ASCENT");
                } else {
                    // send telemetry
                    send_telemetry("ARMED");
                }
                break;
            case FAST_ASCENT:
                // motor push for 0.5 seconds
                // after 0.5 seconds change state to CONTROLLED
                if (xTaskGetTickCount() - last_fsm_change >= pdMS_TO_TICKS(500)) {
                    move_fsm_forward();
                    send_telemetry("FAST_ASCENT -> CONTROLLED");
                } else {
                    // send telemetry
                    send_telemetry("FAST_ASCENT");
                }
                break;
            case CONTROLLED:
                // run control algorithm and move servos accordingly
                // switch to uncontrolled once out of power
                // send telemetry
                send_telemetry("CONTROLLED");
                break;
            case UNCONTROLLED:
                // turn motor off
                // check for pyro conditions (timer past a certain time, max altitude reached)
                // send telemetry
                send_telemetry("UNCONTROLLED");
                break;
            case PYRO:
                // deploy pyro
                // send telemetry
                send_telemetry("PYRO_DEPLOY");
                break;
            case EMERGENCY_STOP:
                send_telemetry("EMERGENCY_STOP");
                break;
        }

        // update servos 

        
    }
}

/** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    command_buffer_handle = xMessageBufferCreateStatic(
        MessageBufferSize, command_buffer_storage, &command_buffer_struct);
    telemetry_buffer_handle = xMessageBufferCreateStatic(
        MessageBufferSize, telemetry_buffer_storage, &telemetry_buffer_struct);
    // main task
    TaskHandle_t main_task_handle = xTaskCreateStatic(main_task, "Main Task", MainTaskStackSize, NULL, MainTaskPriority, main_task_stack, &main_task_buffer);

    if (command_buffer_handle && telemetry_buffer_handle && main_task_handle) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Successfully created tasks, starting scheduler.\r\n", 50, HAL_MAX_DELAY);
        vTaskStartScheduler();
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to create tasks\r\n", 20, HAL_MAX_DELAY);
    }


    while (true) __asm__ volatile (""); // No return
} 
