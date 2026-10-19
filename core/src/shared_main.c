/** @file shared_main.c
 * @brief General entry point for all targets
 */

#include "port.h"
#include "shared_main.h"
#include "FreeRTOS.h"
#include "projdefs.h"
#include "task.h"

#include "message_buffer.h"
#include <stdint.h>
#include <stdio.h>
#include <stm32h7xx_hal_uart.h>
// Main Task variables
#define MainTaskStackSize 4096
#define MainTaskPriority tskIDLE_PRIORITY
MessageBufferHandle_t telemetry_buffer_handle; // Message buffer from telemetry task
StaticTask_t main_task_buffer;
StackType_t main_task_stack[MainTaskStackSize];

// Get the time from startup in milliseconds
static uint64_t getAbsoluteTimeMS(void) {
    return pdTICKS_TO_MS(xTaskGetTickCount());
}

//enum for FSM
typedef enum  {
    GROUND, // 0
    ARMED, // 1
    FAST_ASCENT, // 2
    CONTROLLED, // 3
    UNCONTROLLED, // 4
    PYRO_DEPLOY, // 5
    EMERGENCY_STOP // 6
} FSM_State;

//enum for telemetry commands
typedef enum {
    ARM,
    UNCONTROLL,
    PYRO,
    ESTOP
} Telemetry_Command;



FSM_State FSM = GROUND;

static void set_fsm_state(FSM_State state) {
    FSM = state;
}

static double get_roll() {
    // To be implemented
    return 0;
}

static double get_pitch() {
    // To be implemented
    return 0;
}

static double get_yaw() {
    // To be implemented
    return 0;
}

static int get_altitude() {
    // To be implemented
    return 0;
}

static int get_acceleration() {
    // To be implemented
    return 0;
}

static void send_telemetry(char *state) {
    uint64_t time = getAbsoluteTimeMS();
    char message[100];
    snprintf(message, 100, "State: %s | MS Since Start: %u | Roll: %d | Pitch: %d | Yaw: %d | Altitude: %d | Acceleration: %d\r\n", state, time, get_roll(), get_pitch(), get_yaw(), get_altitude(), get_acceleration());
    xMessageBufferSend(telemetry_buffer_handle, &message, sizeof(message), portMAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t *)message, 100, HAL_MAX_DELAY);
}

#define STACK_SIZE 200
static void blink_led_1(void *_params);
static void blink_led_2(void *_params);
static void blink_led_3(void *_params);

StaticTask_t blink_led_1_buffer;
StackType_t blink_led_1_stack[STACK_SIZE];
StaticTask_t blink_led_2_buffer;
StackType_t blink_led_2_stack[STACK_SIZE];
StaticTask_t blink_led_3_buffer;
StackType_t blink_led_3_stack[STACK_SIZE];

static void blink_led_1(void *_params) {
    UNUSED(_params);

    while (true) {
        HAL_Delay(100);
        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);
    }
}

static void blink_led_2(void *_params) {
    UNUSED(_params);

    while (true) {
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);
    }
}

static void blink_led_3(void *_params) {
    UNUSED(_params);

    while (true) {
        HAL_Delay(400);
        HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_RESET);
        HAL_Delay(400);
        HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_SET);
    }
}

static void main_task(void *_params) {
    UNUSED(_params);
    while (true) {
        uint64_t time = getAbsoluteTimeMS();
        char buffer[100];
        int len = snprintf(buffer, 100, "Main task is running at %u\r\n", time);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) buffer, len, HAL_MAX_DELAY);
        // read telemetry for new commands
        Telemetry_Command telemetry;
        xMessageBufferReceive(telemetry_buffer_handle, &telemetry, sizeof(telemetry), portMAX_DELAY);
        // process telemetry command
        switch (telemetry) {
            case ARM:
                set_fsm_state(ARMED);
                break;
            case UNCONTROLL:
                set_fsm_state(UNCONTROLLED);
                break;
            case PYRO:
                set_fsm_state(PYRO_DEPLOY);
                break;
            case ESTOP:
                set_fsm_state(EMERGENCY_STOP);
                break;
        }
        // process FSM (ground -> armed -> fast ascent -> controlled -> uncontrolled)
        // move servos accordingly
        // send telemetry (FSM state, timer, position, velocity, etc, control in and control out)
        switch (FSM) {
            case GROUND:
                break;
            case ARMED:
                // check for launch signal (aka + acceleration)
                // if detected change state to FAST_ASCENT
                // send telemetry
                send_telemetry("ARMED");
                break;
            case FAST_ASCENT:
                // motor push for 0.5 seconds
                // after 0.5 seconds change state to CONTROLLED
                // send telemetry
                send_telemetry("FAST_ASCENT");
                break;
            case CONTROLLED:
                // run control algorithm and move servos accordingly
                // send telemetry
                send_telemetry("CONTROLLED");
                break;
            case UNCONTROLLED:
                // turn motor off
                // check for pyro conditions (timer past a certain time, max altitude reached)
                // send telemetry
                send_telemetry("UNCONTROLLED");
                break;
            case PYRO_DEPLOY:
                // deploy pyro
                // send telemetry
                send_telemetry("PYRO_DEPLOY");
                break;
            case EMERGENCY_STOP:
                send_telemetry("EMERGENCY_STOP");
                break;
        }


        vTaskDelay(pdMS_TO_TICKS(333));
    }
}

/** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    TaskHandle_t blink_led_1_task = xTaskCreateStatic(blink_led_1, "Blink LED 1", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_1_stack, &blink_led_1_buffer);
    TaskHandle_t blink_led_2_task = xTaskCreateStatic(blink_led_2, "Blink LED 2", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_2_stack, &blink_led_2_buffer);
    TaskHandle_t blink_led_3_task = xTaskCreateStatic(blink_led_3, "Blink LED 3", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_3_stack, &blink_led_3_buffer);
    // main task
    TaskHandle_t main_task_handle = xTaskCreateStatic(main_task, "Main Task", MainTaskStackSize, NULL, tskIDLE_PRIORITY, main_task_stack, &main_task_buffer);

    if (blink_led_1_task && blink_led_2_task && blink_led_3_task && main_task_handle) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Successfully created tasks, starting scheduler.\r\n", 50, HAL_MAX_DELAY);
        vTaskStartScheduler();
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to create tasks\r\n", 25, HAL_MAX_DELAY);
    }


    while (true) __asm__ volatile (""); // No return
} 
