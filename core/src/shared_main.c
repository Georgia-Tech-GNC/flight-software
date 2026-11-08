/** @file shared_main.c
 * @brief General entry point for all targets
 */

#include "port.h"
#include "shared_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include "servo.h"
#include "gimbal_tests.h"

#define STACK_SIZE 4096
#define SERVO_COUNTER_FREQ_HZ (SERVO_TIM_CLK_HZ / (servo_tim.Instance->PSC + 1))

static StaticTask_t gimbal_test_buffer;
static StackType_t gimbal_test_stack[TEST_STACK_SIZE];
static gimbal_test_context_t gimbal_context;

encoder_t test_encoder;

static void blink_led_1(void *_params);
static void blink_led_2(void *_params);
// static void blink_led_3(void *_params);

// static void servo_test(void *__params);
// servo_t test_servo;
// StaticTask_t servo_test_buffer;
// StackType_t servo_test_stack[STACK_SIZE];
// extern TIM_HandleTypeDef htim1;

// servo_t servo_yaw;
// servo_t servo_pitch;


StaticTask_t blink_led_1_buffer;
StackType_t blink_led_1_stack[STACK_SIZE];
StaticTask_t blink_led_2_buffer;
StackType_t blink_led_2_stack[STACK_SIZE];
// StaticTask_t blink_led_3_buffer;
// StackType_t blink_led_3_stack[STACK_SIZE];

static void blink_led_1(void *_params) {
    UNUSED(_params);

    while (true) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Blink 1\r\n", 10, HAL_MAX_DELAY);

        vTaskDelay(100);
        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);
        vTaskDelay(100);
        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);
    }
}

static void blink_led_2(void *_params) {
    UNUSED(_params);

    while (true) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Blink 2\r\n", 10, HAL_MAX_DELAY);

        vTaskDelay(200);
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
        vTaskDelay(200);
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);
    }
}

// static void blink_led_3(void *_params) {
//     UNUSED(_params);

//     servo_init(&servo_yaw,
//                &servo_tim,
//                servo_yaw_tim_channel,
//                SERVO_COUNTER_FREQ_HZ,
//                SERVO_MAX_ANGLE*2.0f,
//                SERVO_MIN_PULSE_WIDTH_US,
//                SERVO_MAX_PULSE_WIDTH_US,
//                false);

//     servo_init(&servo_pitch,
//                &servo_tim,
//                servo_pitch_tim_channel,
//                SERVO_COUNTER_FREQ_HZ,
//                SERVO_MAX_ANGLE*2.0f,
//                SERVO_MIN_PULSE_WIDTH_US,
//                SERVO_MAX_PULSE_WIDTH_US,
//                false);

//     servo_enable(&servo_yaw);
//     servo_enable(&servo_pitch);
    
//     while (true) {       
//         servo_set_pos(&servo_pitch, -30);
//         servo_set_pos(&servo_yaw, -50);
//         vTaskDelay(1000);
//     }
        
// }

// static void servo_test(void *__params) {
//     UNUSED(__params);

//     if (!servo_init(
//         &test_servo,
//         &htim1,
//         TIM_CHANNEL_1,
//         SERVO_COUNTER_FREQ_HZ,
//         120.0f,
//         920,
//         2120,
//         true
//     )) {
//         vTaskDelete(NULL);
//     }

//     if (!servo_enable(&test_servo)) {
//         vTaskDelete(NULL);
//     }

//     while (true) {
//         servo_set_pos(&test_servo, -45.0f);
//         vTaskDelay(pdMS_TO_TICKS(1000));

//         servo_set_pos(&test_servo, -30.0f);
//         vTaskDelay(pdMS_TO_TICKS(1000));

//         servo_set_pos(&test_servo, -15.0f);
//         vTaskDelay(pdMS_TO_TICKS(1000));

//         servo_set_pos(&test_servo, 0.0f);
//         vTaskDelay(pdMS_TO_TICKS(1000));

//         servo_set_pos(&test_servo, 45.0f);
//         vTaskDelay(pdMS_TO_TICKS(1000));

//         servo_set_pos(&test_servo, 0.0f);
//         vTaskDelay(pdMS_TO_TICKS(1000));  
//     }
// }

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim == test_encoder.htim &&
        htim->Channel == TIM_ACTIVE_CHANNEL) {

        encoder_capture_callback(&test_encoder);
    }
}

/** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    // TaskHandle_t blink_led_1_task = xTaskCreateStatic(blink_led_1, "Blink LED 1", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_1_stack, &blink_led_1_buffer);
    // TaskHandle_t blink_led_2_task = xTaskCreateStatic(blink_led_2, "Blink LED 2", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_2_stack, &blink_led_2_buffer);
    // TaskHandle_t blink_led_3_task = xTaskCreateStatic(blink_led_3, "Blink LED 3", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_3_stack, &blink_led_3_buffer);
    // TaskHandle_t servo_task = xTaskCreateStatic(servo_test, "Servo Test", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, servo_test_stack, &servo_test_buffer);
    
    servo_t test_servo;
    servo_init(&test_servo, &servo_tim, TIM_CHANNEL_1, SERVO_COUNTER_FREQ_HZ, SERVO_MAX_ANGLE*2.0f, SERVO_MIN_PULSE_WIDTH_US, SERVO_MAX_PULSE_WIDTH_US, false);
    servo_enable(&test_servo);
    gimbal_context.servo = &test_servo;

    encoder_init(&test_encoder,&htim2,ENCODER_PERIOD_CHANNEL,ENCODER_PULSE_CHANNEL,ENCODER_TIMER_FREQ_HZ);
    encoder_enable(&test_encoder);
    gimbal_context.encoder = &test_encoder;

    // Test 1 and 2
    TaskHandle_t gimbal_test_handle = xTaskCreateStatic(gimbal_test_1_task,"Gimbal Test 1",TEST_STACK_SIZE,&gimbal_context,tskIDLE_PRIORITY + 1,gimbal_test_stack,&gimbal_test_buffer);
    
    // Test 3
    // TaskHandle_t gimbal_test_handle = xTaskCreateStatic(gimbal_test_3_task,"Gimbal Test 3",TEST_STACK_SIZE,&gimbal_context,tskIDLE_PRIORITY + 1,gimbal_test_stack,&gimbal_test_buffer);
    
    // Test 4
    // TaskHandle_t gimbal_test_handle = xTaskCreateStatic(gimbal_test_4_task,"Gimbal Test 4",TEST_STACK_SIZE,&gimbal_context,tskIDLE_PRIORITY + 1,gimbal_test_stack,&gimbal_test_buffer);


    if (gimbal_test_handle) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Successfully created tasks, starting scheduler.\r\n", 50, HAL_MAX_DELAY);
        vTaskStartScheduler();
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to create tasks\r\n", 25, HAL_MAX_DELAY);
    }

    while (true) __asm__ volatile (""); // No return
} 
