/** @file shared_main.c
 * @brief General entry point for all targets
 */

#include "port.h"
#include "shared_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "servo.h"

#define STACK_SIZE 200

static void blink_led_1(void *_params);
static void blink_led_2(void *_params);
static void blink_led_3(void *_params);

static void servo_test(void *__params);
servo_t test_servo;
StaticTask_t servo_test_buffer;
StackType_t servo_test_stack[STACK_SIZE];
extern TIM_HandleTypeDef htim1;

StaticTask_t blink_led_1_buffer;
StackType_t blink_led_1_stack[STACK_SIZE];
StaticTask_t blink_led_2_buffer;
StackType_t blink_led_2_stack[STACK_SIZE];
StaticTask_t blink_led_3_buffer;
StackType_t blink_led_3_stack[STACK_SIZE];

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

static void blink_led_3(void *_params) {
    UNUSED(_params);

    while (true) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Blink 3\r\n", 10, HAL_MAX_DELAY);

        vTaskDelay(400);
        HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_RESET);
        vTaskDelay(400);
        HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_SET);
    }
}

static void servo_test(void *__params) {
    UNUSED(__params);

    if (!servo_init(
        &test_servo,
        &htim1,
        TIM_CHANNEL_1,
        SERVO_TIMER_FREQ_HZ,
        120.0f,
        920,
        2120,
        true
    )) {
        vTaskDelete(NULL);
    }

    // Set initial position before enabling PWM
    servo_set_pos(&test_servo, 0.0f);

    if (!servo_enable(&test_servo)) {
        vTaskDelete(NULL);
    }

    while (true) {
        servo_set_pos(&test_servo, -45.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_set_pos(&test_servo, -30.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_set_pos(&test_servo, -15.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_set_pos(&test_servo, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_set_pos(&test_servo, 45.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));

        servo_set_pos(&test_servo, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));  
    }
}

/** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    TaskHandle_t blink_led_1_task = xTaskCreateStatic(blink_led_1, "Blink LED 1", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_1_stack, &blink_led_1_buffer);
    TaskHandle_t blink_led_2_task = xTaskCreateStatic(blink_led_2, "Blink LED 2", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_2_stack, &blink_led_2_buffer);
    TaskHandle_t blink_led_3_task = xTaskCreateStatic(blink_led_3, "Blink LED 3", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_3_stack, &blink_led_3_buffer);
    TaskHandle_t servo_task = xTaskCreateStatic(servo_test, "Servo Test", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, servo_test_stack, &servo_test_buffer);

    if (blink_led_1_task && blink_led_2_task && blink_led_3_task && servo_task) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Successfully created tasks, starting scheduler.\r\n", 50, HAL_MAX_DELAY);
        vTaskStartScheduler();
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to create tasks\r\n", 25, HAL_MAX_DELAY);
    }

    while (true) __asm__ volatile (""); // No return
} 
