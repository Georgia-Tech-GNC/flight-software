/** @file main.c
 * @brief General entry point for all targets
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#define STACK_SIZE 200

static void blink_red(void *_params);
static void blink_yellow(void *_params);
static void blink_green(void *_params);

StaticTask_t blink_red_buffer;
StackType_t blink_red_stack[STACK_SIZE];
StaticTask_t blink_yellow_buffer;
StackType_t blink_yellow_stack[STACK_SIZE];
StaticTask_t blink_green_buffer;
StackType_t blink_green_stack[STACK_SIZE];

static void blink_red(void *_params) {
    UNUSED(_params);

    while (1) {
        HAL_Delay(100);
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    }
}

static void blink_yellow(void *_params) {
    UNUSED(_params);

    while (1) {
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
    }
}

static void blink_green(void *_params) {
    UNUSED(_params);

    while (1) {
        HAL_Delay(500);
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    }
}

/** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    TaskHandle_t blink_red_task = xTaskCreateStatic(blink_red, "Blink Red", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_red_stack, &blink_red_buffer);
    TaskHandle_t blink_yellow_task = xTaskCreateStatic(blink_yellow, "Blink Yellow", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_yellow_stack, &blink_yellow_buffer);
    TaskHandle_t blink_green_task = xTaskCreateStatic(blink_green, "Blink Green", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_green_stack, &blink_green_buffer);

    if (blink_red_task && blink_yellow_task && blink_green_task) {
        HAL_UART_Transmit(&huart3, (uint8_t *) "Successfully created tasks, starting scheduler.\r\n", 50, HAL_MAX_DELAY);
        vTaskStartScheduler();
    } else {
        HAL_UART_Transmit(&huart3, (uint8_t *) "Failed to create tasks\r\n", 25, HAL_MAX_DELAY);
    }

    while (true) __asm__ volatile (""); // No return
} 
