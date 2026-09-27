/** @file shared_main.c
 * @brief General entry point for all targets
 */

#include "port.h"
#include "shared_main.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../../drivers/lps25hb/lps25hb.h"

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

// Utility function
static void uart_send_message(const char* str) {
    const uint16_t length = (uint16_t)strlen(str);
    const uint16_t size = sizeof(char) * length;
    
    HAL_UART_Transmit(&huart3, (const uint8_t*)str, size, HAL_MAX_DELAY);
}

/** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    // TaskHandle_t blink_led_1_task = xTaskCreateStatic(blink_led_1, "Blink LED 1", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_1_stack, &blink_led_1_buffer);
    // TaskHandle_t blink_led_2_task = xTaskCreateStatic(blink_led_2, "Blink LED 2", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_2_stack, &blink_led_2_buffer);
    // TaskHandle_t blink_led_3_task = xTaskCreateStatic(blink_led_3, "Blink LED 3", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_3_stack, &blink_led_3_buffer);

    // if (blink_led_1_task && blink_led_2_task && blink_led_3_task) {
    //     HAL_UART_Transmit(&debug_uart, (uint8_t *) "Successfully created tasks, starting scheduler.\r\n", 50, HAL_MAX_DELAY);
    //     vTaskStartScheduler();
    // } else {
    //     HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to create tasks\r\n", 25, HAL_MAX_DELAY);
    // }
    
    // lps25hb_initialize()
    
    int x = 5;
    
    while (true) {
        uint8_t tx[] = { 0x50 };
        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);
        uart_send_message("Before Transmit.\n");
        HAL_I2C_Master_Transmit(&hi2c1, LPS25HB_ADDRESS << 1 | 0, tx, sizeof(tx), HAL_MAX_DELAY);
        uart_send_message("After Transmit.\n");
        
        HAL_Delay(1000);
        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);
    }

    while (true) __asm__ volatile (""); // No return
} 
