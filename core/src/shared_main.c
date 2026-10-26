/** @file shared_main.c
 * @brief General entry point for all targets
 */

#include "port.h"
#include "shared_main.h"
#include "FreeRTOS.h"
#include "task.h"

#include "nanoprintf.h"
#include "arm_math.h"

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

#include <inttypes.h>
#include <math.h>

#define FP_SCALE UINT32_C(10000)
#define FP_FORMAT "%s%" PRIu32 ".%04" PRIu32

#define FPTOA(x)                                                        \
    (signbit(x) ? "-" : ""),                                           \
    ((uint32_t)(fabsf(x) * (float)FP_SCALE + 0.5f) / FP_SCALE),        \
    ((uint32_t)(fabsf(x) * (float)FP_SCALE + 0.5f) % FP_SCALE)

/** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    TaskHandle_t blink_led_1_task = xTaskCreateStatic(blink_led_1, "Blink LED 1", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_1_stack, &blink_led_1_buffer);
    TaskHandle_t blink_led_2_task = xTaskCreateStatic(blink_led_2, "Blink LED 2", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_2_stack, &blink_led_2_buffer);
    TaskHandle_t blink_led_3_task = xTaskCreateStatic(blink_led_3, "Blink LED 3", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_3_stack, &blink_led_3_buffer);

    float32_t qa[4] = {1.0f, 0.0f, 1.0f, 0.0f};
    float32_t qb[4] = {0.7071f, 0.7071f, 0.0f, 0.0f};
    float32_t qr[4];
    arm_quaternion_product_single_f32(qa, qb, qr);

    char buffer[256];
    int size = npf_snprintf(
        buffer,
        sizeof(buffer),
        "Result Quaternion: [%f, %f, %f, %f]\r\n",
        qr[0], qr[1], qr[2], qr[3]
    );
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, (uint16_t)size, HAL_MAX_DELAY);



    if (blink_led_1_task && blink_led_2_task && blink_led_3_task) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Successfully created tasks, starting scheduler.\r\n", 50, HAL_MAX_DELAY);
        vTaskStartScheduler();
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to create tasks\r\n", 25, HAL_MAX_DELAY);
    }

    while (true) __asm__ volatile (""); // No return
} 
