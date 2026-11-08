/** @file shared_main.c
 * @brief General entry point for all targets
 */

#include "port.h"
#include "shared_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "encoder.h"

#include <stdio.h>
#include <string.h>

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

extern TIM_HandleTypeDef htim2;
static encoder_t test_encoder;
static StaticTask_t encoder_test_buffer;
static StackType_t encoder_test_stack[STACK_SIZE];

#define ENCODER_MIN_PULSE_WIDTH 500
#define ENCODER_MAX_PULSE_WIDTH 2500
#define ENCODER_ANGLE_RANGE 180
#define ENCODER_ZERO_PULSE_WIDTH 1500

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim == test_encoder.htim &&
        htim->Channel == TIM_ACTIVE_CHANNEL) {

        encoder_capture_callback(&test_encoder);
    }
}

static void encoder_test(void *params) {
    UNUSED(params);

    char message[128];

    float pulse_us;
    float frequency_hz;
    float duty_cycle;

    while (true) {
        bool valid_pulse = encoder_get_pulse_us(&test_encoder, &pulse_us);
        bool valid_frequency = encoder_get_frequency(&test_encoder, &frequency_hz);
        bool valid_duty = encoder_get_duty_cycle(&test_encoder, &duty_cycle);
        
        if (valid_pulse && valid_frequency && valid_duty) {
            snprintf(
                message,
                sizeof(message),
                "Pulse: %.2f us | Frequency: %.2f Hz | Duty: %.2f%%\r\n",
                (double)pulse_us,
                (double)frequency_hz,
                (double)duty_cycle
            );
        } else {
            snprintf(message, sizeof(message), "Error...\r\n");
        }

        HAL_UART_Transmit(&debug_uart,(uint8_t *)message,(uint16_t)strlen(message),HAL_MAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

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

/** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {

    // Initialize encoder
    bool encoder_initialized = encoder_init(
        &test_encoder,
        &htim2,
        ENCODER_PERIOD_CHANNEL,
        ENCODER_PULSE_CHANNEL,
        ENCODER_TIMER_FREQ_HZ,
        ENCODER_ANGLE_RANGE,
        ENCODER_MIN_PULSE_WIDTH,
        ENCODER_MAX_PULSE_WIDTH,
        ENCODER_ZERO_PULSE_WIDTH
    );

    bool encoder_enabled = false;

    if (encoder_initialized) {
        encoder_enabled = encoder_enable(&test_encoder);
    }

    if (!encoder_enabled) {
        char *msg = "Encoder initialization failed\r\n";

        HAL_UART_Transmit(&debug_uart,(uint8_t *)msg,(uint16_t)strlen(msg),HAL_MAX_DELAY);

        while (true) {}
    }

    TaskHandle_t encoder_test_task = xTaskCreateStatic(encoder_test,"Encoder Test",STACK_SIZE,NULL,tskIDLE_PRIORITY,encoder_test_stack,&encoder_test_buffer);

    TaskHandle_t blink_led_1_task = xTaskCreateStatic(blink_led_1, "Blink LED 1", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_1_stack, &blink_led_1_buffer);
    TaskHandle_t blink_led_2_task = xTaskCreateStatic(blink_led_2, "Blink LED 2", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_2_stack, &blink_led_2_buffer);
    TaskHandle_t blink_led_3_task = xTaskCreateStatic(blink_led_3, "Blink LED 3", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_3_stack, &blink_led_3_buffer);

    if (blink_led_1_task && blink_led_2_task && blink_led_3_task && encoder_test_task) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Successfully created tasks, starting scheduler.\r\n", 50, HAL_MAX_DELAY);
        vTaskStartScheduler();
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to create tasks\r\n", 25, HAL_MAX_DELAY);
    }

    while (true) __asm__ volatile (""); // No return
} 
