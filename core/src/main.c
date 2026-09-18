/** @file main.c
 * @brief General entry point for all targets
 */

#include "main.h"

#include "lsm6dso.h"

#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>
#include <stdio.h>
#include <string.h>


/* HELPFUL HINTS:
 * 
 * The following peripheral handles have already been configured for you:
 *  - Led: LED_GREEN_GPIO_Port and LED_GREEN_Pin
 *  - SPI1: hspi1   (you should pass this to HAL methods as a reference, ie. &hspi1)
 *  - SPI1 chip-select pin: SPI1_CS_GPIO_Port and SPI1_CS_Pin
 *  - UART: huart1  (you should pass this to HAL methods as a reference, ie. &huart1)
 * 
 *
 * The following HAL methods may be useful to you:
 *  - GPIO Pins:
 *      - HAL_GPIO_WritePin(<GPIO_PORT>, <GPIO_PIN>, GPIO_PIN_SET or GPIO_PIN_RESET)
 *  - UART:
 *      - HAL_UART_Transmit(<UART_HANDLE>, <DATA POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *      - HAL_UART_Receive(<UART_HANDLE>, <DATA POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *  - SPI:
 *      - HAL_SPI_Transmit(<SPI_HANDLE>, <DATA_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *      - HAL_SPI_Receive(<SPI_HANDLE>, <DATA_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *      - HAL_SPI_TransmitReceive(<SPI_HANDLE>, <SEND_DATA_POINTER>, <RECIEVE_DATA_BUFFER_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *  - Misc.
 *      - HAL_DELAY(<DELAY IN MS>)
 */

// lsm6dso task data
static constexpr uint32_t lsm6dso_task_stack_size = 4096 / sizeof(StackType_t);
static StackType_t lsm6dso_task_stack[lsm6dso_task_stack_size];
static StaticTask_t lsm6dso_task_buffer;
static TaskHandle_t lsm6dso_task_handle;

// led task data
static constexpr uint32_t led_task_stack_size = 4096 / sizeof(StackType_t);
static StackType_t led_task_stack[led_task_stack_size];
static StaticTask_t led_task_buffer;
static TaskHandle_t led_task_handle;

static TaskHandle_t uart_notify_handle = NULL;

// uart recv data
static char uart_recv_char = '\0';

// Utility function
static void uart_send_message(const char* str) {
    const uint16_t length = (uint16_t)strlen(str);
    const uint16_t size = sizeof(char) * length;
    
    HAL_UART_Transmit(&huart3, (const uint8_t*)str, size, HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart != &huart3) 
        return;
    
    if (uart_recv_char != '\n' && uart_recv_char != '\r') {
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&uart_recv_char, 1);
        
        return;
    }
    
    if (uart_notify_handle == NULL)
        return;
    
    BaseType_t higher_priority_task_woken = pdFALSE;
    xTaskNotifyFromISR(uart_notify_handle, 0, eNoAction, &higher_priority_task_woken);
}

static void lsm6dso_task(void *pvParameters) {
    UNUSED(pvParameters);
    
    char buf[100];

    while (true) {
        uart_notify_handle = xTaskGetCurrentTaskHandle();
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&uart_recv_char, 1);
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        
        const float pitch_rate = lsm6dso_get_pitch_rate(&hspi1);
        const float roll_rate = lsm6dso_get_roll_rate(&hspi1);
        const float yaw_rate = lsm6dso_get_yaw_rate(&hspi1);
        
        snprintf(
            buf, 
            sizeof(buf), 
            "Roll Rate (dps): %d, Roll Rate (dps): %d, Yaw Rate (dps): %d", 
            (int)pitch_rate, 
            (int)roll_rate, 
            (int)yaw_rate
        );
        uart_send_message(buf);
    }
}

static void led_task(void* pvParameters) {
    UNUSED(pvParameters);
    
    while (true) {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(125));
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
        vTaskDelay(pdMS_TO_TICKS(125));
    }
}

 /** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    // Don't worry about this line for now :)
    HAL_TIM_Base_Start(&htim2);
    
    uart_send_message("\n\nHello World!\n\n");
    
    if (lsm6dso_initialize(&hspi1)) {
        uart_send_message("Successfully initialize lsm6dso device!\n");
    }
    else {
        uart_send_message("Failed to initialize lsm6dso device!\n");
    }
    
    lsm6dso_task_handle = xTaskCreateStatic(
        lsm6dso_task, 
        "lsm6dso_task", 
        lsm6dso_task_stack_size, 
        NULL, 
        2, 
        lsm6dso_task_stack, 
        &lsm6dso_task_buffer
    );
    
    led_task_handle = xTaskCreateStatic(
        led_task, 
        "led_task", 
        led_task_stack_size, 
        NULL, 
        1, 
        led_task_stack, 
        &led_task_buffer
    );
    
    vTaskStartScheduler();
} 
