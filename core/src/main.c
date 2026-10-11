/** @file main.c
 * @brief General entry point for all targets
 */

#include "main.h"
#include "lsm6dso.h"

#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>
#include <stdio.h>


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
 *      - HAL_UART_Receive_IT(<UART_HANDLE>, <DATA POINTER>, <DATA_SIZE>)
 *  - SPI:
 *      - HAL_SPI_Transmit(<SPI_HANDLE>, <DATA_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *      - HAL_SPI_Receive(<SPI_HANDLE>, <DATA_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *      - HAL_SPI_TransmitReceive(<SPI_HANDLE>, <SEND_DATA_POINTER>, <RECIEVE_DATA_BUFFER_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *  - Misc.
 *      - HAL_DELAY(<DELAY IN MS>)
 */


// Global variables for UART receive
static uint8_t rx_byte = 0;
static TaskHandle_t sensor_task_handle = NULL;

/**
 * Task that reads pitch, yaw, roll from LSM6DSO and prints them over UART when a newline is received.
 * Uses UART interrupts and direct-to-task notification to avoid blocking.
 */
void sensor_task(void *pvParameters) {
    UNUSED(pvParameters);
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

    while (true) {
        xTaskNotifyWait(0, 0xffffffff, NULL, portMAX_DELAY);

        if (rx_byte == '\n') {
            double pitch = lsm6dso_get_pitch_rate(&hspi1);
            double yaw   = lsm6dso_get_yaw_rate(&hspi1);
            double roll  = lsm6dso_get_roll_rate(&hspi1);

            char buffer[100];
            int len = snprintf(buffer, 100, "Pitch: %.2f, Yaw: %.2f, Roll: %.2f\r\n", pitch, yaw, roll);
            HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, HAL_MAX_DELAY);
        }

        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

    }
}

/**
 * Task that blinks the green LED at 250ms intervals (toggle every 250ms -> 500ms period).
 */
void led_task(void *pvParameters) {
    UNUSED(pvParameters);
    while (true) {
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}


/**
 * UART receive complete callback.
 * Called when a byte is received via UART in interrupt mode.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(sensor_task_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}


/**
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    HAL_TIM_Base_Start(&htim2);

    // Create the sensor task (priority 2)
    sensor_task_handle = xTaskCreateStatic(
        sensor_task,
        "SensorTask",   
        4096,     
        NULL,     
        2,          
        NULL,   
        NULL      
    );

    TaskHandle_t led_task_handle = xTaskCreateStatic(
        led_task,
        "LEDTask",   
        4096,  
        NULL,  
        1,       
        NULL,
        NULL   
    );

    if (sensor_task_handle == NULL || led_task_handle == NULL) {
        while (true) {
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
            HAL_Delay(100);
        }
    }

    vTaskStartScheduler();

    while (true) {
    }
}