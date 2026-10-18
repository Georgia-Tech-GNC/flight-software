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
            double pitch = lsm6dso_get_pitch_rate(&hspi1);
            double yaw   = lsm6dso_get_yaw_rate(&hspi1);
            double roll  = lsm6dso_get_roll_rate(&hspi1);

            char buffer[256];

            int32_t pitch_whole = (int32_t)pitch;
            int32_t pitch_frac  = (int32_t)((pitch - pitch_whole) * 1000.0);
            if (pitch_frac < 0) pitch_frac = -pitch_frac;

            int32_t yaw_whole = (int32_t)yaw;
            int32_t yaw_frac  = (int32_t)((yaw - yaw_whole) * 1000.0);
            if (yaw_frac < 0) yaw_frac = -yaw_frac;

            int32_t roll_whole = (int32_t)roll;
            int32_t roll_frac  = (int32_t)((roll - roll_whole) * 1000.0);
            if (roll_frac < 0) roll_frac = -roll_frac;

            int len = snprintf(buffer, sizeof(buffer),
                "Pitch: %c%ld.%03ld | Yaw: %c%ld.%03ld | Roll: %c%ld.%03ld\r\n",
                (pitch < 0 && pitch_whole == 0) ? '-' : ' ', (long)pitch_whole, (long)pitch_frac,
                (yaw < 0   && yaw_whole == 0)   ? '-' : ' ', (long)yaw_whole,   (long)yaw_frac,
                (roll < 0  && roll_whole == 0)  ? '-' : ' ', (long)roll_whole,  (long)roll_frac
            );

            HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, HAL_MAX_DELAY);


        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));

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
        if (rx_byte == '\r') {
            xTaskNotifyFromISR(sensor_task_handle, 0, eNoAction, &higher_priority_task_woken);
        } else {
            HAL_UART_Receive_IT(huart, &rx_byte, 1);
        }
    }
}


/**
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    HAL_TIM_Base_Start(&htim2);
    
    // Create the sensor task (priority 2)
    bool asd = lsm6dso_initialize(&hspi1);
    HAL_Delay(1000);
    if (!asd) {
        char msg[100];
        int len = snprintf(msg, 100, "LSM6DSO not initialized!\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, HAL_MAX_DELAY);
    } else {
        char msg[100];
        int len = snprintf(msg, 100, "LSM6DSO initialized!\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, HAL_MAX_DELAY);
        HAL_Delay(1000);
        // double pitch = lsm6dso_get_pitch_rate(&hspi1);
        // double yaw   = lsm6dso_get_yaw_rate(&hspi1);
        // double roll  = lsm6dso_get_roll_rate(&hspi1);

        // char buffer[256];

        // int32_t pitch_whole = (int32_t)pitch;
        // int32_t pitch_frac  = (int32_t)((pitch - pitch_whole) * 1000.0);
        // if (pitch_frac < 0) pitch_frac = -pitch_frac;

        // int32_t yaw_whole = (int32_t)yaw;
        // int32_t yaw_frac  = (int32_t)((yaw - yaw_whole) * 1000.0);
        // if (yaw_frac < 0) yaw_frac = -yaw_frac;

        // int32_t roll_whole = (int32_t)roll;
        // int32_t roll_frac  = (int32_t)((roll - roll_whole) * 1000.0);
        // if (roll_frac < 0) roll_frac = -roll_frac;

        // len = snprintf(buffer, sizeof(buffer),
        //     "Pitch: %c%ld.%03ld | Yaw: %c%ld.%03ld | Roll: %c%ld.%03ld\r\n",
        //     (pitch < 0 && pitch_whole == 0) ? '-' : ' ', (long)pitch_whole, (long)pitch_frac,
        //     (yaw < 0   && yaw_whole == 0)   ? '-' : ' ', (long)yaw_whole,   (long)yaw_frac,
        //     (roll < 0  && roll_whole == 0)  ? '-' : ' ', (long)roll_whole,  (long)roll_frac
        // );

        // HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, HAL_MAX_DELAY);

    }
    static StackType_t sensor_task_stack[4096];
    static StaticTask_t sensor_task_buffer;

    sensor_task_handle = xTaskCreateStatic(
        sensor_task,
        "SensorTask",
        4096,
        NULL,
        2,
        sensor_task_stack,
        &sensor_task_buffer
    );
    static StackType_t led_task_stack[4096];
    static StaticTask_t led_task_buffer;

    TaskHandle_t led_task_handle = xTaskCreateStatic(
        led_task,
        "LEDTask",
        4096,
        NULL,
        1,
        led_task_stack,
        &led_task_buffer
    );
    // if (led_task_handle == NULL) {
    //     char msg[100];
    //     int len = snprintf(msg, 100, "Led_Task_handle not initialized!\r\n");
    //     while (true) {
    //             HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, HAL_MAX_DELAY);
    //             HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //             HAL_Delay(1000);
    //             HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //             HAL_Delay(1000);
    //         }
    // }
    // if (sensor_task_handle == NULL) {
    //     char msg[100];
    //     int len = snprintf(msg, 100, "Sensor_Task_handle not initialized!\r\n");
    //     while (true) {
    //         HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, HAL_MAX_DELAY);
    //         HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    //         HAL_Delay(1000);
    //         HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    //         HAL_Delay(1000);
    //     }
    // }

    vTaskStartScheduler();

    while (true) {
    }
}