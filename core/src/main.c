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
#include <stdint.h>


// /* HELPFUL HINTS:
//  * 
//  * The following peripheral handles have already been configured for you:
//  *  - Led: LED_GREEN_GPIO_Port and LED_GREEN_Pin
//  *  - SPI1: hspi1   (you should pass this to HAL methods as a reference, ie. &hspi1)
//  *  - SPI1 chip-select pin: SPI1_CS_GPIO_Port and SPI1_CS_Pin
//  *  - UART: huart1  (you should pass this to HAL methods as a reference, ie. &huart1)
//  * 
//  *
//  * The following HAL methods may be useful to you:
//  *  - GPIO Pins:
//  *      - HAL_GPIO_WritePin(<GPIO_PORT>, <GPIO_PIN>, GPIO_PIN_SET or GPIO_PIN_RESET)
//  *  - UART:
//  *      - HAL_UART_Transmit(<UART_HANDLE>, <DATA POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
//  *      - HAL_UART_Receive(<UART_HANDLE>, <DATA POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
//  *  - SPI:
//  *      - HAL_SPI_Transmit(<SPI_HANDLE>, <DATA_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
//  *      - HAL_SPI_Receive(<SPI_HANDLE>, <DATA_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
//  *      - HAL_SPI_TransmitReceive(<SPI_HANDLE>, <SEND_DATA_POINTER>, <RECIEVE_DATA_BUFFER_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
//  *  - Misc.
//  *      - HAL_DELAY(<DELAY IN MS>)
//  */


// part B 

//  /** 
//  * The main methods of all targets are expected to call this method once they are fully initialized
//  */
// [[noreturn]] void shared_main(void) {
//     // Don't worry about this line for now :)
//     HAL_TIM_Base_Start(&htim2);

//     lsm6dso_initialize(&hspi1);
//     while (true) {
//         HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

//         float pitch = lsm6dso_get_pitch_rate(&hspi1);
//         float yaw = lsm6dso_get_yaw_rate(&hspi1);
//         float roll = lsm6dso_get_roll_rate(&hspi1);

//         char print[100];
//         int length = snprintf(print, sizeof(print), "pitch = %d, yaw = %d, roll = %d\r\n", (int)pitch, (int)yaw, (int)roll);

//         HAL_UART_Transmit(&huart3, (uint8_t *)print, length, HAL_MAX_DELAY);

//         HAL_Delay(500);
//         HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
//         HAL_Delay(500);
//     }
// } 



// part C

#define STACK_SIZE 4096
static StackType_t sensor_task_stack[STACK_SIZE];
static StaticTask_t sensor_task_tcb;

static StackType_t led_task_stack[STACK_SIZE];
static StaticTask_t led_task_tcb;

static uint8_t value;

static TaskHandle_t sensor_task_handle;
static TaskHandle_t led_task_handle;

// C-5
// cppcheck-suppress constParameterPointer
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        if (value == '\n') {
            BaseType_t higher_priority_woken = false;
            xTaskNotifyFromISR(sensor_task_handle, 0, eNoAction, &higher_priority_woken);
            portYIELD_FROM_ISR(higher_priority_woken);
        } else { 
            HAL_UART_Receive_IT(&huart3, &value, 1);
            return;
        }
    }
}


// part C first task
static void my_task1(void *pvParameters) {
    UNUSED(pvParameters); // Silence warnings related to pvParameters

    // Tasks are expected to run forever and never return
    while (true) {

        // C-5
        HAL_UART_Receive_IT(&huart3, &value, 1);
        xTaskNotifyWait(0, 0, NULL, HAL_MAX_DELAY);

        float pitch = lsm6dso_get_pitch_rate(&hspi1);
        float yaw = lsm6dso_get_yaw_rate(&hspi1);
        float roll = lsm6dso_get_roll_rate(&hspi1);

        char print[100];
        int length = snprintf(print, sizeof(print), "pitch = %d, yaw = %d, roll = %d\r\n", (int)pitch, (int)yaw, (int)roll);

        HAL_UART_Transmit(&huart3, (uint8_t *)print, (uint16_t)length, HAL_MAX_DELAY);

        // C-4
        // uint8_t value;
        // if (HAL_UART_Receive(&huart3, &value, 1, HAL_MAX_DELAY) == HAL_OK && value == '\n') {
        //     float pitch = lsm6dso_get_pitch_rate(&hspi1);
        //     float yaw = lsm6dso_get_yaw_rate(&hspi1);
        //     float roll = lsm6dso_get_roll_rate(&hspi1);

        //     char print[100];
        //     int length = snprintf(print, sizeof(print), "pitch = %d, yaw = %d, roll = %d\r\n", (int)pitch, (int)yaw, (int)roll);

        //     HAL_UART_Transmit(&huart3, (uint8_t *)print, (uint16_t)length, HAL_MAX_DELAY);
        // }

        // before C-4
        // Insert business logic here
        // float pitch = lsm6dso_get_pitch_rate(&hspi1);
        // float yaw = lsm6dso_get_yaw_rate(&hspi1);
        // float roll = lsm6dso_get_roll_rate(&hspi1);

        // char print[100];
        // int length = snprintf(print, sizeof(print), "pitch = %d, yaw = %d, roll = %d\r\n", (int)pitch, (int)yaw, (int)roll);

        // HAL_UART_Transmit(&huart3, (uint8_t *)print, (uint16_t)length, HAL_MAX_DELAY);

        // // Indicate to the scheduler to halt this task and resume after 250ms
        // vTaskDelay(pdMS_TO_TICKS(250));

    }
}


// part C second task
static void my_task2(void *pvParameters) {
    UNUSED(pvParameters); // Silence warnings related to pvParameters

    // Tasks are expected to run forever and never return
    while (true) {
        // Insert business logic here
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(250));
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

[[noreturn]] void shared_main(void) {
    // Don't worry about this line for now :)
    HAL_TIM_Base_Start(&htim2);

    lsm6dso_initialize(&hspi1);

    // part C task 1
    sensor_task_handle = xTaskCreateStatic(my_task1, "sensor_data", STACK_SIZE, NULL, 2, sensor_task_stack, &sensor_task_tcb);

    // part C task 2
    led_task_handle = xTaskCreateStatic(my_task2, "led_blink", STACK_SIZE, NULL, 1, led_task_stack, &led_task_tcb);

    vTaskStartScheduler();
    while (1) {}
}