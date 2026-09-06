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
 *  - SPI:
 *      - HAL_SPI_Transmit(<SPI_HANDLE>, <DATA_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *      - HAL_SPI_Receive(<SPI_HANDLE>, <DATA_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *      - HAL_SPI_TransmitReceive(<SPI_HANDLE>, <SEND_DATA_POINTER>, <RECIEVE_DATA_BUFFER_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *      - HAL_SPI
 *  - Misc.
 *      - HAL_DELAY(<DELAY IN MS>)
 */

/* newlib-nano's printf does not support floats, so values are split into a
 * whole part and a zero-padded fractional part and printed as two integers.
 * SIGN carries the sign separately because it is lost when the whole part is 0. */
#define SIGN(f)  ((f) < 0 ? "-" : "")
#define WHOLE(f) ((int)((f) < 0 ? -(f) : (f)))
#define FRAC(f)  ((int)(((f) < 0 ? -(f) : (f)) * 100000.0f) % 100000)

 /** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
static StackType_t imu_task_stack[1024];
static StaticTask_t imu_task_tcb;
static TaskHandle_t imu_task_handle;
static StackType_t blink_task_stack[1024];
static StaticTask_t blink_task_tcb;
static TaskHandle_t blink_task_handle;
static uint8_t byte = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance != USART3)
    {
        return;
    }
    if(byte == '\r' || byte == '\n')
    {
        BaseType_t hiWoken = pdFALSE;
        xTaskNotifyFromISR(imu_task_handle, 0, eNoAction, &hiWoken);
        portYIELD_FROM_ISR(hiWoken);
    }
    else {
        HAL_UART_Receive_IT(huart, &byte, 1);
    }
}

static void imu_task(void *pvParameters) {
    UNUSED(pvParameters); // Silence warnings related to pvParameters
    char buf[80];
    // Tasks are expected to run forever and never return
    while (true) {
        // Insert business logic here
        float pitch = lsm6dso_get_pitch_rate(&hspi1);
        float roll  = lsm6dso_get_roll_rate(&hspi1);
        float yaw   = lsm6dso_get_yaw_rate(&hspi1);

        int len = snprintf(buf, sizeof(buf),
                       "P %s%d.%05d  R %s%d.%05d  Y %s%d.%05d\r\n",
                       SIGN(pitch), WHOLE(pitch), FRAC(pitch),
                       SIGN(roll),  WHOLE(roll),  FRAC(roll),
                       SIGN(yaw),   WHOLE(yaw),   FRAC(yaw));
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
        HAL_UART_Receive_IT(&huart3, &byte, 1);
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    }
}

static void blinkTask (void *pvParameters)
{
    while (true)
    {
        UNUSED(pvParameters);
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(250));
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

[[noreturn]] void shared_main(void) {
    // Don't worry about this line for now :)
    HAL_TIM_Base_Start(&htim2);
    bool ok = lsm6dso_initialize(&hspi1);
    char msg[32];
    int n = snprintf(msg, sizeof(msg), "LSM6DSO init: %s\r\n", ok ? "OK" : "FAILED");
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, (uint16_t)n, HAL_MAX_DELAY);
    imu_task_handle = xTaskCreateStatic(imu_task, "IMU", 1024, NULL, 2, imu_task_stack, &imu_task_tcb);
    // if "my_task_handle" is null, something is wrong with the parameters you passed to xTaskCreateStatic
    blink_task_handle = xTaskCreateStatic(blinkTask, "BLINK", 1024, NULL, 1, blink_task_stack, &blink_task_tcb);

    // Start the scheduler. This method will never exit under our usage (it replaces the while(true) loop)
    vTaskStartScheduler();
    while (true);
}