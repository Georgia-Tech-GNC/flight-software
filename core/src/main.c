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


 /** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    // Don't worry about this line for now :)
    HAL_TIM_Base_Start(&htim2);

    lsm6dso_initialize(&hspi1);
    while (true) {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

        float pitch = lsm6dso_get_pitch_rate(&hspi1);
        float yaw = lsm6dso_get_yaw_rate(&hspi1);
        float roll = lsm6dso_get_roll_rate(&hspi1);

        char print[100];
        int length = 