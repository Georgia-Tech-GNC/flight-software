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

 void uart_message(char* str) {
    uint8_t length = (uint8_t)strlen(str);
    uint8_t size = sizeof(char) * length;
    
    HAL_UART_Transmit(&huart3, (const uint8_t*)str, size, HAL_MAX_DELAY);
}


 /** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    // Don't worry about this line for now :)
    HAL_TIM_Base_Start(&htim2);

    char msg[50];
    bool state = false;

    state = lsm6dso_initialize(&hspi1);

    if (state == true) {
        sprintf(msg, "Initializaiton succesfull!\r\n");
        uart_message(msg);

        while (true) {
                float pitch = lsm6dso_get_pitch_rate(&hspi1);
                float yaw = lsm6dso_get_yaw_rate(&hspi1);
                float roll = lsm6dso_get_roll_rate(&hspi1);

                sprintf(msg,"Pitch: %d d/s | Yaw: %d d/s | Roll: %d d/s\r\n", (int)pitch, (int)yaw, (int)roll);
                // HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
                uart_message(msg);
                HAL_Delay(1000);
            }
    } else {
        sprintf(msg, "Error in initialization :(\r\n");
        uart_message(msg);
    }
    
//     uint8_t myString[] = "Hello";
//     uint8_t recivedData[1] = "";

//     while (true) {
//         // HAL_UART_Transmit(&huart3, myString, 5, HAL_MAX_DELAY);
//         HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
//         HAL_UART_Receive(&huart3, recivedData, 1, HAL_MAX_DELAY);
//         HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
//         HAL_UART_Transmit(&huart3, recivedData, 1, HAL_MAX_DELAY);
//     }
} 
