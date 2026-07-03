/** @file main.c
 * @brief General entry point for all targets
 */

#include "main.h"
#include "port.h"



/** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
void shared_main(void) {
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

    while (true) {
        HAL_Delay(500);
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    }
} 
