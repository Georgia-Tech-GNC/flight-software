/** @file main.c
 * @brief General entry point for all targets
 */

#include "main.h"
#include "port.h"

void example(int x);

void example(int x) {
    if (x > 0) {
        __asm__ volatile("nop");
    } else if (x > 0) {  // cppcheck: "Expression is always false"
        __asm__ volatile("nop");
    }
}

/** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
void shared_main(void) {
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

    example(5);

    while (true) {
        HAL_Delay(500);
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    }
} 
