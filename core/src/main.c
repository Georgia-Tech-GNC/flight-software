#include "main.h"

int main(void) {
    initialize_mcu();

    
    while (true) {
        // HAL_UART_Transmit(&huart3, (uint8_t*)"Hello World!\r\n", 15, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(LED_Yellow_GPIO_Port, LED_Yellow_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(LED_Yellow_GPIO_Port, LED_Yellow_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
        HAL_Delay(500);
        
        
    }
}
