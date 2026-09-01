#ifndef PORT_H
#define PORT_H

#include "main.h"

extern UART_HandleTypeDef huart3;

#define LED1_GPIO_PORT LED_GREEN_GPIO_Port
#define LED1_PIN LED_GREEN_Pin
#define LED2_GPIO_PORT LED_YELLOW_GPIO_Port
#define LED2_PIN LED_YELLOW_Pin
#define LED3_GPIO_PORT LED_RED_GPIO_Port
#define LED3_PIN LED_RED_Pin

#define debug_uart huart3

#endif
