#ifndef UART_H
#define UART_H

#include "stdint.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef g_uart2;
extern UART_HandleTypeDef g_uart3;
extern UART_HandleTypeDef g_uart6;

uint8_t uart_init(void);

#endif