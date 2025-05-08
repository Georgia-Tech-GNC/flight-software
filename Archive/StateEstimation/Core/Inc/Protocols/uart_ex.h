#ifndef UART_HANDLERS_H
#define UART_HANDLERS_H

#include "main.h"
#include <string.h>
#include <stdio.h>

// Function declarations
void print_rocket_attitude(RocketAttitude *rocket_atd, UART_HandleTypeDef *huart);

#endif /* UART_HANDLERS_H */