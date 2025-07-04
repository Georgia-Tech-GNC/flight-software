#ifndef GLOBALS_H
#define GLOBALS_H

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef telemetry_uart;
extern UART_HandleTypeDef state_uart;
extern UART_HandleTypeDef debug_uart;

extern SPI_HandleTypeDef sd_spi;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#endif