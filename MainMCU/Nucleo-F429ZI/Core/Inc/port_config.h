#ifndef PORT_CONFIG_H
#define PORT_CONFIG_H

#include "stm32f4xx_hal.h"

#define SD_CS_GPIO_PORT     GPIOA
#define SD_CS_PIN           GPIO_PIN_4

#define telemetry_uart      huart2
#define state_uart          huart6
#define debug_uart          huart3
#define sd_spi              hspi1

#endif