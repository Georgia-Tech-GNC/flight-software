#ifndef PORT_CONFIG_H
#define PORT_CONFIG_H

#include "stm32h7xx_hal.h"

/* Change these to the values for your board */
#define SD_CS_GPIO_PORT     GPIOE
#define SD_CS_PIN           GPIO_PIN_10

#define LD1_GPIO_PORT       GPIOB
#define LD1_PIN             GPIO_PIN_0
#define LD2_GPIO_PORT       GPIOE
#define LD2_PIN             GPIO_PIN_1
#define LD3_GPIO_PORT       GPIOB
#define LD3_PIN             GPIO_PIN_14

#define telemetry_uart      huart2
#define state_uart          huart6
#define debug_uart          huart3
#define sd_spi              hspi1

#endif