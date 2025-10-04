#ifndef GPIO_H
#define GPIO_H

#include "stdint.h"

#define USB_POWER_SWITCH_PIN GPIO_PIN_0
#define USB_POWER_SWITCH_PORT GPIOB

/* LD1 can be either PB0 (SB39 ON and SB47
OFF) or PA5 (SB47 ON and SB39 OFF)*/
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB

uint8_t gpio_init(void);

#endif