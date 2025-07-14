#ifndef GPIO_H
#define GPIO_H

#include "stdint.h"

#define USB_POWER_SWITCH_PIN GPIO_PIN_6
#define USB_POWER_SWITCH_PORT GPIOG

#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB

uint8_t gpio_init(void);

#endif