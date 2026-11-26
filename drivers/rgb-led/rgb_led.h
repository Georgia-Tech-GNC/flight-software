#ifndef RGB_LED_H
#define RGB_LED_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t ch_r;
    uint32_t ch_g;
    uint32_t ch_b;
    bool inverted;
} rgb_led_t;

bool rgb_led_init(rgb_led_t *dev);
void rgb_led_set(rgb_led_t *dev, uint8_t r, uint8_t g, uint8_t b);

#endif