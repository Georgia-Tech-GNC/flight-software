#include "rgb_led.h"
static uint32_t rgb_led_to_ccr(uint8_t value, uint32_t arr, bool inverted) {
    uint32_t ccr = value * (arr + 1) / 255;
    if(inverted){
        ccr = (arr+1) - ccr;
    }
    if(ccr > UINT16_MAX) {
        ccr = UINT16_MAX;
    }
    return ccr;
}

void rgb_led_set(rgb_led_t *dev, uint8_t r, uint8_t g, uint8_t b) {
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(dev->htim);
    __HAL_TIM_SET_COMPARE(dev->htim, dev->ch_r, rgb_led_to_ccr(r, arr, dev->inverted));
     __HAL_TIM_SET_COMPARE(dev->htim, dev->ch_g, rgb_led_to_ccr(g, arr, dev->inverted));
      __HAL_TIM_SET_COMPARE(dev->htim, dev->ch_b, rgb_led_to_ccr(b, arr, dev->inverted));
}

bool rgb_led_init(rgb_led_t *dev) {
    rgb_led_set(dev, 0, 0, 0);
    return(HAL_TIM_PWM_Start(dev->htim, dev->ch_r) == HAL_OK) 
    && HAL_TIM_PWM_Start(dev->htim, dev->ch_g) == HAL_OK
    && HAL_TIM_PWM_Start(dev->htim, dev->ch_b) == HAL_OK;
}
