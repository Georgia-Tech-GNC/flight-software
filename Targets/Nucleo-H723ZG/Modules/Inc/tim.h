#ifndef TIM_H
#define TIM_H

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef g_tim1;
extern TIM_HandleTypeDef g_tim2;

uint8_t tim_init(void);

#endif