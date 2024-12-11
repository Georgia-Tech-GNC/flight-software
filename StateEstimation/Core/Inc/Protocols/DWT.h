/**
 * @file dwt_timing.h
 * @brief High precision timing utilities using DWT
 */
#ifndef DWT_H
#define DWT_H

#include "stm32h7xx_hal.h"
#include "arm_math.h"

// Initialize DWT
void DWT_Init(void);
uint32_t DWT_GetMicros(void);
void delay_us(uint32_t microseconds);
float32_t DWT_TicksToSeconds(uint32_t ticks);

#endif