/**
 * @file system.h
 * @brief System configuration and initialization
 */
#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "stm32h7xx_hal.h"

void SystemClock_Config(void);
void Error_Handler(void);
void MX_DMA_Init(void);
void MX_TIM6_Init(void);
void MX_TIM7_Init(void);
void MX_GPIO_Init(void);

// Timer handles (used by other modules)
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

#endif /* __SYSTEM_H__ */