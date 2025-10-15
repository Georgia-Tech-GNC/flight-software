#ifndef TIM_ISR_H
#define TIM_ISR_H

#include "FreeRTOS.h"
#include "halal.h"

#ifdef HALAL_TIMEBASE_TIM_ENABLED
void timebase_tim_period_elapsed(void);
#endif

#endif