#include "tim_isr.h"
#include "halal.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    #ifdef HALAL_TIMEBASE_TIM_ENABLED
    if (htim->Instance == HALAL_TIMEBASE_TIM_INSTANCE) {
        timebase_tim_period_elapsed();
    }
    #endif
}