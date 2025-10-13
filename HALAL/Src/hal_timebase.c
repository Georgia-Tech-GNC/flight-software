#include "halal.h"
#include "timebase.h"
#include "util.h"

HAL_StatusTypeDef HAL_InitTick(uint32_t tick_priority) {
    if (!HALAL_timebase_init(tick_priority)) return HAL_ERROR;

    return HAL_OK;
}

void HAL_SuspendTick(void) {
    HALAL_timebase_suspend();
}

void HAL_ResumeTick(void) {
    HALAL_timebase_resume();
}
