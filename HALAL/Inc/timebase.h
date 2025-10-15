#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

uint8_t HALAL_timebase_init(uint32_t tick_priority);
uint8_t HALAL_timebase_suspend(void);
uint8_t HALAL_timebase_resume(void);

#endif