#ifndef UTIL_H
#define UTIL_H

#include "FreeRTOS.h"
#include "lib.h"

#define RET_SUCCESS 1
#define RET_FAILURE 0

#define TRUE 1
#define FALSE 0

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

uint32_t await_notification_indexed(uint32_t index, uint32_t mask);
uint32_t await_notification(uint32_t mask);
uint8_t memcpy_state(RocketStateStruct *rocket_state);
uint8_t rocket_assert(uint8_t val, const char *assert_name);

#endif