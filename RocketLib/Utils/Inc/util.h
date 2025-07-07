#ifndef UTIL_H
#define UTIL_H

#include "FreeRTOS.h"
#include "lib.h"

#define SUCCESS 1
#define FAILURE 0

#define TRUE 1
#define FALSE 0

uint32_t await_notification_indexed(uint32_t index, uint32_t mask, TickType_t timeout);
uint32_t await_notification(uint32_t mask, TickType_t timeout);
uint8_t memcpy_state_bytes(uint8_t *rocket_state, size_t buffer_size, size_t *bytes_copied);
uint8_t memcpy_state(RocketStateStruct *rocket_state);
uint8_t rocket_assert(uint8_t val, const char *assert_name);

#endif