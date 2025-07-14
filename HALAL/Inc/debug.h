#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>
#include <stddef.h>

uint8_t HALAL_debug_init(void);

uint8_t HALAL_debug_write(const char *msg, size_t len, uint32_t timeout);

#endif