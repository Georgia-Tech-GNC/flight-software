#ifndef LOG_H
#define LOG_H

#include <stdarg.h>
#include "stdint.h"

typedef enum {
    LOG_INFO = 0,
    LOG_WARNING = 1,
    LOG_ERROR = 2,
} LogLevel;

uint8_t log_printf(LogLevel level, const char *format, ...);

#endif