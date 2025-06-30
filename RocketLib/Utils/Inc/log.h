#ifndef LOG_H
#define LOG_H

typedef enum {
    LOG_INFO = 0,
    LOG_WARNING = 1,
    LOG_ERROR = 2,
} LogLevel;

int log_printf(LogLevel level, char print_buf[], const char *format, ...);

#endif