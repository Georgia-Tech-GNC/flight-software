#include "log.h"

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "util.h"
#include "halal.h"
#include <string.h>
#include <stdio.h>

#define DEBUG_UART_TIMEOUT HAL_MAX_DELAY
#define DEBUG_UART_NEWLINE "\r\n"

#define PREFIX_SIZE 64
#define MSG_BUF_SIZE 1024
#define LINE_BUF_SIZE (MSG_BUF_SIZE + PREFIX_SIZE + strlen(DEBUG_UART_NEWLINE))

/**
 * @brief Log a formatted string to debug_uart
 * @param level the severity of the message being logged
 * @param format the format string
 * @param ... the format arguments
 *
 * @return 1 if successful, 0 otherwise
 */
uint8_t log_printf(LogLevel level, const char *format, ...) {
    char line_buf[LINE_BUF_SIZE];
    char msg_buf[MSG_BUF_SIZE];

    int ret;

    va_list args;
    va_start(args, format);
    ret = vsnprintf(msg_buf, MSG_BUF_SIZE, format, args);
    va_end(args);

    if (ret < 0 || ret > MSG_BUF_SIZE) {
        return 0;
    }

    const char *status_str;

    switch (level) {
        case LOG_INFO:
            status_str = "INFO";
            break;
        case LOG_WARNING:
            status_str = "WARN";
            break;
        case LOG_ERROR:
            status_str = "ERROR";
            break;
        default:
            status_str = "???";
            break;
    }

    TickType_t ms = pdTICKS_TO_MS(xTaskGetTickCount());
    TickType_t sec = ms / 1000;
    ms %= 1000;

    ret = snprintf(line_buf, LINE_BUF_SIZE, "[%ld.%03ld - %s] - %s%s", sec, ms, status_str, msg_buf, DEBUG_UART_NEWLINE);

    if (ret < 0 || (size_t) ret > LINE_BUF_SIZE) {
        return RET_FAILURE;
    }

    if (!HALAL_debug_write(line_buf, strlen(line_buf), DEBUG_UART_TIMEOUT)) {
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}
