#include "log.h"

#define DEBUG_UART_TIMEOUT HAL_MAX_DELAY
#define DEBUG_UART_NEWLINE "\r\n"

#define PREFIX_SIZE 64
#define MSG_BUF_SIZE 256
#define LINE_BUF_SIZE (MSG_BUF_SIZE + PREFIX_SIZE + strlen(DEBUG_UART_NEWLINE))

/**
 * @brief Log a formatted string to debug_uart
 * @param level the severity of the message being logged
 * @param format the format string
 * @param args the format arguments
 *
 * @return 1 if successful, 0 otherwise
 */
int log_printf(LogLevel level, const char *format, va_list args) {
    char line_buf[LINE_BUF_SIZE];
    char msg_buf[MSG_BUF_SIZE];

    int ret;

    ret = vsnprintf(msg_buf, MSG_BUF_SIZE, format, args);

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

    TickType_t ms = pd_TICKS_TO_MS(xTaskGetTickCount());
    TickType_t sec = ms / 1000;
    ms %= 1000;

    ret = snprintf(line_buf, LINE_BUF_SIZE, "%d.%d [%s] - %s%s", sec, ms, status_str, msg_buf, DEBUG_UART_NEWLINE);

     if (ret < 0 || ret > LINE_BUF_SIZE) {
        return 0;
    }

    if (HAL_UART_Transmit(&debug_uart, (uint8_t *) line_buf, strlen(line_buf), DEBUG_UART_TIMEOUT) != HAL_OK) {
        return 0;
    }

    return 1;
}
