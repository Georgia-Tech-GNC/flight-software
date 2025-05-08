#include "FreeRTOS.h"
#include "utils.h"
#include "port_config.h"
#include "globals.h"
#include "stdint.h"

#define DEBUG_MESSAGE_BUFFER_SIZE 100

/**
 * @brief Send a debug message over UART.
 * 
 * When the DEBUG_BUILD flag is not set, 
 *  
 * @param fmt_str   the formatting string (same as printf)
 * @param ...       String template insertions (same as printf)
 * @return          Returns the number of characters written (forwarded from printf)
 */
int log_debug_message(char* fmt_str, ...) {
#ifndef DEBUG_BUILD
    char buf[DEBUG_MESSAGE_BUFFER_SIZE];
    va_list args;
    va_start(args, fmt_str);
    int chars_written = vsprintf(buf, fmt_str, args);
    va_end(args);

    if (chars_written >= 0) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
    }

    return chars_written;
#endif
    return 0;
}

/** 
 * @brief Halts task execution until any notification in the mask is recieved 
 * 
 * @param notification_mask     The mask of notifications that we should wait for
 * @return the notification received
 */
uint32_t wait_for_notification(uint32_t notification_mask) {
    uint32_t notification_value = 0;
    while ((notification_value & notification_mask) == 0) {
        xTaskNotifyWait(0, notification_mask, &notification_value, portMAX_DELAY);
    }
    return notification_value;
}
