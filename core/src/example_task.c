#include "port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "example_task.h"
#include <string.h>
#include <stdio.h>
#include "isr.h"

void example_task(void *_params) {
    UNUSED(_params);

    while (true) {
        WITH_UART(debug,
            HAL_UART_Transmit(uart, (uint8_t *) "Hello, world!\r\n", (uint16_t) 15, HAL_MAX_DELAY);
        );        
    }
        
}


