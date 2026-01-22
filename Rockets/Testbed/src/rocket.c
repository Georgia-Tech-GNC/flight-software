#include "FreeRTOS.h"
#include "task.h"
#include "rocket.h"
#include "testbed.h"
#include "log.h"

uint8_t rocket_init(void) {
    
    return 1;
}

uint8_t rocket_start(void) {
    log_printf(LOG_INFO, "Starting rocket OS");

    vTaskStartScheduler();

    return 1;
}