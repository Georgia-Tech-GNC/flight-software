#include "rocketlib.h"
#include "FreeRTOS.h"

void run_rocket_os() {
    if (initialize_debug() != STATUS_OK) {
        while (1) { }
    }

    debug("[ROS] Successfully initialized debug port");

    if (initialize_rocket() != STATUS_OK) {
        debug("[ROS] Initialization FAILED! Halting.");
    }

    debug("[ROS] Rocket initialization succeeded. Starting FreeRTOS scheduler");

    vTaskStartScheduler();

    debug("[ROS] FreeRTOS scheduler exited unexpectedly");
    while (1) { }
}

void __attribute__((weak)) debug(const char *restrict format, ...) {
    UNUSED(format);
}