#include "rocket.h"

#include "FreeRTOS.h"
#include "portmacro.h"

RocketState g_current_state = {0};

void run_rocket_task(void *args) {
    vTaskDelay(portMAX_DELAY);
}