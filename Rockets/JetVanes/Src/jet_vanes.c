#include "jet_vanes.h"

#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "log.h"

JetVanesRocketState g_current_state = {0};

void jet_vanes_task(void *args) {
    while (1) {
        log_printf(LOG_INFO, "Running!");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}