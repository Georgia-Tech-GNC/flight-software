#include "jet_vanes.h"

#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "log.h"

JetVanesRocketState g_current_state = {0};

void jet_vanes_task(void *args) {
    uint16_t iters = 0;

    while (1) {
        if ((iters++) == 1000) {
            log_printf(LOG_INFO, "Running!");
            iters = 0;
        }
        MX_USB_HOST_Process();

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}   