#include "jet_vanes.h"

#include "halal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "log.h"
#include "rtos_globals.h"
#include "state_flash.h"

JetVanesRocketState g_current_state = {0};

void jet_vanes_task(void *args) {
    while (1) {
        HALAL_adc_convert();

        vTaskDelay(10);
    }

    while (1);
}   