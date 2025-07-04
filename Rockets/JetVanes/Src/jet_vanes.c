#include "jet_vanes.h"

#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

JetVanesRocketState g_current_state = {0};

uint8_t telemetry_msg_ids[2] = {ROCKETSTATE_MSG_ID, SERVODEFLECTIONS_MSG_ID};

void jet_vanes_task(void *args) {
    vTaskDelay(portMAX_DELAY);
}