#include "telemetry_test.h"
#include "telemetry.h"
#include "stdint.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "globals.h"
#include "port_config.h"

void telemetry_test_task(void *args) {

    uint8_t rx[2];

    while (1) {
        char s[19];

        vTaskDelay(1000);
    }
}