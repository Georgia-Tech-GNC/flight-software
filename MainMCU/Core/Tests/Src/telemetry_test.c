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

    uint8_t dummy_data[7] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

    while (1) {

        vTaskDelay(1000);
    }
}