#include "sd_test.h"

#include "FreeRTOS.h"
#include "task.h"

#include "port_config.h"

#include "main.h"
#include "periph_io.h"

#include "globals.h"

#include "ff.h"

#define CHUNK_SIZE 512

void sd_test_task(void *args) {
    vTaskSuspend(NULL);
}