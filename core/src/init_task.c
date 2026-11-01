#include "port.h"
#include "init_task.h"
#include "tasks.h"
#include "FreeRTOS.h"
#include "task.h"

static void init(void);

void init_task(void *_params) {
    UNUSED(_params);

    WITH_UART(debug,
        HAL_UART_Transmit(uart, (uint8_t *) "Initializing...\r\n", 17, HAL_MAX_DELAY);
    );

    init();

    WITH_UART(debug,
        HAL_UART_Transmit(uart, (uint8_t *) "Done initializing.\r\n", 20, HAL_MAX_DELAY);
    );

    vTaskResume(example_task_handle);
    vTaskDelete(NULL);
}

void init(void) {
    //TODO: Driver initialization logic goes here
}
