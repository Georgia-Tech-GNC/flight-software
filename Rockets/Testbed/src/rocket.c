#include "FreeRTOS.h"
#include "task.h"
#include "rocket.h"
#include "testbed.h"
#include "log.h"
#include "util.h"


void testbed_task(void* task_parameters) {
    UNUSED(task_parameters); // Simple macro to stop GCC from complaining that this parameter is unused
    // Insert code here!

    for (;;) {
        // Code
    } vTaskDelay(10000); // Busyloop forever since tasks shouldn't exit
}

TaskHandle_t testbed_task_handle;
StackType_t testbed_task_stack[4096];
StaticTask_t testbed_task_buff;

uint8_t rocket_init(void) {

    testbed_task_handle = xTaskCreateStatic(
        testbed_task,           // The function that this task should run
        "testbed_task",         // Name of task
        4096,                   // Size of task stack
        NULL,                   // Parameters that we want to pass to this task
        1,                      // The task priority (we only have one task so we'll just use 1)
        testbed_task_stack,     // The stack (note that this buffer is statically allocated (hence xTaskCreateStatic instead of xTaskCreate) and is at least as large as the stack size we specified earlier)
        &testbed_task_buff      // This stores the statically-allocated struct that the OS uses internally to store the task data structure
    );
    if (testbed_task_handle == NULL) return 1; // Task creation failed

    log_printf(LOG_INFO, "Rocket initialization complete!");

    return 0;
}

uint8_t rocket_start(void) {
    log_printf(LOG_INFO, "Starting rocket OS");

    vTaskStartScheduler();

    return 0;
}