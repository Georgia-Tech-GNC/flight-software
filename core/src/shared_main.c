/** @file shared_main.c
 * @brief General entry point for all targets
 */

#include "port.h"
#include "shared_main.h"
#include "init_task.h"
#include "example_task.h"
#include "FreeRTOS.h"
#include "isr.h"
#include "task.h"
#include "zed_f9p.h"

#define STACK_SIZE 1024

StaticTask_t example_task_buffer;
StackType_t example_task_stack[STACK_SIZE];
TaskHandle_t example_task_handle;
StaticTask_t init_task_buffer;
StackType_t init_task_stack[STACK_SIZE];
TaskHandle_t init_task_handle;

/** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    example_task_handle = xTaskCreateStatic(example_task, "Example", STACK_SIZE, NULL, tskIDLE_PRIORITY, example_task_stack, &example_task_buffer);
    init_task_handle = xTaskCreateStatic(init_task, "Init", STACK_SIZE, NULL, tskIDLE_PRIORITY, init_task_stack, &init_task_buffer);

    vTaskSuspend(example_task_handle);

    port_init();

    if (init_task_handle && example_task_handle) {
        vTaskStartScheduler();
    }
    while (true) __asm__ volatile (""); // No return
} 
