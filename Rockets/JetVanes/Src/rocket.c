#include "rocket.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "semphr.h"

#include "jet_vanes.h"

/* Static memory for tasks, mutexes, etc */
TaskHandle_t g_jet_vanes_task_handle;
StackType_t jet_vanes_task_stack[4096];
StaticTask_t jet_vanes_task_buff;

uint8_t rocket_init(void) {
    g_jet_vanes_task_handle = xTaskCreateStatic(jet_vanes_task, "jet_vanes_task", 4096, NULL, tskIDLE_PRIORITY, jet_vanes_task_stack, &jet_vanes_task_buff);
    if (g_jet_vanes_task_handle == NULL) return 0;
    
    return 1;
}

uint8_t rocket_start(void) {
    vTaskStartScheduler();

    return 1;
}