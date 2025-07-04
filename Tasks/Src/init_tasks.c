#include "init_tasks.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "semphr.h"

#include "rtos_globals.h"

#include "telemetry_rx.h"
#include "state_flash.h"
#include "state_tx.h"

/* Static memory for tasks, mutexes, etc */
TaskHandle_t g_telemetry_rx_task_handle;
StackType_t telemetry_rx_task_stack[4096];
StaticTask_t telemetry_rx_task_buff;

TaskHandle_t g_state_tx_task_handle;
StackType_t state_tx_task_stack[4096];
StaticTask_t state_tx_task_buff;

TaskHandle_t g_state_flash_task_handle;
StackType_t state_flash_task_stack[4096];
StaticTask_t state_flash_task_buff;

SemaphoreHandle_t g_state_mutex_handle;
StaticSemaphore_t state_mutex_buff;

StreamBufferHandle_t g_telemetry_rx_sb_handle;
uint8_t telemetry_rx_sb_storage[TELEMETRY_RX_MAX_PROCESS_SIZE + 2];
StaticStreamBuffer_t telemetry_rx_sb_buff;

uint8_t init_tasks(void) {
    /* Create mutexes */
    g_state_mutex_handle = xSemaphoreCreateMutexStatic(&state_mutex_buff);
    if (g_state_mutex_handle == NULL) return 0;
    
    /* Create stream/message buffers */
    g_telemetry_rx_sb_handle = xStreamBufferCreateStatic(TELEMETRY_RX_MAX_PROCESS_SIZE + 1, 1, telemetry_rx_sb_storage, &telemetry_rx_sb_buff);
    if (g_telemetry_rx_sb_handle == NULL) return 0;

    /* Create tasks */
    g_telemetry_rx_task_handle = xTaskCreateStatic(telemetry_rx_task, "telemetry_rx_task", 4096, NULL, tskIDLE_PRIORITY, telemetry_rx_task_stack, &telemetry_rx_task_buff);
    if (g_telemetry_rx_task_handle == NULL) return 0;

    g_state_tx_task_handle = xTaskCreateStatic(state_tx_task, "state_tx_task", 4096, NULL, tskIDLE_PRIORITY, state_tx_task_stack, &state_tx_task_buff);
    if (g_state_tx_task_handle == NULL) return 0;

    g_state_flash_task_handle = xTaskCreateStatic(state_flash_task, "state_flash_task", 4096, NULL, tskIDLE_PRIORITY, state_flash_task_stack, &state_flash_task_buff);
    if (g_state_flash_task_handle == NULL) return 0;

    return 1;
}