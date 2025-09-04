#ifndef RTOS_GLOBALS_H
#define RTOS_GLOBALS_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"

extern TaskHandle_t g_rocket_task_handle;
extern TaskHandle_t g_telemetry_rx_task_handle;
extern TaskHandle_t g_state_tx_task_handle;
extern TaskHandle_t g_state_flash_task_handle;

extern SemaphoreHandle_t g_state_mutex_handle;
extern StreamBufferHandle_t g_telemetry_rx_sb_handle;

#endif