#ifndef GLOBALS_H
#define GLOBALS_H

#include "port_config.h"

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "message_buffer.h"

extern TaskHandle_t g_test_task_handle;
extern TaskHandle_t g_sdio_task_handle;
extern TaskHandle_t g_telemetry_tx_task_handle;
extern TaskHandle_t g_telemetry_rx_task_handle;
extern TaskHandle_t g_state_rx_task_handle;
extern SemaphoreHandle_t g_state_mutex_handle;
extern MessageBufferHandle_t g_telemetry_tx_mb_handle;
extern StreamBufferHandle_t g_telemetry_rx_sb_handle;
extern StreamBufferHandle_t g_state_rx_sb_handle;
extern MessageBufferHandle_t g_sdio_mb_handle;

extern UART_HandleTypeDef telemetry_uart;
extern UART_HandleTypeDef state_uart;
extern UART_HandleTypeDef debug_uart;

extern SPI_HandleTypeDef sd_spi;

#ifdef MCU_H725ZGT6
extern OSPI_HandleTypeDef flash_spi;
#endif

extern uint8_t *g_current_state;

#endif