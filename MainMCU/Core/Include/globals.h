#ifndef GLOBALS_H
#define GLOBALS_H

#include "port_config.h"

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "message_buffer.h"
#include "state.h"
#include "tests.h"
#include "servo.h"

#ifdef USE_TESTS
extern TaskHandle_t g_test_task_handle;
#endif
extern TaskHandle_t g_telemetry_rx_task_handle;
extern TaskHandle_t g_state_tx_task_handle;
extern TaskHandle_t g_state_flash_task_handle;

#ifndef STATIC_FIRE
extern TaskHandle_t g_run_controls_task_handle;
#else
extern TaskHandle_t g_static_fire_task_handle;
#endif

extern SemaphoreHandle_t g_state_mutex_handle;

extern StreamBufferHandle_t g_telemetry_rx_sb_handle;

extern UART_HandleTypeDef telemetry_uart;
extern UART_HandleTypeDef state_uart;
extern UART_HandleTypeDef debug_uart;

extern SPI_HandleTypeDef sd_spi;

#ifdef USE_ADC1
extern ADC_HandleTypeDef hadc1;
#endif
#ifdef USE_ADC2
extern ADC_HandleTypeDef hadc2;
#endif
#ifdef USE_ADC3
extern ADC_HandleTypeDef hadc3;
#endif

#ifdef USE_TIM1
extern TIM_HandleTypeDef htim1;
#endif
#ifdef USE_TIM2
extern TIM_HandleTypeDef htim2;
#endif
#ifdef USE_TIM3
extern TIM_HandleTypeDef htim3;
#endif
#ifdef USE_TIM4
extern TIM_HandleTypeDef htim4;
#endif

#ifdef MCU_H725ZGT6
extern OSPI_HandleTypeDef flash_spi;
#endif

extern RocketState g_current_state;

#endif