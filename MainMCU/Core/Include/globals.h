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

#define TASK_STACK_SIZE 4096
typedef struct {
    TaskHandle_t handle;
    StackType_t  stack[TASK_STACK_SIZE];
    StaticTask_t freertos_internal_data;
} task_data_t;

extern task_data_t g_master_task;
extern task_data_t g_telemetry_task;

typedef struct {
    StreamBufferHandle_t handle;
    uint8_t              internal_buffer[64];
    StaticStreamBuffer_t freertos_internal_data;
} stream_buffer_64_bit_t;

extern stream_buffer_64_bit_t g_state_stream_buffer;

typedef struct {
    SemaphoreHandle_t handle;
    StaticSemaphore_t freertos_internal_data;
} semaphore_data_t;

extern semaphore_data_t g_state_lock;

extern UART_HandleTypeDef telemetry_uart;
extern UART_HandleTypeDef state_uart;
extern UART_HandleTypeDef debug_uart;

extern SPI_HandleTypeDef sd_spi;

extern servo_t servo_1;

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

extern OSPI_HandleTypeDef flash_spi;

extern RocketState g_current_state;

#endif