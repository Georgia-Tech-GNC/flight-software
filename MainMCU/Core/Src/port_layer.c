#include "port_layer.h"

/* Static memory for tasks, mutexes, etc */
TaskHandle_t g_telemetry_rx_task_handle;
StackType_t telemetry_rx_task_stack[4096];
StaticTask_t telemetry_rx_task_buff;

TaskHandle_t g_state_rx_task_handle;
StackType_t state_rx_task_stack[4096];
StaticTask_t state_rx_task_buff;

TaskHandle_t g_state_flash_task_handle;
StackType_t state_flash_task_stack[4096];
StaticTask_t state_flash_task_buff;

TaskHandle_t g_run_controls_task_handle;
StackType_t run_controls_task_stack[4096];
StaticTask_t run_controls_task_buff;

SemaphoreHandle_t g_state_mutex_handle;
StaticSemaphore_t state_mutex_buff;

StreamBufferHandle_t g_telemetry_rx_sb_handle;
uint8_t telemetry_rx_sb_storage[TELEMETRY_RX_MAX_PROCESS_SIZE + 2];
StaticStreamBuffer_t telemetry_rx_sb_buff;

StreamBufferHandle_t g_state_rx_sb_handle;
uint8_t state_rx_sb_storage[TELEMETRY_RX_MAX_PROCESS_SIZE + 2];
StaticStreamBuffer_t state_rx_sb_buff;

/* Global rocket state struct */
RocketState g_current_state = {0};

/**
 * @brief Initialize the application
 * @return 1 if successful, 0 otherwise
 */
int port_init(void) {
    /* Create mutexes */
    // Mutex to protect the state struct
    /*
    g_state_mutex_handle = xSemaphoreCreateMutexStatic(&state_mutex_buff);
    if (g_state_mutex_handle == NULL) return 0;
    
    /* Create stream/message buffers /
    g_telemetry_rx_sb_handle = xStreamBufferCreateStatic(TELEMETRY_RX_MAX_PROCESS_SIZE + 1, 1, telemetry_rx_sb_storage, &telemetry_rx_sb_buff);
    if (g_telemetry_rx_sb_handle == NULL) return 0;

    /* Create tasks /
    g_telemetry_rx_task_handle = xTaskCreateStatic(telemetry_rx_task, "telemetry_rx_task", 4096, NULL, tskIDLE_PRIORITY, telemetry_rx_task_stack, &telemetry_rx_task_buff);
    if (g_telemetry_rx_task_handle == NULL) return 0;

    // Reads inputted state from the secondary MCU
    g_state_rx_task_handle = xTaskCreateStatic(state_rx_task, "state_rx_task", 4096, NULL, tskIDLE_PRIORITY, state_rx_task_stack, &state_rx_task_buff);
    if (g_state_rx_task_handle == NULL) return 0;

    // Periodically saves the current rocket state to the flash chip
    g_state_flash_task_handle = xTaskCreateStatic(state_flash_task, "state_flash_task", 4096, NULL, tskIDLE_PRIORITY, state_flash_task_stack, &state_flash_task_buff);
    if (g_state_flash_task_handle == NULL) return 0;

    g_run_controls_task_handle = xTaskCreateStatic(run_controls_task, "run_controls_task", 4096, NULL, tskIDLE_PRIORITY, run_controls_task_stack, &run_controls_task_buff);
    if (g_run_controls_task_handle == NULL) return 0;
    
    /* Begin listening over uart /
    if (!begin_uart_listen()) {
        return 0;
    }
    */
    
    return 1;
}

/**
 * @brief Start the application
 */
void port_start(void) {

    HAL_TIM_PWM_Start(PWM0_TIMER, PWM0_CHANNEL);

    uint16_t count = 1000.0 * 3.2;
    __HAL_TIM_SET_COMPARE(PWM0_TIMER, PWM0_CHANNEL, count);

    while (1) { 

    }
    //vTaskStartScheduler();
}