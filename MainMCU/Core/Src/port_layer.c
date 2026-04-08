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

StreamBufferHandle_t g_state_rx_sb_handle;
uint8_t state_rx_sb_storage[64];
StaticStreamBuffer_t state_rx_sb_buff;

/* Global rocket state struct */
RocketState g_current_state = {0};

uint8_t state_uart_rx_buf[64];

void master_task(void* args) {
    HAL_UART_Transmit(&debug_uart, "Started master task!\r\n", 22, HAL_MAX_DELAY);

    while (1) {
        uint8_t state_rx_buffer[16];
        xStreamBufferReceive(g_state_rx_sb_handle, state_rx_buffer, 16, portMAX_DELAY);

        float a, b, c, d;
        memcpy(&a, state_rx_buffer, sizeof(float));
        memcpy(&b, state_rx_buffer + 4, sizeof(float));
        memcpy(&c, state_rx_buffer + 8, sizeof(float));
        memcpy(&d, state_rx_buffer + 12, sizeof(float));

        char msg_buffer[256];
        size_t msg_size = sprintf(msg_buffer, "%.4f %.4f %.4f %.4f\r\n", a, b, c, d);
        HAL_UART_Transmit(&debug_uart, (uint8_t*)msg_buffer, msg_size, HAL_MAX_DELAY);
    }
}

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
    
    */

    g_run_controls_task_handle = xTaskCreateStatic(
        master_task, 
        "run_controls_task", 
        4096, 
        NULL, 
        1, 
        run_controls_task_stack, 
        &run_controls_task_buff
    );
    if (g_run_controls_task_handle == NULL) return 0;

    g_state_rx_sb_handle = xStreamBufferCreateStatic(
        64, 1, state_rx_sb_storage, &state_rx_sb_buff);
    if (g_state_rx_sb_handle == NULL) return 0;

    if (HAL_UARTEx_ReceiveToIdle_IT(&state_uart, state_uart_rx_buf, 16) != HAL_OK) {
        return 0;
    }

    HAL_UART_Transmit(&state_uart, "r", 1, HAL_MAX_DELAY);
    
    return 1;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    /* FreeRTOS boilerplate */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Direct uart data to the appropriate place */
    if (huart->Instance == telemetry_uart.Instance) {
        /* Send telemetry data to the telemetry_rx task */
        //xStreamBufferSendFromISR(g_telemetry_rx_sb_handle, telemetry_uart_rx_buf, size, &xHigherPriorityTaskWoken);
        //HAL_UARTEx_ReceiveToIdle_IT(&telemetry_uart, telemetry_uart_rx_buf, TELEMETRY_RX_MAX_PROCESS_SIZE);
    } else if (huart->Instance == state_uart.Instance) {
        xStreamBufferSendFromISR(g_state_rx_sb_handle, state_uart_rx_buf, size, &xHigherPriorityTaskWoken);
        HAL_UARTEx_ReceiveToIdle_IT(&state_uart, state_uart_rx_buf, 16);
    }

    /* FreeRTOS boilerplate */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Start the application
 */
void port_start(void) {

    //HAL_TIM_PWM_Start(PWM0_TIMER, PWM0_CHANNEL);

    //uint16_t count = 1000.0 * 3.2;
    //__HAL_TIM_SET_COMPARE(PWM0_TIMER, PWM0_CHANNEL, count);

    
    vTaskStartScheduler();
}