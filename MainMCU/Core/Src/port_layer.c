#include "port_layer.h"

// Declare Tasks
task_data_t g_master_task;

// Declare stream buffers and mutexes
stream_buffer_64_bit_t g_state_stream_buffer;
semaphore_data_t g_state_lock;

// Declare global rocket state struct
RocketState g_current_state = {0};

// Local buffers for UART lines
uint8_t state_uart_rx_buf[64];

/**
 * @brief Initialize the application
 * @return 1 if successful, 0 otherwise
 */
int port_init(void) {
    
    // Create mutexes
    g_state_lock.handle = xSemaphoreCreateMutexStatic(&g_state_lock.freertos_internal_data);
    if (g_state_lock.handle == NULL) return 0;
    
    // Create all tasks
    g_master_task.handle = xTaskCreateStatic(
        master_task_handler, "master_task", TASK_STACK_SIZE, NULL, 1, g_master_task.stack, &g_master_task.freertos_internal_data
    );
    if (g_master_task.handle == NULL) return 0;

    // Create stream buffers
    g_state_stream_buffer.handle = xStreamBufferCreateStatic(
        64, 1, g_state_stream_buffer.internal_buffer, &g_state_stream_buffer.freertos_internal_data);
    if (g_state_stream_buffer.handle == NULL) return 0;

    // Start the UART Interrupt handlers
    if (HAL_UARTEx_ReceiveToIdle_IT(&state_uart, state_uart_rx_buf, 16) != HAL_OK) {
        return 0;
    }
    
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
        xStreamBufferSendFromISR(g_state_stream_buffer.handle, state_uart_rx_buf, size, &xHigherPriorityTaskWoken);
        HAL_UARTEx_ReceiveToIdle_IT(&state_uart, state_uart_rx_buf, 16);
    }

    /* FreeRTOS boilerplate */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Start the application
 */
void port_start(void) {

    // Signal to the state estimation MCU to begin
    HAL_UART_Transmit(&state_uart, "r", 1, HAL_MAX_DELAY);
    //HAL_TIM_PWM_Start(PWM0_TIMER, PWM0_CHANNEL);

    //uint16_t count = 1000.0 * 3.2;
    //__HAL_TIM_SET_COMPARE(PWM0_TIMER, PWM0_CHANNEL, count);

    vTaskStartScheduler();
}