#include "uart_isr.h"

uint8_t telemetry_uart_rx_buf[TELEMETRY_RX_MAX_PROCESS_SIZE];
// uint8_t state_uart_rx_buf[STATE_PACKET_SIZE];

uint8_t state_bytes[STATE_PACKET_SIZE];

uint8_t state_bytes_received = 0;

void process_state_bytes(uint16_t size, BaseType_t *xHigherPriorityTaskWoken);
void update_rocket_state(BaseType_t *xHigherPriorityTaskWoken);

/**
 * @brief Begin listening for UART data over telemetry_uart and state_uart
 * @return 1 if successful, 0 otherwise
 */
int begin_uart_listen(void) {
    /**
     * RecieveToIdle will call the RxEventCallback interrupt when either an idle event is detected
     * or the expected amount of data is recieved
    */ 
    if (HAL_UARTEx_ReceiveToIdle_IT(&telemetry_uart, telemetry_uart_rx_buf, TELEMETRY_RX_MAX_PROCESS_SIZE) != HAL_OK) {
        return 0;
    }
    
    //if (HAL_UARTEx_ReceiveToIdle_IT(&state_uart, state_uart_rx_buf, STATE_PACKET_SIZE) != HAL_OK) {
    //    return 0;
    //}

    return 1;
}

// #ifndef STATIC_FIRE
/*
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    /* FreeRTOS boilerplate //
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Direct uart data to the appropriate place /
    if (huart->Instance == telemetry_uart.Instance) {
        /* Send telemetry data to the telemetry_rx task /
        xStreamBufferSendFromISR(g_telemetry_rx_sb_handle, telemetry_uart_rx_buf, size, &xHigherPriorityTaskWoken);
        HAL_UARTEx_ReceiveToIdle_IT(&telemetry_uart, telemetry_uart_rx_buf, TELEMETRY_RX_MAX_PROCESS_SIZE);
    } else if (huart->Instance == state_uart.Instance) {
        xStreamBufferSendFromISR(g_telemetry_rx_sb_handle, telemetry_uart_rx_buf, size, &xHigherPriorityTaskWoken);
        HAL_UARTEx_ReceiveToIdle_IT(&state_uart, state_uart_rx_buf, STATE_PACKET_SIZE);
    }

    /* FreeRTOS boilerplate /
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
*/
