#include "uart_isr.h"
#include "halal.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
#ifdef HALAL_RADIO_UART_ENABLED
    if (huart->Instance == HALAL_RADIO_UART_INSTANCE) {
        radio_uart_rx_event_isr(size);
    }
#endif

#ifdef HALAL_STATE_ESTIMATION_UART_ENABLED
    if (huart->Instance == HALAL_STATE_ESTIMATION_UART_INSTANCE) {
        state_estimation_uart_rx_event_isr(size);
    }
#endif
}