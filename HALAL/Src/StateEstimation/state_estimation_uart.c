#include "state_estimation.h"
#include "util.h"
#include "halal.h"
#include "lib.h"
#include "rocket.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "rtos_globals.h"
#include "state_flash.h"
#include "state_tx.h"

#include <string.h>

UART_HandleTypeDef state_estimation_uart = {0};

uint8_t state_uart_rx_buf[HALAL_STATE_ESTIMATION_PACKET_SIZE];
uint8_t state_bytes[HALAL_STATE_ESTIMATION_PACKET_SIZE];

uint8_t HALAL_state_estimation_init(void) {
#ifndef HALAL_STATE_ESTIMATION_UART_ENABLED
    return RET_FAILURE;
#endif

    state_estimation_uart.Instance = HALAL_STATE_ESTIMATION_UART_INSTANCE;
    state_estimation_uart.Init.BaudRate = HALAL_STATE_ESTIMATION_UART_BAUDRATE;
    state_estimation_uart.Init.WordLength = HALAL_STATE_ESTIMATION_UART_WORDLENGTH;
    state_estimation_uart.Init.StopBits = HALAL_STATE_ESTIMATION_UART_STOPBITS;
    state_estimation_uart.Init.Parity = HALAL_STATE_ESTIMATION_UART_PARITY;
    state_estimation_uart.Init.Mode = UART_MODE_TX_RX;
    state_estimation_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    state_estimation_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    
    HALAL_STATE_ESTIMATION_UART_RCC_EN();

    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = HALAL_STATE_ESTIMATION_UART_TX_PIN;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = HALAL_STATE_ESTIMATION_UART_AF;

    HAL_GPIO_Init(HALAL_STATE_ESTIMATION_UART_TX_PORT, &gpio_init);

    gpio_init.Pin = HALAL_STATE_ESTIMATION_UART_RX_PIN;

    HAL_GPIO_Init(HALAL_STATE_ESTIMATION_UART_RX_PORT, &gpio_init);

    if (HAL_UART_Init(&state_estimation_uart) != HAL_OK) {
        return RET_FAILURE;
    }

    HAL_NVIC_SetPriority(HALAL_STATE_ESTIMATION_UART_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(HALAL_STATE_ESTIMATION_UART_IRQn);
   
    return RET_SUCCESS;
}

uint8_t HALAL_state_estimation_start(void) {
    if (HAL_UARTEx_ReceiveToIdle_IT(&state_estimation_uart, state_uart_rx_buf, sizeof(state_uart_rx_buf)) != HAL_OK) {
        return RET_FAILURE;
    }

    if (HAL_UART_Transmit(&state_estimation_uart, "GO", 2, HAL_MAX_DELAY) != HAL_OK) {
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

void HALAL_STATE_ESTIMATION_UART_ISR(void) {
    HAL_UART_IRQHandler(&state_estimation_uart);
}

void state_estimation_uart_rx_event_isr(uint16_t size, BaseType_t *xHigherPriorityTaskWoken) {
    static uint8_t state_bytes_received = 0;

    uint16_t size_to_recieve = size;
    
    /* Clamp initial size to recieve to not exceed STATE_PACKET_SIZE */
    if (state_bytes_received + size > HALAL_STATE_ESTIMATION_PACKET_SIZE) {
        size_to_recieve = HALAL_STATE_ESTIMATION_PACKET_SIZE - state_bytes_received;
    }

    /* Fill state buffer */
    memcpy(state_bytes + state_bytes_received, state_uart_rx_buf, size_to_recieve);
    state_bytes_received += size_to_recieve;

    /* If a full packet is recieved, update the rocket state */
    if (state_bytes_received == HALAL_STATE_ESTIMATION_PACKET_SIZE) {
        HALAL_state_estimation_callback(state_bytes, HALAL_STATE_ESTIMATION_PACKET_SIZE, xHigherPriorityTaskWoken);

        xTaskNotifyFromISR(g_state_tx_task_handle, SEND_STATE_NOTIFICATION_BIT, eSetBits, xHigherPriorityTaskWoken);
        xTaskNotifyFromISR(g_state_flash_task_handle, FLASH_STATE_NOTIFICATION_BIT, eSetBits, xHigherPriorityTaskWoken);
        
        state_bytes_received = 0;
    }

    uint16_t remaining_size = size - size_to_recieve;

    /* Copy remaining data */
    memcpy(state_bytes + state_bytes_received, state_uart_rx_buf + size_to_recieve, remaining_size);
    state_bytes_received += remaining_size;

    HAL_UARTEx_ReceiveToIdle_IT(&state_estimation_uart, state_uart_rx_buf, sizeof(state_uart_rx_buf));
}