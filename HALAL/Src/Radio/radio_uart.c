#include "radio.h"
#include "halal.h"
#include "util.h"
#include "rtos_globals.h"

#define RADIO_UART_RX_BUF_SIZE 64

UART_HandleTypeDef radio_uart = {0};
uint8_t radio_uart_rx_buf[RADIO_UART_RX_BUF_SIZE];

uint8_t HALAL_radio_init(void) {
#ifndef HALAL_RADIO_UART_ENABLED
    return RET_FAILURE;
#endif

    radio_uart.Instance = HALAL_RADIO_UART_INSTANCE;
    radio_uart.Init.BaudRate = HALAL_RADIO_UART_BAUDRATE;
    radio_uart.Init.WordLength = HALAL_RADIO_UART_WORDLENGTH;
    radio_uart.Init.StopBits = HALAL_RADIO_UART_STOPBITS;
    radio_uart.Init.Parity = HALAL_RADIO_UART_PARITY;
    radio_uart.Init.Mode = UART_MODE_TX_RX;
    radio_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    radio_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    
    HALAL_RADIO_UART_RCC_EN();

    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = HALAL_RADIO_UART_TX_PIN;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = HALAL_RADIO_UART_AF;

    HAL_GPIO_Init(HALAL_RADIO_UART_TX_PORT, &gpio_init);

    gpio_init.Pin = HALAL_RADIO_UART_RX_PIN;

    HAL_GPIO_Init(HALAL_RADIO_UART_RX_PORT, &gpio_init);

    if (HAL_UART_Init(&radio_uart) != HAL_OK) {
        return RET_FAILURE;
    }

    HAL_NVIC_SetPriority(HALAL_RADIO_UART_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(HALAL_RADIO_UART_IRQn);
   
    return RET_SUCCESS;
}

uint8_t HALAL_radio_start(void) {
    if (HAL_UARTEx_ReceiveToIdle_IT(&radio_uart, radio_uart_rx_buf, sizeof(radio_uart_rx_buf)) != HAL_OK) {
        return RET_FAILURE;
    }

    return RET_SUCCESS;
}

uint8_t HALAL_radio_transmit(const uint8_t *msg, size_t len, uint32_t timeout) {
    if (HAL_UART_Transmit(&radio_uart, msg, len, timeout) == HAL_OK) {
        return RET_SUCCESS;
    }

    return RET_FAILURE;
}

void HALAL_RADIO_UART_ISR() {
    HAL_UART_IRQHandler(&radio_uart);
}

void radio_uart_rx_event_isr(uint16_t size, BaseType_t *xHigherPriorityTaskWoken) {
    xStreamBufferSendFromISR(g_telemetry_rx_sb_handle, radio_uart_rx_buf, size, xHigherPriorityTaskWoken);
    HAL_UARTEx_ReceiveToIdle_IT(&radio_uart, radio_uart_rx_buf, sizeof(radio_uart_rx_buf));
}