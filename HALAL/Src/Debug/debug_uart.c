#include "debug.h"
#include "halal.h"
#include "util.h"

UART_HandleTypeDef debug_uart = {0};

uint8_t HALAL_debug_init(void) {
#ifndef HALAL_DEBUG_UART_ENABLED
    return RET_FAILURE;
#endif

    debug_uart.Instance = HALAL_DEBUG_UART_INSTANCE;
    debug_uart.Init.BaudRate = HALAL_DEBUG_UART_BAUDRATE;
    debug_uart.Init.WordLength = HALAL_DEBUG_UART_WORDLENGTH;
    debug_uart.Init.StopBits = HALAL_DEBUG_UART_STOPBITS;
    debug_uart.Init.Parity = HALAL_DEBUG_UART_PARITY;
    debug_uart.Init.Mode = UART_MODE_TX_RX;
    debug_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    debug_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    
    HALAL_DEBUG_UART_RCC_EN();

    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = HALAL_DEBUG_UART_TX_PIN;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = HALAL_DEBUG_UART_AF;

    HAL_GPIO_Init(HALAL_DEBUG_UART_TX_PORT, &gpio_init);

    gpio_init.Pin = HALAL_DEBUG_UART_RX_PIN;

    HAL_GPIO_Init(HALAL_DEBUG_UART_RX_PORT, &gpio_init);

    if (HAL_UART_Init(&debug_uart) != HAL_OK) {
        return RET_FAILURE;
    }
   
    return RET_SUCCESS;
}

uint8_t HALAL_debug_write(const char *msg, size_t len, uint32_t timeout) {
    if (HAL_UART_Transmit(&debug_uart, (const uint8_t *) msg, len, timeout) == HAL_OK) {
        return RET_SUCCESS;
    }

    return RET_FAILURE;
}