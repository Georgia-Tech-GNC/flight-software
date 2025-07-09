#include "uart.h"
#include "stm32f4xx_hal.h"
#include "util.h"

UART_HandleTypeDef g_uart2;
UART_HandleTypeDef g_uart3;
UART_HandleTypeDef g_uart6;

static uint8_t uart2_init(void);
static uint8_t uart3_init(void);
static uint8_t uart6_init(void);

static void uart2_msp_init(void);
static void uart3_msp_init(void);
static void uart6_msp_init(void);

uint8_t uart_init(void) {
    if (!uart2_init()) return FAILURE;
    if (!uart3_init()) return FAILURE;
    if (!uart6_init()) return FAILURE;

    return SUCCESS;
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART2) uart2_msp_init();
    if (huart->Instance == USART3) uart3_msp_init();
    if (huart->Instance == USART6) uart6_msp_init();
}

void USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&g_uart2);
}

void USART3_IRQHandler(void) {
    HAL_UART_IRQHandler(&g_uart3);
}

void USART6_IRQHandler(void) {
    HAL_UART_IRQHandler(&g_uart6);
}

static uint8_t uart2_init(void) {
    g_uart2.Instance = USART2;
    g_uart2.Init.BaudRate = 57600;
    g_uart2.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart2.Init.StopBits = UART_STOPBITS_1;
    g_uart2.Init.Parity = UART_PARITY_NONE;
    g_uart2.Init.Mode = UART_MODE_TX_RX;
    g_uart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart2.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&g_uart2) != HAL_OK) {
        return FAILURE;
    }

    return SUCCESS;
}

static void uart2_msp_init(void) {
    __HAL_RCC_USART2_CLK_ENABLE();

    /*
        PD5 -> USART2_TX
        PD6 -> USART2_RX
    */

    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF7_USART2;

    HAL_GPIO_Init(GPIOD, &gpio_init);

    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}
 
static uint8_t uart3_init(void) {
    g_uart3.Instance = USART3;
    g_uart3.Init.BaudRate = 115200;
    g_uart3.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart3.Init.StopBits = UART_STOPBITS_1;
    g_uart3.Init.Parity = UART_PARITY_NONE;
    g_uart3.Init.Mode = UART_MODE_TX_RX;
    g_uart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart3.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&g_uart3) != HAL_OK) {
        return FAILURE;
    }
   
    return SUCCESS;
}

static void uart3_msp_init(void) {
    __HAL_RCC_USART3_CLK_ENABLE();

    /*
        PD8 -> USART3_TX
        PD9 -> USART3_RX
    */

    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF7_USART3;

    HAL_GPIO_Init(GPIOD, &gpio_init);

    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}
 
static uint8_t uart6_init(void) {
    g_uart6.Instance = USART6;
    g_uart6.Init.BaudRate = 115200;
    g_uart6.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart6.Init.StopBits = UART_STOPBITS_1;
    g_uart6.Init.Parity = UART_PARITY_NONE;
    g_uart6.Init.Mode = UART_MODE_TX_RX;
    g_uart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart6.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&g_uart6) != HAL_OK) {
        return FAILURE;
    }

    return SUCCESS;
}

static void uart6_msp_init(void) {
    __HAL_RCC_USART6_CLK_ENABLE();

    /*
        PC6 -> USART6_TX
        PG9 -> USART6_RX
    */

    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = GPIO_PIN_6;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF8_USART6;

    HAL_GPIO_Init(GPIOC, &gpio_init);

    gpio_init.Pin = GPIO_PIN_9;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF8_USART6;

    HAL_GPIO_Init(GPIOG, &gpio_init);

    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
}