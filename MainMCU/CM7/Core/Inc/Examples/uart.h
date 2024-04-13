#ifndef __UART_H__
#define __UART_H__

#include "stm32h745xx.h"
#include <stdio.h>

#define GPIOAEN         (1U << 0)
#define USART2EN        (1U << 17) 

#define SYS_FREQ        16000000
#define APB1_CLK        SYS_FREQ
#define BAUD_RATE       115200

#define CR1_TE          (1U << 3)
#define CR1_UE          (1U << 13)

#define SR_TXE          (1U << 7)

void uart_tx_init(void);

void uart_write(uint8_t ch);

int __io_putchar(int ch);

static uint32_t uart_compute_baud_rate(uint32_t periph_clk, uint32_t baud_rate);

static void uart_set_baudrate(uint32_t periph_clk, uint32_t baud_rate);

#endif