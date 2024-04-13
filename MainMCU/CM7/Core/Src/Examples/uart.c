#include "uart.h"

void uart_tx_init(void) {
   RCC->AHB1ENR |= GPIOAEN;
   //PA2 is moder 2
   //AF needs 4th bit as 0 and left bit is 1
   GPIOA->MODER &= ~(1U<<4);
   GPIOA->MODER |= (1U<<5);
   //To set to TX_LINE, set AF7 to 0111(for alternative function
   //AFR has two items: 0 is AFRL and 1 is AFRH
   GPIOA->AFR[0] |= (1U << 8);
   GPIOA->AFR[0] |= (1U << 9);
   GPIOA->AFR[0] |= (1U << 10);
   GPIOA->AFR[0] &= ~(1U << 11);
   //Enabling clock
   RCC->APB1LENR |= USART2EN;
   //setting baud rate
   uart_set_baudrate(APB1_CLK, BAUD_RATE);
   //enabling USART pins
   USART2->CR1 = CR1_TE; //setting everything to zero except for this
   USART2->CR1 |= CR1_UE;

}

//transmit a character
void uart_write(uint8_t ch) {
    //Make sure transmit data register is empty
    while (!USART2->ISR & SR_TXE) {
        continue;
    }

    USART2->TDR = (ch & 0xFF);
    //Write data to transmit
}

/*Receiving a character*/
uint8_t uart_read(void) {
    while (!USART2->ISR  & SR_TXE) {
        continue;
    }
    return (uint8_t)(USART2->RDR & 0xFF);
}


/* This is a way to use printf */
int __io_putchar(int ch) {
    uart_write(ch);
    return ch;
}

static void uart_set_baudrate(uint32_t periph_clk, uint32_t baud_rate) {
    USART2->BRR = uart_compute_baud_rate(periph_clk, baud_rate);
}

static uint32_t uart_compute_baud_rate(uint32_t periph_clk, uint32_t baud_rate) {
    return (periph_clk + (baud_rate / 2U)) / baud_rate;
}