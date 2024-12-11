/**
 * @file uart.h
 * @brief UART peripheral configuration
 */
#ifndef __UART_H__
#define __UART_H__

#include "stm32h7xx_hal.h"

void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_UART4_Init(void);
void MX_USB_OTG_HS_PCD_Init(void);

// UART handles (used by other modules)
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern DMA_HandleTypeDef hdma_usart3_rx;


#endif /* __UART_H__ */