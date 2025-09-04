#ifndef UART_ISR_H
#define UART_ISR_H

#include <stdint.h>
#include "halal.h"
#include "FreeRTOS.h"

#ifdef HALAL_RADIO_UART_ENABLED
void radio_uart_rx_event_isr(uint16_t size, BaseType_t *xHigherPriorityTaskWoken);
#endif

#ifdef HALAL_STATE_ESTIMATION_UART_ENABLED
void state_estimation_uart_rx_event_isr(uint16_t size, BaseType_t *xHigherPriorityTaskWoken);
#endif

#endif
