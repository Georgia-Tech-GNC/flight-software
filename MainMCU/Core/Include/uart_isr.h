#ifndef UART_ISR_H
#define UART_ISR_H

#include "telemetry_rx.h"
#include "port_config.h"
#include "globals.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "run_controls.h"
#include "state_flash.h"
#include "state_tx.h"

#define STATE_PACKET_SIZE 118

int begin_uart_listen(void);

#endif