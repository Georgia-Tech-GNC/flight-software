#ifndef PORT_LAYER_H
#define PORT_LAYER_H

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "message_buffer.h"
#include "semphr.h"
#include "task.h"

#include "port_config.h"

#include "sdio.h"
#include "state_rx.h"
#include "telemetry.h"

#include "globals.h"

#include "tests.h"

#define TELEMETRY_RX_BUFFER_SIZE 16
#define STATE_RX_BUFFER_SIZE 256

int port_init(void);
void port_start(void);

#endif