#ifndef TELEMETRY_RX_H
#define TELEMETRY_RX_H

#include "static_fire.h"
#include <string.h>

#include "telemetry.h"
#include "state_flash.h"

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"

#include "packet_encode.h"
#include "protocol.h"

#include "globals.h"

#define TELEMETRY_RX_MAX_PROCESS_SIZE 64

void telemetry_rx_task(void *args);

#endif