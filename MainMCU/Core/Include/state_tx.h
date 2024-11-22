#ifndef STATE_TX_H
#define STATE_TX_H

#include "FreeRTOS.h"
#include "task.h"

#include "string.h"

#include "protocol.h"
#include "globals.h"

#include "telemetry.h"

#define TX_FREQ_HZ 5
#define BEGIN_STATE_TX_NOTIFICATION_BIT 0x01

void state_tx_task(void *args);

#endif