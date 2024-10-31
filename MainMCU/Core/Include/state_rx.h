#ifndef STATE_RX_H

#include "stdint.h"

void state_rx_task(void *args);

#define STATE_STRUCT_SIZE 14
#define STATE_SEMAPHORE_TIMEOUT 10

#define STATE_RX_H
#endif