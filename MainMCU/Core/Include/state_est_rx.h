#ifndef STATE_RX_H
#define STATE_RX_H

#include "stdint.h"

#define STATE_ESTIMATION_BYTES 118

void state_est_rx_task(void *args);

#endif