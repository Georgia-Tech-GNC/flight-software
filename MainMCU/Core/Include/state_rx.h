#ifndef STATE_RX_H

#include "stdint.h"

void state_rx_task(void *args);

typedef struct {
    uint32_t timestamp;
    uint16_t counter;
    char message[8];
} StateStruct;

#define STATE_STRUCT_SIZE sizeof(StateStruct)
#define STATE_SEMAPHORE_TIMEOUT 10

#define STATE_RX_H
#endif