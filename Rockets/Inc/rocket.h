#ifndef ROCKET_H
#define ROCKET_H

#include "stdint.h"
#include "lib.h"

extern RocketStateStruct g_current_state;

uint8_t rocket_init(void);
uint8_t rocket_start(void);

#endif