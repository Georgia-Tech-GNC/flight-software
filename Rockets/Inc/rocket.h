#ifndef ROCKET_H
#define ROCKET_H

#include "stdint.h"


#ifdef ROCKET_JET_VANES
    #include "jet_vanes.h"
    extern RocketStateStruct g_current_state;

#endif

uint8_t rocket_init(void);
uint8_t rocket_start(void);

#endif