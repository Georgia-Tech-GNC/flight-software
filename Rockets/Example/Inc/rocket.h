#ifndef ROCKET_H
#define ROCKET_H

#include "lib.h"

extern RocketState g_current_state;

void run_rocket_task(void *args);

#endif