#ifndef INIT_H
#define INIT_H

#include "servo.h"

extern servo_t servo_pitch;
extern servo_t servo_yaw;

void init_task(void *_params);

#endif
