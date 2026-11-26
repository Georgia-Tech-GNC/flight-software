#ifndef INIT_H
#define INIT_H

#include "servo.h"
#include "rgb_led.h"

extern servo_t servo_pitch;
extern servo_t servo_yaw;
extern rgb_led_t rgb_led;

void init_task(void *_params);

#endif
