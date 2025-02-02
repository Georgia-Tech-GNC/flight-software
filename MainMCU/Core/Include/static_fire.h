#ifndef STATIC_FIRE_H
#define STATIC_FIRE_H

#include "main.h"
#include "servo.h"
#include "port_config.h"
#include "globals.h"
#include "state_flash.h"

#define BEGIN_STATIC_FIRE_NOTIFICATION_BIT 0x01

#define STATIC_FIRE_ACTUATION_TIME_MS 5000
#define STATIC_FIRE_ACTUATION_RANGE_DEG 20

void static_fire_task(void *args);

#endif