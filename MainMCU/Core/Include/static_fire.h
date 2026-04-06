#ifndef STATIC_FIRE_H
#define STATIC_FIRE_H

#include "main.h"
#include "servo.h"
#include "port_config.h"
#include "globals.h"
#include "state_flash.h"
#include "state_tx.h"

#define ZERO_SERVOS_NOTIFICATION_BIT 0x01
#define BEGIN_STATIC_FIRE_NOTIFICATION_BIT 0x02

#define KP 0.05
#define KI 0
#define KD 0

#define MOTOR_STARTUP_TIME 3000

#define STATIC_FIRE_ACTUATION_TIME_MS 10000.0
#define STATIC_FIRE_ACTUATION_RANGE_DEG 30.0
#define STATIC_FIRE_ACTUATION_RANGE_RAD (STATIC_FIRE_ACTUATION_RANGE_DEG * PI / 180.0)


void static_fire_task(void *args);

#endif