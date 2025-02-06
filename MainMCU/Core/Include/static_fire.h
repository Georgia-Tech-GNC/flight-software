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

#define MOTOR_STARTUP_TIME 3000

#define STATIC_FIRE_ACTUATION_TIME_MS 10000
#define STATIC_FIRE_ACTUATION_RANGE_DEG 30.0
#define STATIC_FIRE_ACTUATION_RANGE_RAD (STATIC_FIRE_ACTUATION_RANGE_DEG * PI / 180.0)

typedef struct {
    uint16_t mid_position;
    uint16_t zero_position;
    uint16_t min_position;
    uint16_t max_position;
} ServoCalibration;

void static_fire_task(void *args);

#endif