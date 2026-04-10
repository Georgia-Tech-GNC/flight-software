#pragma once
#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "main.h"
#include "arm_math.h"
#include <stdbool.h>
#include "assert.h"

#define ARR 64000
#define APB1_TIMER_FREQ 96000000
#define TIMER_PRESC 29

//#define COUNTS_PER_US (APB1_TIMER_FREQ / (TIMER_PRESC + 1) / 1e6)
#define US_TO_COUNTS 3.2
static_assert(APB1_TIMER_FREQ == 96000000, "If APB1 frequency is not this, adjust US_TO_COUNTS");
static_assert(TIMER_PRESC == 29, "If timer prescalar frequency is not this, adjust US_TO_COUNTS");

/*
PWM0 = PA15, TIM2_CH1
PWM1 = PB3, TIM2_CH2
PWM2 = PB10, TIM2_CH3
PWM3 = PB7, TIM4_CH2
PWM4 = PC7, TIM3_CH2
*/

typedef struct Servo {
    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;
} servo_t;

/** Initialize servo object and start sending PWM on this TIM channel
 * 
 * @param servo         servo struct to initialize
 * @param htim          timer to use
 * @param channel       channel to use
 * @param initial_pos   desired initial servo position in ticks
 */
void servo_init(servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);

/** Sets the current commanded position of this servo in ticks */
void servo_set_pos(servo_t *servo, uint16_t ticks);
