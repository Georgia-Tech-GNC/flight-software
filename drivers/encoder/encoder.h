#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

// Depends on clock config CubeMX
#define ENCODER_PERIOD_CHANNEL TIM_CHANNEL_1
#define ENCODER_PULSE_CHANNEL TIM_CHANNEL_2
#define TIM_ACTIVE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_1
#define ENCODER_TIMER_FREQ_HZ 3200000U

typedef struct {
    TIM_HandleTypeDef *htim;

    uint32_t period_channel;
    uint32_t pulse_channel;
    uint32_t timer_freq_hz;

    volatile uint32_t period_counts;
    volatile uint32_t pulse_counts;

    bool enabled;
} encoder_t;

bool encoder_init(
    encoder_t *encoder,
    TIM_HandleTypeDef *htim,
    uint32_t period_channel,
    uint32_t pulse_channel,
    uint32_t timer_freq_hz
);

bool encoder_enable(encoder_t *encoder);

bool encoder_disable(encoder_t *encoder);

bool encoder_get_pulse_us(encoder_t *encoder, float *pulse_us);

bool encoder_get_frequency(encoder_t *encoder, float *frequency_hz);

bool encoder_get_duty_cycle(encoder_t *encoder, float *duty_cycle);

bool encoder_capture_callback(encoder_t *encoder);


#endif 