#include "servo.h"
#include <stdbool.h>

// Should timer_freq_hz be MACRO? Or per servo?
// timer_freq_hz -> freq_input_clock / (presc + 1)

bool servo_init(
    servo_t *servo,
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    uint32_t timer_freq_hz,
    float angle_range_deg,
    uint16_t min_pulse_us,
    uint16_t max_pulse_us,
    bool reversed
) {
    if (servo == NULL || htim == NULL || timer_freq_hz == 0 || angle_range_deg <= 0.0f) {
        return false;
    }

    if (min_pulse_us >= max_pulse_us) {
        return false;
    }

    servo->htim = htim;
    servo->tim_channel = channel;
    servo->timer_freq_hz = timer_freq_hz;

    servo->min_pulse_us = min_pulse_us;
    servo->max_pulse_us = max_pulse_us;
    servo->angle_range_deg = angle_range_deg;
    servo->zero_pulse_us = servo->min_pulse_us + (servo->max_pulse_us - servo->min_pulse_us) / 2;

    servo->us_per_degree = (servo->max_pulse_us - servo->min_pulse_us) / servo->angle_range_deg;

    servo->reversed = reversed;
    servo->enabled = false;

    return true;
}

bool servo_enable(servo_t *servo) {
    if (servo == NULL) {
        return false;
    }

    if (servo->enabled) {
        return true;
    }

    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(servo->htim, servo->tim_channel);

    if (status != HAL_OK) {
        return false;
    }

    servo->enabled = true;
    return true;
}


bool servo_disable(servo_t *servo) {
    if (servo == NULL) {
        return false;
    }

    if (!servo->enabled) {
        return true;
    }

    HAL_StatusTypeDef status = HAL_TIM_PWM_Stop(servo->htim, servo->tim_channel);

    if (status != HAL_OK) {
        return false;
    }

    servo->enabled = false;
    return true;
}

bool servo_set_pulse_us(servo_t *servo, uint16_t pulse_us) {
    if (servo == NULL || servo->htim == NULL || servo->timer_freq_hz == 0) {
        return false;
    }

    if (pulse_us < servo->min_pulse_us) {
        pulse_us = servo->min_pulse_us;
    }

    if (pulse_us > servo->max_pulse_us) {
        pulse_us = servo->max_pulse_us;
    }

    uint32_t counts = (uint32_t)roundf(((float)pulse_us * (float)servo->timer_freq_hz) / 1000000.0f);

    __HAL_TIM_SET_COMPARE(servo->htim, servo->tim_channel, counts);

    return true;
}

bool servo_set_pos(servo_t *servo, float angle_deg) {
    if (servo == NULL) {
        return false;
    }

    float direction = 1.0f;
    if (servo->reversed) {
        direction = -1.0f;
    } 

    float pulse_us = servo->zero_pulse_us + (direction * angle_deg * servo->us_per_degree);

    if (pulse_us < servo->min_pulse_us) {
        pulse_us = servo->min_pulse_us;
    }

    if (pulse_us > servo->max_pulse_us) {
        pulse_us = servo->max_pulse_us;
    }

    return servo_set_pulse_us(servo, (uint16_t)roundf(pulse_us));
}

bool servo_set_zero(servo_t *servo, uint16_t zero_pulse_us) {
    if (servo == NULL) {
        return false;
    }

    if (zero_pulse_us < servo->min_pulse_us || zero_pulse_us > servo->max_pulse_us) {
        return false;
    }

    servo->zero_pulse_us = zero_pulse_us;

    return true;
}