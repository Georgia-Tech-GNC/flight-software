#include "encoder.h"

bool encoder_init(
    encoder_t *encoder,
    TIM_HandleTypeDef *htim,
    uint32_t period_channel,
    uint32_t pulse_channel,
    uint32_t timer_freq_hz
) {
    if (encoder == NULL || htim == NULL) {
        return false;
    }

    encoder->htim = htim;
    encoder->period_channel = period_channel;
    encoder->pulse_channel = pulse_channel;
    encoder->timer_freq_hz = timer_freq_hz;

    encoder->period_counts = 0;
    encoder->pulse_counts = 0;
    encoder->enabled = false;

    return true;
}

bool encoder_enable(encoder_t *encoder) {
    if (encoder == NULL) {
        return false;
    }

    // Start HIGH capture
    if (HAL_TIM_IC_Start(encoder->htim,encoder->pulse_channel) != HAL_OK) {
        return false;
    }

    // Start period capture 
    if (HAL_TIM_IC_Start_IT(encoder->htim, encoder->period_channel) != HAL_OK) {
        HAL_TIM_IC_Stop(encoder->htim, encoder->pulse_channel);
        return false;
    }

    encoder->enabled = true;
    return true;
}

bool encoder_disable(encoder_t *encoder) {
    if (encoder == NULL) {
        return false;
    }

    // Stop period capture 
    if (HAL_TIM_IC_Stop_IT(encoder->htim,encoder->period_channel) != HAL_OK) {
        return false;
    }

    // Stop HIGH capture
    if (HAL_TIM_IC_Stop(encoder->htim,encoder->pulse_channel) != HAL_OK) {
        return false;
    }

    encoder->enabled = false;
    encoder->period_counts = 0;
    encoder->pulse_counts = 0;

    return true;
}

bool encoder_get_pulse_us(encoder_t *encoder, float *pulse_us) {
    if (encoder == NULL || pulse_us == NULL) {
        return false;
    }

    if (encoder->period_counts == 0) {
        return false;
    }

    *pulse_us = ((float)encoder->pulse_counts * 1000000.0f)/ (float)encoder->timer_freq_hz;

    return true;
}

bool encoder_get_frequency(encoder_t *encoder, float *frequency_hz) {
    if (encoder == NULL || frequency_hz == NULL) {
        return false;
    }

    uint32_t period = encoder->period_counts;

    if (period == 0) {
        return false;
    }

    *frequency_hz = (float)encoder->timer_freq_hz / (float)period;

    return true;
}

bool encoder_get_duty_cycle(encoder_t *encoder, float *duty_cycle) {
    if (encoder == NULL || duty_cycle == NULL) {
        return false;
    }

    uint32_t period = encoder->period_counts;
    uint32_t pulse = encoder->pulse_counts;

    if (period == 0 || pulse > period) {
        return false;
    }

    *duty_cycle = ((float)pulse / (float)period) * 100.0f;

    return true;
}

bool encoder_capture_callback(encoder_t *encoder) {
    if (encoder == NULL || !encoder->enabled) {
        return false;
    }

    uint32_t period = HAL_TIM_ReadCapturedValue(encoder->htim, encoder->period_channel);

    uint32_t pulse = HAL_TIM_ReadCapturedValue(encoder->htim, encoder->pulse_channel);

    if (period == 0 || pulse > period) {
        return true;
    }

    encoder->pulse_counts = pulse;
    encoder->period_counts = period;

    return true;
}