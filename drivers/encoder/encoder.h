#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

// Depends on clock config CubeMX
#define ENCODER_PERIOD_CHANNEL TIM_CHANNEL_1
#define ENCODER_PULSE_CHANNEL TIM_CHANNEL_2
#define TIM_ACTIVE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_1

// Timer counter frequency after prescaler, in Hz
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

/**
 * Initialize a PWM encoder configuration.
 *
 * Configures the encoder parameters without starting input capture.
 * Input capture remains disabled after initialization.
 *
 * @param encoder Pointer to the encoder instance to initialize.
 * @param htim Pointer to the configured HAL timer handle.
 * @param period_channel Timer channel measuring the PWM period (TIM_CHANNEL_x).
 * @param pulse_channel Timer channel measuring the HIGH pulse width (TIM_CHANNEL_x).
 * @param timer_freq_hz Timer input clock / (prescaler + 1), in Hz.
 *
 * @return true if initialization succeeds, false otherwise.
 */
bool encoder_init(
    encoder_t *encoder,
    TIM_HandleTypeDef *htim,
    uint32_t period_channel,
    uint32_t pulse_channel,
    uint32_t timer_freq_hz
);

/**
 * Enable PWM input capture for the encoder.
 *
 * Starts the pulse capture channel and enables interrupts on the
 * period capture channel.
 *
 * @param encoder Pointer to the encoder instance.
 * @return true on success, false on failure.
 */
bool encoder_enable(encoder_t *encoder);

/**
 * Disable PWM input capture for the encoder.
 *
 * Stops both capture channels and resets the stored measurements.
 *
 * @param encoder Pointer to the encoder instance.
 * @return true on success, false on failure.
 */
bool encoder_disable(encoder_t *encoder);

/**
 * Get the measured HIGH pulse width in microseconds.
 *
 * Converts the captured pulse counts into microseconds using
 * the configured timer counter frequency.
 *
 * @param encoder Pointer to the encoder instance.
 * @param pulse_us Pointer to store the pulse width in microseconds.
 * @return true if a valid measurement is available, false otherwise.
 */
bool encoder_get_pulse_us(encoder_t *encoder, float *pulse_us);

/**
 * Get the measured PWM signal frequency in Hz.
 *
 * Calculates the frequency from the captured period counts
 * and the configured timer counter frequency.
 *
 * @param encoder Pointer to the encoder instance.
 * @param frequency_hz Pointer to store the frequency in Hz.
 * @return true if a valid measurement is available, false otherwise.
 */
bool encoder_get_frequency(encoder_t *encoder, float *frequency_hz);

/**
 * Get the measured PWM duty cycle as a percentage.
 *
 * Calculates the ratio of the HIGH pulse width to the full
 * PWM period, expressed as a percentage from 0 to 100.
 *
 * @param encoder Pointer to the encoder instance.
 * @param duty_cycle Pointer to store the duty cycle percentage.
 * @return true if a valid measurement is available, false otherwise.
 */
bool encoder_get_duty_cycle(encoder_t *encoder, float *duty_cycle);

/**
 * Process a PWM input capture interrupt.
 *
 * Reads the captured period and pulse width from the timer
 * and updates the encoder measurements. Must be called from
 * HAL_TIM_IC_CaptureCallback() when the configured period
 * channel triggers an interrupt.
 *
 * @param encoder Pointer to the encoder instance.
 * @return true if a valid capture is processed, false otherwise.
 */
bool encoder_capture_callback(encoder_t *encoder);


#endif 