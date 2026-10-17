#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

/* Copied from legacy driver
PWM0 = PA15, TIM2_CH1
PWM1 = PB3, TIM2_CH2
PWM2 = PB10, TIM2_CH3
PWM3 = PB7, TIM4_CH2
PWM4 = PC7, TIM3_CH2
*/

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;

    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    uint16_t zero_pulse_us;
    float angle_range_deg;

    uint32_t timer_freq_hz;

    float us_per_degree;

    bool reversed;
    bool enabled;
} servo_t;

/**
 * Initialize a servo motor configuration.
 *
 * Configures the servo parameters without starting PWM output.
 * The timer must be configured separately before enabling the servo.
 * PWM output remains disabled after initialization.
 *
 * @param servo Pointer to the servo instance to initialize.
 * @param htim Pointer to the configured HAL timer handle.
 * @param channel Timer PWM channel (TIM_CHANNEL_x).
 * @param timer_freq_hz freq_input_clock / (presc + 1), in Hz.
 * @param angle_range_deg Total angular range in degrees (e.g., 90 for -45 to +45 degrees).
 * @param min_pulse_us Minimum allowed pulse width in microseconds.
 * @param max_pulse_us Maximum allowed pulse width in microseconds.
 * @param reversed True to reverse the angular direction.
 *
 * @return true if initialization succeeds, false otherwise.
 */
bool servo_init(
    servo_t *servo,
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    uint32_t timer_freq_hz,
    float angle_range_deg,
    uint16_t min_pulse_us,
    uint16_t max_pulse_us,
    bool reversed
); 

/**
 * Enable PWM output for the servo.
 *
 * @param servo Pointer to the servo instance.
 * @return true on success, false on failure.
 */
bool servo_enable(servo_t *servo);

/**
 * Disable PWM output for the servo.
 *
 * @param servo Pointer to the servo instance.
 * @return true on success, false on failure.
 */
bool servo_disable(servo_t *servo);

/**
 * Set the servo position in degrees (converts to PWM pulse and calls servo_set_pulse_us()).
 * Does NOT start PWM generation
 * Angles outside the configured limits are clamped.
 * 
 * @param servo Pointer to the servo instance.
 * @param angle_deg Desired angle relative to the zero position.
 * @return true on success, false on failure.
 */
bool servo_set_pos(servo_t *servo, float angle_deg);

/**
 * Set the servo PWM pulse width directly.
 * Pulse widths outside the configured limits are clamped.
 *
 * @param servo Pointer to the servo instance.
 * @param pulse_us Requested pulse width in microseconds.
 * @return true on success, false on failure.
 */
bool servo_set_pulse_us(servo_t *servo, uint16_t pulse_us);

/**
 * Configure the servo's zero-angle pulse width.
 *
 * @param servo Pointer to the servo instance.
 * @param zero_pulse_us Pulse width for 0 degrees, in microseconds.
 * @return true if the value is valid, false otherwise.
 */
bool servo_set_zero(servo_t *servo, uint16_t zero_pulse_us);

#endif