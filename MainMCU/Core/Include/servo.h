#pragma once
#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "main.h"
#include "arm_math.h"
#include "FreeRTOS.h"
#include <stdbool.h>

#define ARR 64000
#define APB1_TIMER_FREQ 96000000
#define TIMER_PRESC 29

#define COUNTS_PER_US (APB1_TIMER_FREQ / (TIMER_PRESC + 1) / 1e6)


#define MIN_PULSE_WIDTH_US 500
#define MAX_PULSE_WIDTH_US 2500

#define SERVO_RANGE (SERVO_RANGE_DEG/180.0 * PI)
#define SERVO_RANGE_DEG (81.0/255.0 * 355.0)

#define CALI_RANGE (PI/2)

#define MAX_SERVO_DEGREES_PER_SEC 50.0
/* Degrees / second * 1 second / msToTicks(1000ms)ticks * (MIN - MAX)us / SERVO_RANGE_DEG * COUNTS_PER_US counts / 1 us*/
#define MAX_SERVO_PWM_SPEED (MAX_SERVO_DEGREES_PER_SEC / pdMS_TO_TICKS(1000) * (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) / SERVO_RANGE_DEG * COUNTS_PER_US)
//#define MAX_SERVO_PWM_SPEED 1
/*
PWM0 = PA15, TIM2_CH1
PWM1 = PB3, TIM2_CH2
PWM2 = PB10, TIM2_CH3
PWM3 = PB7, TIM4_CH2
PWM4 = PC7, TIM3_CH2
*/

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t tim_channel;
  uint16_t adc_zero;
  double angle_zero;
  uint16_t setpoint_pwm;
  uint16_t current_pwm;
  uint16_t adc_range[2];
  bool invert;
} Servo_T;

// Initialize Servo Struct
void servo_init(Servo_T *servo, TIM_HandleTypeDef *htim, uint32_t channel, bool invert);

// Enable Servo Output
void servo_enable(Servo_T *servo);

// Disable Servo Output
void servo_disable(Servo_T *servo);

// Command servo angle
uint16_t servo_set_angle(Servo_T *servo, double angle_rad);

/** Command the servo angle via a number from 0 to 1 as a proportion of the full pwm range
 * This method disregards zero values or servo invert status
 */
uint16_t servo_set_angle_from_direct_ratio(Servo_T *servo, double ratio);

// Get Commanded Angle PWM
double servo_get_angle(Servo_T *servo);

// Convert an adc reading into a servo deflection from the zero point
double servo_adc_to_rad(Servo_T *servo, uint16_t adc_value);

uint16_t servo_rad_to_adc(Servo_T *servo);

// Go to callibration points
void servo_go_to_calibration_start(Servo_T *servo);
void servo_go_to_calibration_end(Servo_T *servo);

// Perform zero point calculation
void servo_set_zero(Servo_T *servo, uint16_t adc_start, uint16_t adc_end, uint16_t adc_zero);

void update_servo_true_command_position(Servo_T *servo, TickType_t update_period);