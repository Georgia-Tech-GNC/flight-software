#include "servo.h"

#define INVERT(inv) (inv ? -1 : 1)

// Initialize Servo Struct
void servo_init(Servo_T *servo, TIM_HandleTypeDef *htim, uint32_t channel, bool invert) {
  servo->htim = htim;
  servo->tim_channel = channel;
  servo->setpoint_pwm = 0;
  servo->current_pwm = 0;
  servo->angle_zero = 0;
};

// Enable Servo
void servo_enable(Servo_T *servo) {
  HAL_TIM_PWM_Start(servo->htim, servo->tim_channel);
}

// Disable Servo
void servo_disable(Servo_T *servo) {
  HAL_TIM_PWM_Stop(servo->htim, servo->tim_channel);
}
#include "port_config.h"
#include "globals.h"

// Set servo angle
uint16_t servo_set_angle(Servo_T *servo, double angle_radians) {
  double real_angle = (INVERT(servo->invert) * angle_radians) + servo->angle_zero;
  uint16_t desired_pwm = ((real_angle + SERVO_RANGE/2)/SERVO_RANGE * (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) + MIN_PULSE_WIDTH_US);
  servo->setpoint_pwm = desired_pwm;
  char augh[100];
  //sprintf(augh, "Setpoint pwm: %d, angle_zero %d\r\n", desired_pwm, servo->angle_zero);
  HAL_UART_Transmit(&debug_uart, augh, strlen(augh), HAL_MAX_DELAY);
  return desired_pwm * COUNTS_PER_US;
}

uint16_t servo_set_angle_from_direct_ratio(Servo_T *servo, double ratio) {
  servo->setpoint_pwm = ratio*(MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) + MIN_PULSE_WIDTH_US;
  return servo->setpoint_pwm * COUNTS_PER_US;
}

// Get servo setpoint angle in radians
double servo_get_angle(Servo_T *servo) {
  uint16_t pwm_offset = servo->setpoint_pwm - MIN_PULSE_WIDTH_US;
  double true_angle_rad = pwm_offset * (SERVO_RANGE / (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US)) - SERVO_RANGE/2;

  return INVERT(servo->invert) * (true_angle_rad - servo->angle_zero);
}

double servo_adc_to_rad(Servo_T *servo, uint16_t adc_value) {
  return INVERT(servo->invert) * CALI_RANGE/(servo->adc_range[1] - servo->adc_range[0]) * (adc_value - servo->adc_zero);
}

uint16_t servo_rad_to_adc(Servo_T *servo) {
  uint16_t pwm_offset = servo->setpoint_pwm - MIN_PULSE_WIDTH_US;
  double true_angle_rad = pwm_offset * (SERVO_RANGE / (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US)) - SERVO_RANGE/2;

  return ((double) true_angle_rad * INVERT(servo->invert) * (servo->adc_range[1] - servo->adc_range[0]) / CALI_RANGE) + servo->adc_zero;
}

// Go to callibration points
void servo_go_to_calibration_start(Servo_T *servo) {
  servo_set_angle_from_direct_ratio(servo, (SERVO_RANGE - CALI_RANGE)/ (2* SERVO_RANGE));
}

void servo_go_to_calibration_end(Servo_T *servo) {
  servo_set_angle_from_direct_ratio(servo, (SERVO_RANGE + CALI_RANGE)/ (2* SERVO_RANGE));
}

// Perform zero point calculation
void servo_set_zero(Servo_T *servo, uint16_t adc_start, uint16_t adc_end, uint16_t adc_zero) {
  servo->adc_range[0] = adc_start;
  servo->adc_range[1] = adc_end;
  servo->adc_zero = adc_zero;
  servo->angle_zero = CALI_RANGE/(adc_end - adc_start) * (adc_zero - (adc_start + adc_end)/2);
}

void update_servo_true_command_position(Servo_T *servo, TickType_t update_period) {
  if (servo->current_pwm == servo->setpoint_pwm) return; // Servo already at correct position

  uint16_t max_travel = (update_period * MAX_SERVO_PWM_SPEED);
  int16_t desired_travel = (int16_t) servo->setpoint_pwm - (int16_t) servo->current_pwm;
  if (desired_travel > 0) {
    servo->current_pwm += (max_travel < desired_travel) ? max_travel : desired_travel;
  } else {
    servo->current_pwm -= (max_travel < -desired_travel) ? max_travel : -desired_travel;
  }

  // Set servo position
  char buf[100];
  sprintf(buf, "Setting servo to %d pwm, target %d\r\n", servo->current_pwm, servo->setpoint_pwm);
  HAL_UART_Transmit(&debug_uart, buf, strlen(buf), HAL_MAX_DELAY);
  __HAL_TIM_SET_COMPARE(servo->htim, servo->tim_channel, servo->current_pwm * COUNTS_PER_US);
}