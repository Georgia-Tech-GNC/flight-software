#include "servo.h"

#define INVERT(inv) (inv ? -1 : 1)

// Initialize Servo Struct
void servo_init(Servo_T *servo, TIM_HandleTypeDef *htim, uint32_t channel, bool invert) {
  servo->htim = htim;
  servo->tim_channel = channel;
  servo->setpoint = 0;
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
  servo->setpoint = angle_radians;
  double real_angle = (INVERT(servo->invert) * angle_radians) + servo->angle_zero;
  uint16_t count = ((real_angle + SERVO_RANGE/2)/SERVO_RANGE * (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) + MIN_PULSE_WIDTH_US) * COUNTS_PER_US;
  char buf[100];
  sprintf(buf, "Setting servo to %f degrees, %d counts\r\n", angle_radians, count);
  HAL_UART_Transmit(&debug_uart, buf, strlen(buf), HAL_MAX_DELAY);
  __HAL_TIM_SET_COMPARE(servo->htim, servo->tim_channel, count);
  return count;
}

// Get servo set angleservo
double servo_get_angle(Servo_T *servo) {
  return servo->setpoint;
}

double servo_adc_to_rad(Servo_T *servo, uint16_t adc_value) {
  return INVERT(servo->invert) * CALI_RANGE/(servo->adc_range[1] - servo->adc_range[0]) * (adc_value - servo->adc_zero);
}

uint16_t servo_rad_to_adc(Servo_T *servo) {
  return ((double) servo->setpoint / INVERT(servo->invert) * (servo->adc_range[1] - servo->adc_range[0]) / CALI_RANGE) + servo->adc_zero;
}

// Go to callibration points
void servo_go_to_calibration_start(Servo_T *servo) {
  __HAL_TIM_SET_COMPARE(servo->htim, servo->tim_channel, ((SERVO_RANGE - CALI_RANGE)/ (2* SERVO_RANGE) * (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) + MIN_PULSE_WIDTH_US) * COUNTS_PER_US);
}

void servo_go_to_calibration_end(Servo_T *servo) {
  __HAL_TIM_SET_COMPARE(servo->htim, servo->tim_channel, ((SERVO_RANGE + CALI_RANGE)/ (2* SERVO_RANGE) * (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) + MIN_PULSE_WIDTH_US) * COUNTS_PER_US);
}

// Perform zero point calculation
void servo_set_zero(Servo_T *servo, uint16_t adc_start, uint16_t adc_end, uint16_t adc_zero) {__HAL_TIM_SET_COMPARE(servo->htim, servo->tim_channel, ((SERVO_RANGE - CALI_RANGE)/ (2 * SERVO_RANGE) * (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) + MIN_PULSE_WIDTH_US) * COUNTS_PER_US);
  servo->adc_range[0] = adc_start;
  servo->adc_range[1] = adc_end;
  servo->adc_zero = adc_zero;
  servo->angle_zero = CALI_RANGE/(adc_end - adc_start) * (adc_zero - (adc_start + adc_end)/2);
}