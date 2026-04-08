#include "servo.h"
#include "math.h"

// Initialize Servo Struct
void servo_init(servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel) {
  servo->htim = htim;
  servo->tim_channel = channel;

  HAL_TIM_PWM_Start(servo->htim, servo->tim_channel);
};

void servo_set_pos(servo_t *servo, uint16_t ticks) {
    int count = roundf(((float)ticks) * US_TO_COUNTS);
    __HAL_TIM_SET_COMPARE(servo->htim, servo->tim_channel, count);
}
