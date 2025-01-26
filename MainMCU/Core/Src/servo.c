#include "servo.h"

// Initialize Servo Struct
void initServo(Servo_T *servo, TIM_HandleTypeDef *htim, uint32_t tim_channel) {
  servo->htim = htim;
  servo->tim_channel = tim_channel;
}

// Enable Servo
void enableServo(Servo_T *servo) {
  HAL_TIM_PWM_Start(servo->htim, servo->tim_channel);
}

// Disable Servo
void disableServo(Servo_T *servo) {
  HAL_TIM_PWM_Stop(servo->htim, servo->tim_channel);
}

// Command servo angle
void setServoAngle(Servo_T *servo, double angle_radians) {
  __HAL_TIM_SetCompare(servo->htim, servo->tim_channel, (MIN_ANG_COUNT + (MAX_ANG_COUNT - MIN_ANG_COUNT) * ((angle_radians - MIN_ANG_RAD) / (MAX_ANG_RAD - MIN_ANG_RAD))));
}

// Get Commanded Angle
double getCommandedAngle(Servo_T *servo) {
  
  return (__HAL_TIM_GET_COMPARE(servo->htim, servo->tim_channel) - MIN_ANG_COUNT)/(MAX_ANG_COUNT - MIN_ANG_COUNT) * (MAX_ANG_RAD - MIN_ANG_RAD) + MIN_ANG_RAD;
}

// Read servo angle

// PID Controller
double updatePID(PIDController_T *pid, double measurement, double newTime) {
  double error = pid->setPoint - measurement;
  double dt = (newTime - pid->lastTime) / 1000000.0;
  double dedt = (error - pid->lastError) / dt;

  pid->lastError = error;
  pid->integrator += error * dt;
  pid->lastTime = newTime;
  pid->lastOutput = pid->kP * error + pid->kI * pid->integrator + pid->kD * dedt;
  return pid->lastOutput;
}