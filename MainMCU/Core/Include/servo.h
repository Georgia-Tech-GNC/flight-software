#pragma once
#include <stdint.h>
#include "stm32h7xx_hal.h"

#define ARR 64000
#define MIN_ANG_COUNT 1600
#define MAX_ANG_COUNT 8000
#define MIN_ANG_RAD 0
#define MAX_ANG_RAD 3.1415

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
} Servo_T;

// Initialize Servo Struct
void initServo(Servo_T *servo, TIM_HandleTypeDef *htim, uint32_t channel);

// Enable Servo Output
void enableServo(Servo_T *servo);

// Disable Servo Output
void disableServo(Servo_T *servo);

// Command servo angle
void setServoAngle(Servo_T *servo, double angle_radians);
// Get Commanded Angle
double getCommandedAngle(Servo_T *servo);

// Read servo angle

// PID Controller
typedef struct {
  double kP;
  double kI;
  double kD;
  double lastError;
  double integrator;
  uint32_t lastTime;
  double lastOutput;
  double setPoint;
} PIDController_T;

double updatePID(PIDController_T *pid, double measurement, double newTime);