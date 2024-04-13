/**
  ******************************************************************************
  * @file    servo.h
  * @author  Kanav Chugh
  * @brief   Source file for servo methods
  *         
  ******************************************************************************
  * @attention
  *
  * 
  * 
  *
  * 
  * 
  * 
  *  
  *
  ******************************************************************************
  */

#include "../Inc/servo.h"

/**
 * @param x angle instance
 * @param in_min minimum input
 * @param in_max maximum output
 * @param out_min actual minimum
 * @param out_max actual maximum
 * @returns truncated range from out to in
*/
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * This function sets the angle of a servo with a PWM output. Range is 0 through 100 degrees
 * @param angle input angle
 * @param servo servo
 * 
*/
void servo_write(uint8_t angle, __IO uint32_t servo) {
  if (angle < 0) {
    angle = 0;
  }
	if (angle > 100) {
    angle = 100;
  }
  servo = map(angle, MIN_ANGLE, MAX_ANGLE, MIN_TICK, MAX_TICK);
}