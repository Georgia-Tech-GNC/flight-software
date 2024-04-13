/**
  ******************************************************************************
  * @file    servo.h
  * @author  Kanav Chugh
  * @brief   This is the file for basic servo methods
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

#ifndef __SERVO_H__
#define __SERVO_H__


#include "stm32h745xx.h"
#include "core_cm7.h"
#include "core_cm4.h"
#include "stdint.h"

#define MIN_TICK 3500
#define MAX_TICK 10000
#define MIN_ANGLE 0
#define MAX_ANGLE 180

long map(long x, long in_min, long in_max, long out_min, long out_max);
void servo_write(uint8_t angle, __IO uint32_t servo);

#endif