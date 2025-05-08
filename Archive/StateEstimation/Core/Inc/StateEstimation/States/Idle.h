/**
 * @file Idle.h
 * @author Albert Zheng
 * @brief Header file for idle state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#ifndef __IDLE_H__
#define __IDLE_H__

#include "state_est_helpers.h"
#include "stm32h7xx_hal.h"

extern uint8_t ready_message_printed;

void run_idle(UART_HandleTypeDef *huart);

#endif