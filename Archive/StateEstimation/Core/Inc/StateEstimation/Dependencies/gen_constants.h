#ifndef __GEN_CONSTANTS_H_
#define __GEN_CONSTANTS_H_

#include "data_handling.h"
#include "flight_ekf.h"
#include "attitude.h"

extern uint8_t gekf_initialize;
extern uint8_t fekf_initialize;

extern uint16_t rocket_state;
extern uint32_t global_time;
extern float32_t fast_ascent_start_time;
extern float32_t global_time_seconds;
extern uint32_t prev_global_time;
extern int16_t first_iter;
extern int16_t first_slow_ascent_iter;
extern float32_t startTOV;
extern int16_t activatedTOV;
extern float32_t prev_alt;
extern uint8_t iterations;
extern uint8_t has_not_run_fast_ascent;
extern uint8_t first_time;
extern uint8_t start;
extern uint32_t global_time_step;
extern uint8_t signal_received[2];

#endif