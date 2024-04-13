#ifndef __TIMEBASE_H__
#define __TIMEBASE_H__

#include <stdint.h>


#define ONE_SEC_LOAD    16000000
#define MAX_DELAY       0xFFFFFFFFU

#define CTRL_ENABLE (1U << 0)
#define CTRL_CLKSOURCE (1U << 2)
#define CTRL_TICKINT    (1U << 1)
#define CTRL_COUNTFLAG  (1U << 16)

volatile uint32_t curr_tick;
volatile uint32_t curr_tick_p; 
volatile uint32_t tick_freq = 1;



void timebase_init(void);
void SysTick_Handler(void);
void delay(uint32_t seconds);
uint32_t get_tick(void);
void tick_increment(void);

#endif