#include "timebase.h"
#include "stm32h755xx.h"


void tick_increment(void) {
    curr_tick += tick_freq;
}

uint32_t get_tick(void) {
    //Most of the time for bare metal is here
    __disable_irq();
    curr_tick_p = curr_tick;
    __enable_irq();


    return curr_tick_p;
}

/**
 * @param seconds # of seconds to delay
*/
void delay(uint32_t seconds) {
    uint32_t start_tick = get_tick();
    uint32_t wait = seconds;
    if (wait < MAX_DELAY) {
        wait += (uint32_t)tick_freq;
    }
    while (get_tick() - start_tick < wait) {}
}



void timebase_init(void) {
    //using systic
    //reload timer with  # of cycles per second
    SysTick->LOAD = ONE_SEC_LOAD - 1;

    //clear systick curr value register
    SysTick->VAL = 0;
    //select internal clock source
    SysTick->CTRL |= CTRL_CLKSOURCE;
    //enables interrupts
    SysTick->CTRL |= CTRL_TICKINT;
    //enable systick
    SysTick->CTRL |= CTRL_ENABLE;

    __enable_irq(); //enable global interrupt
}

void SysTick_Handler(void) {
    tick_increment();
}