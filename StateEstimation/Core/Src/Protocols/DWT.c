#include "DWT.h"

static uint32_t cpu_freq_mhz = 224;  

void DWT_Init(void) {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; 
        DWT->CYCCNT = 0; 
    }
}

/**
 * @brief delays system thread by time in microseconds
 * @param microseconds time in microseconds to delay
*/
void delay_us(uint32_t microseconds) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = microseconds * (SystemCoreClock / 1000000);
    
    // Handle overflow case
    while ((uint32_t)(DWT->CYCCNT - startTick) < delayTicks);
}

uint32_t DWT_GetMicros(void) {
    return DWT->CYCCNT / cpu_freq_mhz;
}

float32_t DWT_TicksToSeconds(uint32_t ticks) {
    return (float32_t)ticks / (float32_t)(cpu_freq_mhz * 1000000);
}