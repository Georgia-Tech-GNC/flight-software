/**
 * @file DWT.h
 * @brief Data Watchpoint and Trace (DWT) utility functions
 * @author Kanav Chugh
 *
 * @details This module provides functions for utilizing the ARM Cortex-M DWT peripheral
 *          for precise timing and delay operations. The DWT cycle counter is used to
 *          implement microsecond delays and timing measurements. Functions include
 *          initialization, microsecond delays, and time conversion utilities.
 *
 * @note CPU frequency is configured for 224 MHz operation
 *
 */

#include "DWT.h"

/**
 * @brief Global variable storing CPU frequency in MHz
 */
static uint32_t cpu_freq_mhz = 224;  

/**
 * @brief Initializes the DWT (Data Watchpoint and Trace) peripheral
 * @details Enables the cycle counter by setting appropriate bits in DWT CTRL
 *         and CoreDebug DEMCR registers. If cycle counter is already enabled,
 *         no action is taken.
 * @return None
 */
void DWT_Init(void) {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; 
        DWT->CYCCNT = 0; 
    }
}

/**
 * @brief Delays system thread by specified time in microseconds
 * @param microseconds Time in microseconds to delay
 * @details Uses DWT cycle counter to create precise microsecond delays.
 *          Handles counter overflow cases.
 * @return None
 */
void delay_us(uint32_t microseconds) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = microseconds * (SystemCoreClock / 1000000);
    
    // Handle overflow case
    while ((uint32_t)(DWT->CYCCNT - startTick) < delayTicks);
}

/**
 * @brief Gets the current system time in microseconds
 * @return Current system time in microseconds based on DWT cycle counter
 * @details Converts DWT cycle count to microseconds using CPU frequency
 */
uint32_t DWT_GetMicros(void) {
    return DWT->CYCCNT / cpu_freq_mhz;
}

/**
 * @brief Converts DWT ticks to seconds
 * @param ticks Number of DWT cycle counter ticks to convert
 * @return Time in seconds as a floating-point number
 * @details Converts raw DWT cycle counts to seconds using CPU frequency
 */
float32_t DWT_TicksToSeconds(uint32_t ticks) {
    return (float32_t)ticks / (float32_t)(cpu_freq_mhz * 1000000);
}