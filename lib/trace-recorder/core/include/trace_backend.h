/** @file trace_backend.h
 * @brief Target-specific backend methods to be provided by the user
 * 
 * In order to use the trace recorder library on a target,
 * the methods in this file should be implemented.
 *
 * On ARM chips like the cortex M series, the instruction trace macrocell
 * is recommended for live streaming data. Note that these methods 
 * (except for initialization methods) will always be called from inside 
 * a critical section with interrupts disabled, so slow and/or blocking 
 * options like UART are not recommended.
 */

#ifndef TRACE_BACKEND_H
#define TRACE_BACKEND_H

#include <stdint.h>

/** @brief Perform any required initialization of the trace backend */
void trace_backend_initialize();

/** Get time from a monotonically incrementing microsecond time source.
 * Unsigned 32-bit integer overflows are handled internally.
 */
uint32_t trace_backend_get_us_time();

/** @brief Enters a critical section
 * Note that this method may be called from anywhere in the code,
 * including interrupts and potentially while an RTOS critical section
 * is already active. 
 *
 * @return an optional state value, which will be passed to trace_backend_exit_critical()
 *
 * @see trace_backend_exit_critical()
 */
uint32_t trace_backend_enter_critical();

/** @brief Exits a critical section
 * Note that this method may be called from anywhere in the code,
 * including interrupts and potentially while an RTOS critical section
 * is already active. This method will always be paired with a preceding 
 * call to trace_backend_enter_critical();
 *
 * @param state The optional state value provided by trace_backend_enter_critical()
 *
 * @see trace_backend_enter_critical()
 */
void trace_backend_exit_critical(uint32_t state);

/** @brief Sends an 8-bit data packet over the specified channel.
 * This method will always be called from inside a critical section.
 * 
 * @param channel The channel to use (from 0-31, inclusive)
 * @param data    The 8-bit data to send
 */
void trace_backend_send8(uint8_t channel, uint8_t data);

/** @brief Sends a 16-bit data packet over the specified channel.
 * This method will always be called from inside a critical section.
 * 
 * @param channel The channel to use (from 0-31, inclusive)
 * @param data    The 16-bit data to send
 */
void trace_backend_send16(uint8_t channel, uint16_t data);

/** @brief Sends a 32-bit data packet over the specified channel.
 * This method will always be called from inside a critical section.
 * 
 * @param channel The channel to use (from 0-31, inclusive)
 * @param data    The 32-bit data to send
 */
void trace_backend_send32(uint8_t channel, uint32_t data);

#endif