#ifndef TRACE_RECORDER_H
#define TRACE_RECORDER_H

#include <stdint.h>

/** @brief Initializes the trace recorder module
 * This method must always be called before the FreeRTOS scheduler
 * is started or any trace calls are made.
 *
 * @param idle_task_pointer A pointer to the idle task handle (so that 
 *                          it is ignored during tracing). Can be null
 *                          if not applicable.
 */
void trace_recorder_initialize();

void trace_recorder_log_event(uint16_t event_id);

void trace_recorder_log_data(uint8_t label, float value);

#endif