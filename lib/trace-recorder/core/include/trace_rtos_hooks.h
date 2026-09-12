#ifndef TRACE_RTOS_HOOKS_H
#define TRACE_RTOS_HOOKS_H

#include "trace_recorder.h"

void trace_recorder_enter_task(const void* const task_handle);
void trace_recorder_exit_task(const void* const task_handle);

#define traceTASK_SWITCHED_IN() trace_recorder_enter_task(pxCurrentTCB)
#define traceTASK_SWITCHED_OUT() trace_recorder_exit_task(pxCurrentTCB)

#endif