#include "trace_recorder.h"
#include "trace_backend.h"
#include "string.h"

#define MAX_REGISTERED_TASKS 15
/** Number of tasks that have been registered with a given ID */
static uint32_t g_num_registered_tasks;
/** Array of registed task handles (index corresponds to ID) */
static const void* g_task_handle_registry[MAX_REGISTERED_TASKS];

/** Pointer to the idle task, which should be ignored when logging RTOS events */
static const void* g_idle_task_pointer;

/** Global timestamp of the last send timestamped packet */
static uint32_t g_last_packet_time;

static uint32_t __encode_name(const char* const name) {
    uint32_t value = 0;
    for (int i = 0; i < 5; i++) {
        if (name[i] == '\0') break;

        uint32_t encoded = name[i];
        if (encoded >= 'A' && encoded <= 'Z') encoded -= 'A' - 1;
        else if (encoded >= 'a' && encoded <= 'z') encoded -= 'a' - 1;
        else if (encoded == ' ') encoded = 27;
        else if (encoded == '-') encoded = 28;
        else if (encoded == '_') encoded = 29;
        else encoded = 31;

        value |= encoded << (i * 5);
    }
    return value;
}

static uint32_t get_delta_timestamp() {
    uint32_t delta = trace_backend_get_us_time() - g_last_packet_time;
    g_last_packet_time += delta;
    return (delta > 0xFFF) ? 0xFFF : delta;
}

static uint8_t get_task_id(const void* const task_handle) {
    for (uint32_t i = 0; i < g_num_registered_tasks; i++) {
        if (g_task_handle_registry[i] == task_handle) return i;
    }
    if (g_num_registered_tasks < MAX_REGISTERED_TASKS) {
        g_task_handle_registry[g_num_registered_tasks] = task_handle;
        return g_num_registered_tasks++;
    }
    return 15;
}

void trace_recorder_initialize() {
    uint32_t state = trace_backend_enter_critical();

    g_num_registered_tasks = 0;
    g_last_packet_time = trace_backend_get_us_time();

    trace_backend_exit_critical(state);
}

void trace_recorder_register_task(const void* const task_handle, const char* const task_name, const char* const idle_name) {
    if (strcmp(task_name, idle_name) == 0) {
        uint32_t state = trace_backend_enter_critical();
        g_idle_task_pointer = task_handle;
        trace_backend_exit_critical(state);
    } else {
        uint32_t encoded_name = __encode_name(task_name);
        uint32_t state = trace_backend_enter_critical();

        uint8_t task_id = get_task_id(task_handle);
        if (task_id < MAX_REGISTERED_TASKS) {
            trace_backend_send32(0, (task_id << 25) | encoded_name);
        }

        trace_backend_exit_critical(state);
    }
}

void trace_recorder_enter_task(const void* const task_handle) {
    if (task_handle == g_idle_task_pointer) return;
    int32_t state = trace_backend_enter_critical();
    trace_backend_send16(1, (get_task_id(task_handle) << 12) | get_delta_timestamp());
    trace_backend_exit_critical(state);
}

void trace_recorder_exit_task(const void* const task_handle) {
    if (task_handle == g_idle_task_pointer) return;
    int32_t state = trace_backend_enter_critical();
    trace_backend_send16(2, (get_task_id(task_handle) << 12) | get_delta_timestamp());
    trace_backend_exit_critical(state);
}

void trace_recorder_tick() {
    int32_t state = trace_backend_enter_critical();
    uint32_t delta = trace_backend_get_us_time() - g_last_packet_time;
    if (delta > 2000) {
        g_last_packet_time += delta;
        trace_backend_send16(3, (delta > 0xFFF) ? 0xFFF : delta);
    }
    trace_backend_exit_critical(state);
}

void trace_recorder_log_event(uint16_t event_id) {

}

void trace_recorder_log_data(uint8_t label, float value) {

}

