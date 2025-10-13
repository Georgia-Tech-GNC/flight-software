#include "main.h"
#include "log.h"
#include "rocket.h"
#include "halal.h"
#include "rtos_globals.h"
#include "FreeRTOS.h"
#include "semphr.h"

int main(void) {
    if (HALAL_init()) {
        log_printf(LOG_INFO, "HALAL initialized");
    } else {
        log_printf(LOG_ERROR, "HALAL not initialized");
    }
  
    rocket_init();
    rocket_start();
}

void HALAL_radio_callback(uint8_t *radio_data, size_t size, BaseType_t *xHigherPriorityTaskWoken) {
    xStreamBufferSendFromISR(g_telemetry_rx_sb_handle, radio_data, size, xHigherPriorityTaskWoken);
}

void HALAL_adc_convert_callback(uint32_t channel_uuid, uint32_t adc_value, BaseType_t *xHigherPriorityTaskWoken) {
        if (xSemaphoreTakeFromISR(g_state_mutex_handle, xHigherPriorityTaskWoken) == pdPASS) {
        set_adc_value(&g_current_state, (JetVanesADCChannel) channel_uuid, adc_value);
        xSemaphoreGiveFromISR(g_state_mutex_handle, xHigherPriorityTaskWoken);
    }
}

void HALAL_state_estimation_callback(uint8_t *state_bytes, size_t size, BaseType_t *xHigherPriorityTaskWoken) {
    if (xSemaphoreTakeFromISR(g_state_mutex_handle, xHigherPriorityTaskWoken) == pdPASS) {
        update_rocket_state(&g_current_state, state_bytes, size);
        xSemaphoreGiveFromISR(g_state_mutex_handle, xHigherPriorityTaskWoken);
    }
}

void Error_Handler(void) {
    log_printf(LOG_ERROR, "HAL Error Handler entered");
    __disable_irq();
    while (1);
}