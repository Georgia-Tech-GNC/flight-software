#include "adc_convert.h"
#include "string.h"

void adc_convert_task(void *args) {
    vTaskDelay(100);
    HAL_ADC_Start_IT(FIRST_ADC);
    vTaskDelay(100);
    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "ADC Convert Task\r\n", 18, HAL_MAX_DELAY);
        char buf[100];
        sprintf(buf, "Pyro Continuity: %d %d %d\r\n", g_current_state.analog_feedback_data.pyro_0_cont, g_current_state.analog_feedback_data.pyro_1_cont, g_current_state.analog_feedback_data.pyro_2_cont);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
        sprintf(buf, "Current Feedback: %d\r\n", g_current_state.analog_feedback_data.current_fb_33);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
        xSemaphoreGive(g_state_mutex_handle);
    }

    uint32_t notification_value = 0;
    while ((notification_value & BEGIN_ADC_NOTIFICATION_BIT) == 0) {
        xTaskNotifyWait(0, BEGIN_ADC_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
    }

    while (1) {
        HAL_ADC_Start_IT(FIRST_ADC);
        vTaskDelay(pdMS_TO_TICKS(1000 / ADC_CONVERT_FREQ_HZ));
    }
}