#include "adc_convert.h"

void adc_convert_task(void *args) {
    uint32_t notification_value = 0;
    while ((notification_value & BEGIN_ADC_NOTIFICATION_BIT) == 0) {
        xTaskNotifyWait(0, BEGIN_ADC_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
    }

    while (1) {
        HAL_ADC_Start_IT(FIRST_ADC);
        vTaskDelay(pdMS_TO_TICKS(1000 / ADC_CONVERT_FREQ_HZ));
    }
}