#include "port_config.h"
#include "globals.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "adc.h"

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    static uint16_t adc1_conv_ptr = 0;
    static uint16_t adc2_conv_ptr = 0;
    static uint16_t adc3_conv_ptr = 0;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t adc_val = HAL_ADC_GetValue(hadc);
    ADC_Channel channel;

    ADC_HandleTypeDef *to_start = NULL;
    
#ifdef USE_ADC1
    if (hadc->Instance == hadc1.Instance) {
        channel = ADC1_SEQUENCE[adc1_conv_ptr];
        adc1_conv_ptr = (adc1_conv_ptr + 1) % ADC1_N_CHANNELS;

        if (adc1_conv_ptr == 0) {
            to_start = ADC1_NEXT;
        }
    }
#endif

#ifdef USE_ADC2
    if (hadc->Instance == hadc2.Instance) {
        channel = ADC2_SEQUENCE[adc2_conv_ptr];
        adc2_conv_ptr = (adc2_conv_ptr + 1) % ADC2_N_CHANNELS;

        if (adc2_conv_ptr == 0) {
            to_start = ADC2_NEXT;
        }
    }
#endif

#ifdef USE_ADC3
    if (hadc->Instance == hadc3.Instance) {
        channel = ADC3_SEQUENCE[adc3_conv_ptr];
        adc3_conv_ptr = (adc3_conv_ptr + 1) % ADC3_N_CHANNELS;

        if (adc3_conv_ptr == 0) {
            to_start = ADC3_NEXT;
        }
    }
#endif

    if (xSemaphoreTakeFromISR(g_state_mutex_handle, &xHigherPriorityTaskWoken) == pdTRUE) {
        uint32_t rolling_avg = 0;

        switch (channel) {
            case ADC_PYRO_I_0:
                g_current_state.analog_feedback_data.pyro_0_cont = adc_val;
                break;
            case ADC_PYRO_I_1:
                g_current_state.analog_feedback_data.pyro_1_cont = adc_val;
                break;
            case ADC_PYRO_I_2:
                g_current_state.analog_feedback_data.pyro_2_cont = adc_val;
                break;
            case ADC_VCC_I:
                g_current_state.analog_feedback_data.current_fb_33 = adc_val;
                break;
        }

        g_current_state.analog_feedback_data.timestamp = xTaskGetTickCount();
        xSemaphoreGiveFromISR(g_state_mutex_handle, &xHigherPriorityTaskWoken);
    }

    if (to_start != NULL) {
        HAL_ADC_Start_IT(to_start);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
