#include "port_config.h"
#include "globals.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "adc.h"

uint16_t adc1_conv_ptr = 0;
uint16_t adc2_conv_ptr = 0;
uint16_t adc3_conv_ptr = 0;

uint16_t pyro_1_cont_avg_buf[10] = {0};
uint16_t pyro_2_cont_avg_buf[10] = {0};
uint16_t pyro_3_cont_avg_buf[10] = {0};
uint16_t current_fb_33_avg_buf[10] = {0};

uint16_t pyro_1_cont_avg_ptr = 0;
uint16_t pyro_2_cont_avg_ptr = 0;
uint16_t pyro_3_cont_avg_ptr = 0;
uint16_t current_fb_33_avg_ptr = 0;


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
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
                pyro_1_cont_avg_buf[pyro_1_cont_avg_ptr] = adc_val;
                pyro_1_cont_avg_ptr = (pyro_1_cont_avg_ptr + 1) % 10;

                for (int i = 0; i < 10; i++) {
                    rolling_avg += pyro_1_cont_avg_buf[i];
                }

                g_current_state.analog_feedback_data.pyro_0_cont = rolling_avg / 10;
                break;
            case ADC_PYRO_I_1:
                pyro_2_cont_avg_buf[pyro_2_cont_avg_ptr] = adc_val;
                pyro_2_cont_avg_ptr = (pyro_2_cont_avg_ptr + 1) % 10;

                for (int i = 0; i < 10; i++) {
                    rolling_avg += pyro_2_cont_avg_buf[i];
                }

                g_current_state.analog_feedback_data.pyro_1_cont = rolling_avg / 10;
                break;
            case ADC_PYRO_I_2:
                pyro_3_cont_avg_buf[pyro_3_cont_avg_ptr] = adc_val;
                pyro_3_cont_avg_ptr = (pyro_3_cont_avg_ptr + 1) % 10;

                for (int i = 0; i < 10; i++) {
                    rolling_avg += pyro_3_cont_avg_buf[i];
                }

                g_current_state.analog_feedback_data.pyro_2_cont = rolling_avg / 10;
                break;
            case ADC_VCC_I:
                current_fb_33_avg_buf[current_fb_33_avg_ptr] = adc_val;
                current_fb_33_avg_ptr = (current_fb_33_avg_ptr + 1) % 10;
                
                for (int i = 0; i < 10; i++) {
                    rolling_avg += current_fb_33_avg_buf[i];
                }

                g_current_state.analog_feedback_data.current_fb_33 = rolling_avg / 10;
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
