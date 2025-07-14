#include "port_config.h"
#include "globals.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "adc.h"
#include "string.h"

/**
 * This interrupt is called every time a single ADC conversion is completed.
 * Through adc[1-3]_conv_ptr, we keep track of which channel we are currently
 * converting. We then store the value in the appropriate field of the
 * analog_feedback_data struct in the global state. If we have finished
 * converting all channels in the sequence, we start the next ADC.
 * 
 * The sequence of channels for each ADC is defined in port_config.h as well
 * as the sequence of ADCs to convert.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    static uint16_t adc1_conv_ptr = 0;
    static uint16_t adc2_conv_ptr = 0;
    static uint16_t adc3_conv_ptr = 0;

    /* FreeRTOS boilerplate */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Value of the conversion */
    uint16_t adc_val = HAL_ADC_GetValue(hadc);

    /* ID of the channel being converted */
    RocketADCChannel channel;

    /**
     * For each ADC, get the channel being converted and increment the
     * corresponding pointer. If the pointer is at the end of the sequence,
     * set to_start to the next ADC to start.
     */
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

_start != NULL) {
        HAL_ADC_Start_IT(to_start);
    }
}
