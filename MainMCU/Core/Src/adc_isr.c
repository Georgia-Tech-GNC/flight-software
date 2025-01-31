#include "port_config.h"
#include "globals.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "adc.h"

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
    ADC_Channel channel;

    /* Pointer to the next ADC to start. NULL if the sequence is over */
    ADC_HandleTypeDef *to_start = NULL;
    
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
#endif

    /* Always use mutex with g_current_state */
    //if (xSemaphoreTakeFromISR(g_state_mutex_handle, &xHigherPriorityTaskWoken) == pdTRUE) {
        /* Update the appropriate field in g_current_state */
        char buf[100];
        switch (channel) {
            case ADC_PYRO_I_0:
                sprintf(buf, "ADC_PYRO_I_0: %d\r\n", adc_val);
                //HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
                g_current_state.analog_feedback_data.pyro_0_cont = adc_val;
                break;
            case ADC_PYRO_I_1:
                sprintf(buf, "ADC_PYRO_I_1: %d\r\n", adc_val);
                //HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
                g_current_state.analog_feedback_data.pyro_1_cont = adc_val;
                break;
            case ADC_PYRO_I_2:
                sprintf(buf, "ADC_PYRO_I_2: %d\r\n", adc_val);
                //HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
                g_current_state.analog_feedback_data.pyro_2_cont = adc_val;
                break;
            case ADC_VCC_I:
                sprintf(buf, "ADC_VCC_I: %d\r\n", adc_val);
                //HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
                g_current_state.analog_feedback_data.current_fb_33 = adc_val;
                break;
            case ADC_SERVO_0:
                sprintf(buf, "ADC_SERVO_0: %d\r\n", adc_val);
                //HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
                break;
            case ADC_SERVO_1:
                sprintf(buf, "ADC_SERVO_1: %d\r\n", adc_val);
                //HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
                break;
            case ADC_SERVO_2:
                sprintf(buf, "ADC_SERVO_2: %d\r\n", adc_val);
                //HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
                break;
            case ADC_SERVO_3:
                sprintf(buf, "ADC_SERVO_3: %d\r\n", adc_val);
                //HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
                break;
            case ADC_SERVO_4:
                sprintf(buf, "ADC_SERVO_4: %d\r\n", adc_val);
                HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
                break;
        }

        /* Update timestamp */
        g_current_state.analog_feedback_data.timestamp = xTaskGetTickCount();

        //xSemaphoreGiveFromISR(g_state_mutex_handle, &xHigherPriorityTaskWoken);
    //}

    /* Start next ADC */
    if (to_start != NULL) {
        HAL_ADC_Start_IT(to_start);
    }

    /* FreeRTOS boilerplate */
    //portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
