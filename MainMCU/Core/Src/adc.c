#include "adc.h"
#include "port_config.h"
#include "globals.h"
#include "stdint.h"
#include "string.h"

void poll_adcs() {
    ADC_ChannelConfTypeDef sConfig = {0};

    uint16_t adc_readings[ADC_NUM_CHANNELS] = {0};
#ifdef USE_ADC1

    for (int i = 0; i < ADC1_N_CHANNELS; i ++) {
        sConfig.Channel = ADC1_CHANNELS[i];
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
        sConfig.SingleDiff = ADC_SINGLE_ENDED;
        sConfig.OffsetNumber = ADC_OFFSET_NONE;
        sConfig.Offset = 0;
        sConfig.OffsetSignedSaturation = DISABLE;

        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
            HAL_UART_Transmit(&debug_uart, (uint8_t *) "Error configuring ADC1 channel\r\n", 32, HAL_MAX_DELAY);
        }

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1);

        adc_readings[ADC1_SEQUENCE[i]] = HAL_ADC_GetValue(&hadc1);

        HAL_ADC_Stop(&hadc1);
    }
#endif

#ifdef USE_ADC2
    for (int i = 0; i < ADC2_N_CHANNELS; i ++) {
        sConfig.Channel = ADC2_CHANNELS[i];
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
        sConfig.SingleDiff = ADC_SINGLE_ENDED;
        sConfig.OffsetNumber = ADC_OFFSET_NONE;
        sConfig.Offset = 0;
        sConfig.OffsetSignedSaturation = DISABLE;

        if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK){
            HAL_UART_Transmit(&debug_uart, (uint8_t *) "Error configuring ADC2 channel\r\n", 32, HAL_MAX_DELAY);
        }

        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, 1);

        adc_readings[ADC2_SEQUENCE[i]] = HAL_ADC_GetValue(&hadc2);
        
        HAL_ADC_Stop(&hadc2);
    }
#endif

#ifdef USE_ADC3
    for (int i = 0; i < ADC3_N_CHANNELS; i ++) {
        sConfig.Channel = ADC3_CHANNELS[i];
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
        sConfig.SingleDiff = ADC_SINGLE_ENDED;
        sConfig.OffsetNumber = ADC_OFFSET_NONE;
        sConfig.Offset = 0;
        sConfig.OffsetSignedSaturation = DISABLE;

        if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK){
            HAL_UART_Transmit(&debug_uart, (uint8_t *) "Error configuring ADC3 channel\r\n", 32, HAL_MAX_DELAY);
        }

        HAL_ADC_Start(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, 1);

        adc_readings[ADC3_SEQUENCE[i]] = HAL_ADC_GetValue(&hadc3);
        
        HAL_ADC_Stop(&hadc3);
    }
#endif
    
    static float servo_1_actual = 0;
    static float servo_2_actual = 0;
    static float servo_3_actual = 0;
    static float servo_4_actual = 0;

    if (servo_1_actual == 0) {
        servo_1_actual = adc_readings[ADC_SERVO_1];
    } else {
        servo_1_actual = (1 - LOWPASS_ALPHA) * servo_1_actual + LOWPASS_ALPHA * adc_readings[ADC_SERVO_1];
    }

    char buf[100];
    sprintf(buf, "Servo 1 ADC: %d\r\n", (uint16_t) servo_1_actual);
    //HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
    /*
    sprintf(buf, "Servo 2 ADC: %d\r\n", adc_readings[ADC_SERVO_2]);
    HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);

    sprintf(buf, "Servo 3 ADC: %d\r\n", adc_readings[ADC_SERVO_3]);
    HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);

    sprintf(buf, "Servo 4 ADC: %d\r\n", adc_readings[ADC_SERVO_4]);
    HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, strlen(buf), HAL_MAX_DELAY);
    */
    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        g_current_state.servo_deflections.servo_1_actual = adc_readings[ADC_SERVO_1];
        g_current_state.servo_deflections.servo_2_actual = adc_readings[ADC_SERVO_2];
        g_current_state.servo_deflections.servo_3_actual = adc_readings[ADC_SERVO_3];
        g_current_state.servo_deflections.servo_4_actual = adc_readings[ADC_SERVO_4];

        xSemaphoreGive(g_state_mutex_handle);
    }
}