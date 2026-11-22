#include "port.h"
#include "mfc.h"
#include <stdio.h>

#define MFC_VREF            3.3f //ADC Reference Value
#define MFC_I_ZERO_OFFSET_V 0.0f

//ADC (Digital number) * Vref/2^N
static float mfc_counts_to_volts(uint16_t counts, float vref) {
    return counts * vref / MFC_ADC_MAX_COUNTS;
}

//Voltage divider
static float mfc_fb_to_battery_volts(float v_fb) {
    return v_fb * (MFC_BAT_R_TOP + MFC_BAT_R_BOT) / MFC_BAT_R_BOT;
}

//No offset/resistance
static float mfc_amp_to_current_amps(float v_amp, float zero_offset_v) {
    return (v_amp - zero_offset_v) / (MFC_SHUNT_OHMS * MFC_CUR_GAIN);
}

//read adc 16 times and return average
static uint16_t adc_read_avg(uint32_t channel) {
    ADC_ChannelConfTypeDef cfg = {0};
//ADC pin
    cfg.Channel = channel;
    cfg.Rank = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    cfg.SingleDiff = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset = 0;
    HAL_ADC_ConfigChannel(&feedback_adc, &cfg);

    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        HAL_ADC_Start(&feedback_adc);
        HAL_ADC_PollForConversion(&feedback_adc, HAL_MAX_DELAY);
        sum += HAL_ADC_GetValue(&feedback_adc);
    }
    return (uint16_t) (sum / 16);
}

//Battery voltage feedback circut
float mfc_read_battery_volts(void) {
    float v_fb = mfc_counts_to_volts(adc_read_avg(feedback_vbat_adc_channel), MFC_VREF);
    return mfc_fb_to_battery_volts(v_fb);
}

//Battery Current feedback circuit
float mfc_read_current_amps(void) {
    float v_amp = mfc_counts_to_volts(adc_read_avg(feedback_current_adc_channel), MFC_VREF);
    return mfc_amp_to_current_amps(v_amp, MFC_I_ZERO_OFFSET_V);
}

//Digital signal
bool mfc_pyro_global_enable(void) {
    return HAL_GPIO_ReadPin(PYRO_GLOBAL_EN_GPIO_PORT, PYRO_GLOBAL_EN_PIN) == GPIO_PIN_SET;
}

bool mfc_pyro1_status(void) {
    return HAL_GPIO_ReadPin(PYRO1_CONT_GPIO_PORT, PYRO1_CONT_PIN) == GPIO_PIN_SET;
}

bool mfc_pyro2_status(void) {
    return HAL_GPIO_ReadPin(PYRO2_CONT_GPIO_PORT, PYRO2_CONT_PIN) == GPIO_PIN_SET;
}

//Add enable pyro channel

void mfc_pyro1_set(bool on) {
    HAL_GPIO_WritePin(PYRO1_EN_GPIO_PORT, PYRO1_EN_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void mfc_pyro2_set(bool on) {
    HAL_GPIO_WritePin(PYRO2_EN_GPIO_PORT, PYRO2_EN_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

mfc_metrics_t mfc_get_metrics(void) {
    mfc_metrics_t m;
    float bat = mfc_read_battery_volts();
    float cur = mfc_read_current_amps();

    int bat_cv  = (int)(bat * 100.0f + 0.5f);
    int cur_cma = (int)(cur * 100000.0f + (cur >= 0 ? 0.5f : -0.5f));

    m.battery_volts  = (float)bat_cv / 100.0f;
    m.current_amps   = (float)cur_cma / 100000.0f;
    m.pyro_global_en = mfc_pyro_global_enable();
    m.pyro1_stat     = mfc_pyro1_status();
    m.pyro2_stat     = mfc_pyro2_status();
    return m;
}
