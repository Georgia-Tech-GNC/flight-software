#ifndef ADC_H
#define ADC_H

#define ADC_NUM_CHANNELS 9
#define LOWPASS_ALPHA 0.00625

typedef enum {
    ADC_PYRO_I_0 = 0,
    ADC_PYRO_I_1 = 1,
    ADC_PYRO_I_2 = 2,
    ADC_VCC_I = 3,
    ADC_SERVO_0 = 4,
    ADC_SERVO_1 = 5,
    ADC_SERVO_2 = 6,
    ADC_SERVO_3 = 7,
    ADC_SERVO_4 = 8,
} ADC_Channel;

void poll_adcs();

#endif