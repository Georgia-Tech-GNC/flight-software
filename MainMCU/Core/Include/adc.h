#ifndef ADC_H
#define ADC_H

/*
#define ADC_NUM_CHANNELS 16

typedef enum {
    ADC_I_SENSE_0 = 0,
    ADC_I_SENSE_1 = 1,
    ADC_I_SENSE_2 = 2,
    ADC_I_SENSE_3 = 3,
    ADC_I_SENSE_4 = 4,
    ADC_SERVO_0 = 5,
    ADC_SERVO_1 = 6,
    ADC_SERVO_2 = 7,
    ADC_SERVO_3 = 8,
    ADC_SERVO_4 = 9,
    ADC_PYRO_I_0 = 10,
    ADC_PYRO_I_1 = 11,
    ADC_PYRO_I_2 = 12,
    ADC_VCC_I = 13,
    ADC_VCC_V = 14,
    ADC_BUCK_V = 15,
} ADC_Channel;
*/

#define ADC_NUM_CHANNELS 4

typedef enum {
    ADC_PYRO_I_0 = 0,
    ADC_PYRO_I_1 = 1,
    ADC_PYRO_I_2 = 2,
    ADC_VCC_I = 3,
} ADC_Channel;

#endif