#include "static_fire.h"

void init_servos(Servo_T *servos);
void enable_servos(Servo_T *servos);
void disable_servos(Servo_T *servos);
void set_servo_angles(Servo_T *servos, float angle_rad);
void record_servo_zeropoint(Servo_T *servos, ServoCalibration *calibrations, uint16_t **servo_deflections);
void record_servo_midpoint(Servo_T *servos, ServoCalibration *calibrations, uint16_t **servo_deflections);
void record_servo_endpoints(Servo_T *servos, ServoCalibration *calibrations, uint16_t **servo_deflections);
float get_absolute_angle(float angle_rad, ServoCalibration *calibration);
uint16_t angle_to_adc(float angle_rad, ServoCalibration *calibration);

void init_servos(Servo_T *servos) {
    initServo(&servos[0], PWM0_TIMER, PWM0_CHANNEL);
    initServo(&servos[1], PWM1_TIMER, PWM1_CHANNEL);
    initServo(&servos[2], PWM2_TIMER, PWM2_CHANNEL);
    initServo(&servos[3], PWM3_TIMER, PWM3_CHANNEL);
}

void enable_servos(Servo_T *servos) {
    for (int i = 0; i < 4; i++) {
        enableServo(&servos[i]);
    }
}

void disable_servos(Servo_T *servos) {
    for (int i = 0; i < 4; i++) {
        disableServo(&servos[i]);
    }
}

void set_servo_angles(Servo_T *servos, float angle_rad) {
    for (int i = 0; i < 4; i++) {
        setServoAngle(&servos[i], angle_rad);
    }
}

void measure_adc_with_delay(void) {
    HAL_ADC_Start_IT(FIRST_ADC);
    vTaskDelay(pdTICKS_TO_MS(10));
}

void record_servo_zeropoint(Servo_T *servos, ServoCalibration *calibrations, uint16_t **servo_deflections) {
    measure_adc_with_delay();

    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < 4; i++) {
            calibrations[i].zero_position = (*servo_deflections[i]);
        }

        xSemaphoreGive(g_state_mutex_handle);
    }
}

void record_servo_midpoint(Servo_T *servos, ServoCalibration *calibrations, uint16_t **servo_deflections) {
    set_servo_angles(servos, PI / 2);
    vTaskDelay(pdTICKS_TO_MS(1000));
    measure_adc_with_delay();

    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < 4; i++) {
            calibrations[i].mid_position = (*servo_deflections[i]);
        }

        xSemaphoreGive(g_state_mutex_handle);
    }
}

void record_servo_endpoints(Servo_T *servos, ServoCalibration *calibrations, uint16_t **servo_deflections) {
    float minAngle = PI / 2 - STATIC_FIRE_ACTUATION_RANGE_RAD;
    float maxAngle = PI / 2 + STATIC_FIRE_ACTUATION_RANGE_RAD;

    set_servo_angles(servos, minAngle);

    vTaskDelay(pdTICKS_TO_MS(2000));
    measure_adc_with_delay();

    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < 4; i++) {
            calibrations[i].min_position = (*servo_deflections[i]);
        }

        xSemaphoreGive(g_state_mutex_handle);
    }

    set_servo_angles(servos, maxAngle);

    vTaskDelay(pdTICKS_TO_MS(2000));
    measure_adc_with_delay();

    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < 4; i++) {
            calibrations[i].max_position = (*servo_deflections[i]);
        }

        xSemaphoreGive(g_state_mutex_handle);
    }
}

float get_absolute_angle(float angle_rad, ServoCalibration *calibration) {
    float counts_per_rad = (float) (calibration->max_position - calibration->min_position) / (2 * STATIC_FIRE_ACTUATION_RANGE_RAD);
    float offset_rad = (float) (calibration->zero_position - calibration->mid_position) / counts_per_rad;

    return angle_rad + offset_rad;
}

uint16_t angle_to_adc(float angle_rad, ServoCalibration *calibration) {
    float counts_per_rad = (float) (calibration->max_position - calibration->min_position) / (2 * STATIC_FIRE_ACTUATION_RANGE_RAD);

    return (angle_rad - PI / 2) * counts_per_rad + calibration->mid_position;
}

void static_fire_task(void *args) {
    /* Initialize servos */
    Servo_T servo1 = {0};
    Servo_T servo2 = {0};
    Servo_T servo3 = {0};
    Servo_T servo4 = {0};

    Servo_T servos[] = {servo1, servo2, servo3, servo4};

    ServoCalibration servo1_calibration = {0};
    ServoCalibration servo2_calibration = {0};
    ServoCalibration servo3_calibration = {0};
    ServoCalibration servo4_calibration = {0};

    ServoCalibration calibrations[] = {servo1_calibration, servo2_calibration, servo3_calibration, servo4_calibration};

    uint16_t *servo1_deflection = &g_current_state.servo_deflections.servo_1_actual;
    uint16_t *servo2_deflection = &g_current_state.servo_deflections.servo_2_actual;
    uint16_t *servo3_deflection = &g_current_state.servo_deflections.servo_3_actual;
    uint16_t *servo4_deflection = &g_current_state.servo_deflections.servo_4_actual;

    uint16_t *servo_deflections[] = {servo1_deflection, servo2_deflection, servo3_deflection, servo4_deflection};

    init_servos(servos);
    enable_servos(servos);

    record_servo_midpoint(servos, calibrations, servo_deflections);
    disable_servos(servos);

    uint32_t notification_value = 0;
    while ((notification_value & ZERO_SERVOS_NOTIFICATION_BIT) == 0) {
        xTaskNotifyWait(0, ZERO_SERVOS_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
    }

    record_servo_zeropoint(servos, calibrations, servo_deflections);

    enable_servos(servos);
    record_servo_endpoints(servos, calibrations, servo_deflections);
    disable_servos(servos);

    /* Wait for notification to begin */
    notification_value = 0;
    while ((notification_value & BEGIN_STATIC_FIRE_NOTIFICATION_BIT) == 0) {
        xTaskNotifyWait(0, BEGIN_STATIC_FIRE_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
    }

    enable_servos(servos);

    HAL_GPIO_WritePin(PYRO_0_GPIO_Port, PYRO_0_Pin, GPIO_PIN_SET);

    vTaskDelay(pdMS_TO_TICKS(MOTOR_STARTUP_TIME));

    TickType_t start_ms = pdTICKS_TO_MS(xTaskGetTickCount());

    while (1) {
        TickType_t current_ms = pdTICKS_TO_MS(xTaskGetTickCount());

        float t = (float) (current_ms - start_ms) / STATIC_FIRE_ACTUATION_TIME_MS;
        float angle_rad = t * STATIC_FIRE_ACTUATION_RANGE_DEG;

        float desired_angles[4] = {PI / 2 + angle_rad, 0, PI / 2 - angle_rad, 0};
        uint16_t desired_adc_values[4];
        float desired_angles_absolute[4];

        for (int i = 0; i < 4; i++) {
            desired_angles_absolute[i] = get_absolute_angle(desired_angles[i], &calibrations[i]);
            desired_adc_values[i] = angle_to_adc(desired_angles[i], &calibrations[i]);

            setServoAngle(&servos[i], desired_angles_absolute[i]);
        }

        if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
            g_current_state.servo_deflections.servo_1_desired = desired_adc_values[0];
            g_current_state.servo_deflections.servo_2_desired = desired_adc_values[1];
            g_current_state.servo_deflections.servo_3_desired = desired_adc_values[2];
            g_current_state.servo_deflections.servo_4_desired = desired_adc_values[3];

            g_current_state.servo_deflections.timestamp = xTaskGetTickCount();
            
            xSemaphoreGive(g_state_mutex_handle);
        }

        if (current_ms - start_ms > STATIC_FIRE_ACTUATION_TIME_MS) {
            break;
        }
    
        /* Trigger ADC conversion sequence */
        HAL_ADC_Start_IT(FIRST_ADC);

        xTaskNotify(g_state_tx_task_handle, SEND_STATE_NOTIFICATION_BIT, eSetBits);
        xTaskNotify(g_state_flash_task_handle, FLASH_STATE_NOTIFICATION_BIT, eSetBits);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    xTaskNotify(g_state_flash_task_handle, FLASH_SD_CARD_NOTIFICATION_BIT, eSetBits);

    vTaskSuspend(NULL);
}