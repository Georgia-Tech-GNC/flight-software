#include "static_fire.h"

void init_servos(Servo_T *servos);
void enable_servos(Servo_T *servos);
void disable_servos(Servo_T *servos);
void set_servo_angles(Servo_T *servos, float angle_rad);
void record_servo_adcs(uint16_t *zeros, uint16_t **servo_deflections);
void measure_adc_with_delay(void);
void calibrate_servos(Servo_T *servos, uint16_t **servo_deflections);

void init_servos(Servo_T *servos) {
    servo_init(&servos[0], PWM0_TIMER, PWM0_CHANNEL, false);
    servo_init(&servos[1], PWM1_TIMER, PWM1_CHANNEL, false);
    servo_init(&servos[2], PWM2_TIMER, PWM2_CHANNEL, false);
    servo_init(&servos[3], PWM3_TIMER, PWM3_CHANNEL, false);
}

void enable_servos(Servo_T *servos) {
    for (int i = 0; i < 4; i++) {
        servo_enable(&servos[i]);
    }
}

void disable_servos(Servo_T *servos) {
    for (int i = 0; i < 4; i++) {
        servo_disable(&servos[i]);
    }
}

void set_servo_angles(Servo_T *servos, float angle_rad) {
    for (int i = 0; i < 4; i++) {
        char buf[100];
        sprintf(buf, "Timer counts: %d\r\n", servo_set_angle(&servos[i], angle_rad));
        HAL_UART_Transmit(&debug_uart, buf, strlen(buf), 10);
    }
}

void measure_adc_with_delay(void) {
    HAL_ADC_Start_IT(FIRST_ADC);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void record_servo_adcs(uint16_t *adcs, uint16_t **servo_deflections) {
    measure_adc_with_delay();

    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < 4; i++) {
            adcs[i] = (*servo_deflections[i]);
        }

        xSemaphoreGive(g_state_mutex_handle);
    }
}

void calibrate_servos(Servo_T *servos, uint16_t **servo_deflections) {
    uint16_t adc_zeros[4] = {0};
    uint16_t adc_starts[4] = {0};
    uint16_t adc_ends[4] = {0};
    
    record_servo_adcs(adc_zeros, servo_deflections);

    // Wait to remove jig
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Enable Servos and sweep to record end points
    enable_servos(servos);

    for (int i = 0; i < 4; ++i) {
        servo_go_to_calibration_start(&servos[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(750));
    record_servo_adcs(adc_starts, servo_deflections);

    for (int i = 0; i < 4; ++i) {
        servo_go_to_calibration_end(&servos[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(750));
    record_servo_adcs(adc_ends, servo_deflections);

    for (int i = 0; i < 4; ++i) {
        servo_set_zero(&servos[i], adc_starts[i], adc_ends[i], adc_zeros[i]);
    }

    char buf[100];
    sprintf(buf, "Zero: %d, Start: %d, End: %d\n", adc_zeros[3], adc_starts[3], adc_ends[3]);
    HAL_UART_Transmit(&debug_uart, buf, strlen(buf), 10);

    // Go back to zero position
    set_servo_angles(servos, 0);
}

void static_fire_task(void *args) {
    /* Initialize servos */
    Servo_T servo1 = {0};
    Servo_T servo2 = {0};
    Servo_T servo3 = {0};
    Servo_T servo4 = {0};

    Servo_T servos[] = {servo1, servo2, servo3, servo4};

    uint16_t *servo1_deflection = &g_current_state.servo_deflections.servo_1_actual;
    uint16_t *servo2_deflection = &g_current_state.servo_deflections.servo_2_actual;
    uint16_t *servo3_deflection = &g_current_state.servo_deflections.servo_3_actual;
    uint16_t *servo4_deflection = &g_current_state.servo_deflections.servo_4_actual;

    uint16_t *servo_deflections[] = {servo1_deflection, servo2_deflection, servo3_deflection, servo4_deflection};

    init_servos(servos);
    
    /*
    // Wait for zero servo notification
    /*uint32_t notification_value = 0;
    while ((notification_value & ZERO_SERVOS_NOTIFICATION_BIT) == 0) {
        xTaskNotifyWait(0, ZERO_SERVOS_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
    }*/

    //vTaskDelay(pdMS_TO_TICKS(5000));

    // Calibrate servos
    //calibrate_servos(servos, servo_deflections);
    
    /* Wait for notification to begin */
    /*notification_value = 0;
    while ((notification_value & BEGIN_STATIC_FIRE_NOTIFICATION_BIT) == 0) {
        xTaskNotifyWait(0, BEGIN_STATIC_FIRE_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
    }*/
   /*
    vTaskDelay(pdMS_TO_TICKS(2000));

    HAL_GPIO_WritePin(PYRO_0_GPIO_Port, PYRO_0_Pin, GPIO_PIN_SET);

    vTaskDelay(pdMS_TO_TICKS(MOTOR_STARTUP_TIME));

    TickType_t start_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    */
    while (1) {
        /*
        TickType_t current_ms = pdTICKS_TO_MS(xTaskGetTickCount());

        float t = (float) (current_ms - start_ms) / STATIC_FIRE_ACTUATION_TIME_MS;
        float angle_rad = t * STATIC_FIRE_ACTUATION_RANGE_DEG;

        float desired_angles[4] = {PI / 2 + angle_rad, 0, PI / 2 - angle_rad, 0};
        

        for (int i = 0; i < 4; i++) {
            char buf[100];
            sprintf(buf, "Setting servo %d angle %f\r\n", i, desired_angles[i]);
            HAL_UART_Transmit(&debug_uart, buf, strlen(buf), HAL_MAX_DELAY);

            servo_set_angle(&servos[i], desired_angles[i]);
        }

        /*
        if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
            g_current_state.servo_deflections.servo_1_desired = servo_rad_to_adc(&servos[0]);
            g_current_state.servo_deflections.servo_2_desired = servo_rad_to_adc(&servos[1]);
            g_current_state.servo_deflections.servo_3_desired = servo_rad_to_adc(&servos[2]);
            g_current_state.servo_deflections.servo_4_desired = servo_rad_to_adc(&servos[3]);

            g_current_state.servo_deflections.timestamp = xTaskGetTickCount();
    
            xSemaphoreGive(g_state_mutex_handle);
        }
        
        if (current_ms - start_ms > STATIC_FIRE_ACTUATION_TIME_MS) {
            break;
        }
        */
        /* Trigger ADC conversion sequence */
        
        measure_adc_with_delay();

        //xTaskNotify(g_state_tx_task_handle, SEND_STATE_NOTIFICATION_BIT, eSetBits);
        //xTaskNotify(g_state_flash_task_handle, FLASH_STATE_NOTIFICATION_BIT, eSetBits);

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    xTaskNotify(g_state_flash_task_handle, FLASH_SD_CARD_NOTIFICATION_BIT, eSetBits);

    vTaskSuspend(NULL);
}