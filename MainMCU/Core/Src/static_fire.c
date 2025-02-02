#include "static_fire.h"

void static_fire_task(void *args) {
    /* Initialize servos */
    Servo_T servo1;
    Servo_T servo2;
    Servo_T servo3;
    Servo_T servo4;

    initServo(&servo1, PWM0_TIMER, PWM0_CHANNEL);
    initServo(&servo2, PWM1_TIMER, PWM1_CHANNEL);
    initServo(&servo3, PWM2_TIMER, PWM2_CHANNEL);
    initServo(&servo4, PWM3_TIMER, PWM3_CHANNEL);

    enableServo(&servo1);
    enableServo(&servo2);
    enableServo(&servo3);
    enableServo(&servo4);

    /* Wait for notification to begin */
    uint32_t notification_value = 0;
    while ((notification_value & BEGIN_STATIC_FIRE_NOTIFICATION_BIT) == 0) {
        xTaskNotifyWait(0, BEGIN_STATIC_FIRE_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
    }

    TickType_t start_ms = pdTICKS_TO_MS(xTaskGetTickCount());

    while (1) {
        TickType_t current_ms = pdTICKS_TO_MS(xTaskGetTickCount());

        float t = (float) (current_ms - start_ms) / STATIC_FIRE_ACTUATION_TIME_MS;
        float angle_rad = t * STATIC_FIRE_ACTUATION_RANGE_DEG * PI / 180.0;

        setServoAngle(&servo1, angle_rad);
        setServoAngle(&servo2, angle_rad);
        setServoAngle(&servo3, angle_rad);
        setServoAngle(&servo4, angle_rad);

        if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
            uint16_t adc_angle = SERVO_RAD_TO_ADC(angle_rad);

            g_current_state.servo_deflections.servo_1_desired = adc_angle;
            g_current_state.servo_deflections.servo_2_desired = adc_angle;
            g_current_state.servo_deflections.servo_3_desired = adc_angle;
            g_current_state.servo_deflections.servo_4_desired = adc_angle;
            
            xSemaphoreGive(g_state_mutex_handle);
        }

        if (current_ms - start_ms > STATIC_FIRE_ACTUATION_TIME_MS) {
            break;
        }
    
        /* Trigger ADC conversion sequence */
        HAL_ADC_Start_IT(FIRST_ADC);

        /* Maximum frequency of 20hz */    
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    xTaskNotify(g_state_flash_task_handle, FLASH_STATE_NOTIFICATION_BIT, eSetBits);

    vTaskSuspend(NULL);
}