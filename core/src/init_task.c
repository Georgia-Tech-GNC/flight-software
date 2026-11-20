#include "port.h"
#include "init_task.h"
#include "tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "servo.h"

static void init(void);

servo_t servo_pitch;
servo_t servo_yaw;

void init_task(void *_params) {
    UNUSED(_params);

    WITH_UART(debug,
        HAL_UART_Transmit(uart, (uint8_t *) "Initializing...\r\n", 17, HAL_MAX_DELAY);
    );

    init();

    WITH_UART(debug,
        HAL_UART_Transmit(uart, (uint8_t *) "Done initializing.\r\n", 20, HAL_MAX_DELAY);
    );

    vTaskResume(example_task_handle);
    vTaskDelete(NULL);
}

void init(void) {
    //TODO: Driver initialization logic goes here
    servo_init(&servo_pitch,
               &servo_tim,
               servo_pitch_tim_channel,
               SERVO_TIM_CLK_HZ/(servo_tim.Instance->PSC + 1),
               SERVO_MAX_ANGLE*2.0f,
               SERVO_MIN_PULSE_WIDTH_US,
               SERVO_MAX_PULSE_WIDTH_US,
               false);

    servo_init(&servo_yaw,
        &servo_tim,
        servo_yaw_tim_channel,
        SERVO_TIM_CLK_HZ/(servo_tim.Instance->PSC + 1),
        SERVO_MAX_ANGLE*2.0f,
        SERVO_MIN_PULSE_WIDTH_US,
        SERVO_MAX_PULSE_WIDTH_US,
        false);
}
