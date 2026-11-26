#include "port.h"
#include "init_task.h"
#include "tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "servo.h"

static bool init(void);

servo_t servo_pitch;
servo_t servo_yaw;
rgb_led_t rgb_led;

void init_task(void *_params) {
    UNUSED(_params);

    WITH_UART(debug,
        HAL_UART_Transmit(uart, (uint8_t *) "Initializing...\r\n", 17, HAL_MAX_DELAY);
    );

    if (init()) {
        WITH_UART(debug,
            HAL_UART_Transmit(uart, (uint8_t *) "Done initializing.\r\n", 20, HAL_MAX_DELAY);
        );

        vTaskResume(example_task_handle);
    } else {
        WITH_UART(debug,
            HAL_UART_Transmit(uart, (uint8_t *) "Failed to initialize.\r\n", 23, HAL_MAX_DELAY);
        );
    }

    vTaskDelete(NULL);
}

bool init(void) {
    if (!servo_init(&servo_pitch,
        &servo_tim,
        servo_pitch_tim_channel,
        SERVO_TIM_CLK_HZ/(servo_tim.Instance->PSC + 1),
        SERVO_MAX_ANGLE*2.0,
        SERVO_MIN_PULSE_WIDTH_US,
        SERVO_MAX_PULSE_WIDTH_US,
        false
    )) {
        return false;
    }

    if (!servo_init(&servo_yaw,
        &servo_tim,
        servo_yaw_tim_channel,
        SERVO_TIM_CLK_HZ/(servo_tim.Instance->PSC + 1),
        SERVO_MAX_ANGLE*2.0,
        SERVO_MIN_PULSE_WIDTH_US,
        SERVO_MAX_PULSE_WIDTH_US,
        false
    )) {
        return false;
    }
    
    rgb_led.htim = &rgb_led_tim;
    rgb_led.ch_r = rgb_led_red_tim_channel;
    rgb_led.ch_g = rgb_led_green_tim_channel;
    rgb_led.ch_b = rgb_led_blue_tim_channel;
    rgb_led.inverted = true;

    if (!rgb_led_init(&rgb_led)) {
        return false;
    }

    return true;
}
