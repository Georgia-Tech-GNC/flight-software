#include "port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "example_task.h"
#include <string.h>
#include <stdio.h>
#include "isr.h"
#include "servo.h"

servo_t servo_pitch;

void adis16500_data_ready_callback(void) {
    HAL_GPIO_TogglePin(PYRO2_EN_GPIO_PORT, PYRO2_EN_PIN);
}

void example_task(void *_params) {
    UNUSED(_params);

    servo_init(&servo_pitch,
               &servo_tim,
               servo_pitch_tim_channel,
               SERVO_TIM_CLK_HZ/(servo_tim.Instance->PSC + 1),
               SERVO_MAX_ANGLE*2.0f,
               SERVO_MIN_PULSE_WIDTH_US,
               SERVO_MAX_PULSE_WIDTH_US,
               false);

    servo_enable(&servo_pitch);

    while (true) {
        servo_set_pos(&servo_pitch, -60);
        vTaskDelay(2000);
        servo_set_pos(&servo_pitch, 60);
        vTaskDelay(2000);
        // WITH_UART(debug,
        //     HAL_UART_Transmit(uart, (uint8_t *) "Hello, world!\r\n", (uint16_t) 15, HAL_MAX_DELAY);
        // );        
    }
        
}


