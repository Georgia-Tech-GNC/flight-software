/** @file shared_main.c
 * @brief General entry point for all targets
 */

#include "port.h"
#include "shared_main.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define STACK_SIZE 4096
#include "lps25hb.h"

static void blink_led_1(void *_params);
static void blink_led_2(void *_params);
static void blink_led_3(void *_params);
void servo_set_angle(TIM_HandleTypeDef *htim, uint16_t channel, float angle);
void tim_set_us(TIM_HandleTypeDef *htim, uint16_t channel, float us, uint32_t base_freq_hz);
void tim_set_duty(TIM_HandleTypeDef *htim, uint16_t channel, float duty);

StaticTask_t blink_led_1_buffer;
StackType_t blink_led_1_stack[STACK_SIZE];
StaticTask_t blink_led_2_buffer;
StackType_t blink_led_2_stack[STACK_SIZE];
StaticTask_t blink_led_3_buffer;
StackType_t blink_led_3_stack[STACK_SIZE];

static void blink_led_1(void *_params) {
    UNUSED(_params);

    while (true) {
        vTaskDelay(100);
        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);
        vTaskDelay(100);
        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);
    }
}

static void blink_led_2(void *_params) {
    UNUSED(_params);

    while (true) {
        vTaskDelay(200);
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
        vTaskDelay(200);
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);
    }
}

void servo_set_angle(TIM_HandleTypeDef *htim, uint16_t channel, float angle) {
    float lerp = (angle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
    float us = lerp * (SERVO_MAX_PULSE_WIDTH_US - SERVO_MIN_PULSE_WIDTH_US) + SERVO_MIN_PULSE_WIDTH_US;
    tim_set_us(htim, channel, us, SERVO_TIM_CLK_HZ);
}

void tim_set_duty(TIM_HandleTypeDef *htim, uint16_t channel, float duty) {
    __HAL_TIM_SET_COMPARE(htim, channel, (uint16_t) ((float) htim->Instance->ARR * duty));
}

void tim_set_us(TIM_HandleTypeDef *htim, uint16_t channel, float us, uint32_t base_freq_hz) {
    float count_hz = (float) base_freq_hz / (float) ((float) htim->Instance->PSC + 1);
    float counts_per_us = count_hz / 1000000.0f;

    uint16_t counts = (uint16_t)(us * counts_per_us);
    __HAL_TIM_SET_COMPARE(htim, channel, counts);
}

static void blink_led_3(void *_params) {
    UNUSED(_params);

    HAL_TIM_PWM_Start(&servo_tim, servo_pitch_tim_channel);
    HAL_TIM_PWM_Start(&servo_tim, servo_yaw_tim_channel);
    HAL_TIM_PWM_Start(&rgb_led_tim, rgb_led_red_tim_channel);
    HAL_TIM_PWM_Start(&rgb_led_tim, rgb_led_green_tim_channel);
    HAL_TIM_PWM_Start(&rgb_led_tim, rgb_led_blue_tim_channel);
    
    while (true) {
        vTaskDelay(400);
                HAL_GPIO_TogglePin(PYRO2_EN_GPIO_PORT, PYRO2_EN_PIN);

        HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_RESET);
        uint8_t transmit_data[] = {0xAB, 0xBD, 0xEF};
        uint8_t receive_data[3];
        HAL_GPIO_WritePin(adis16500_spi_cs_port, adis16500_spi_cs_pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&adis16500_spi, transmit_data, receive_data, 3, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(adis16500_spi_cs_port, adis16500_spi_cs_pin, GPIO_PIN_SET);
        char buf[100];
        snprintf(buf, 100, "Recieved SPI bytes (adis16500) 0x%02X, 0x%02X, 0x%02X\r\n", receive_data[0], receive_data[1], receive_data[2]);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) buf, (uint16_t) strlen(buf), HAL_MAX_DELAY);

        HAL_GPIO_WritePin(lsm6dsv_spi_cs_port, lsm6dsv_spi_cs_pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&lsm6dsv_spi, transmit_data, receive_data, 3, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(lsm6dsv_spi_cs_port, lsm6dsv_spi_cs_pin, GPIO_PIN_SET);
        char buf3[100];
        snprintf(buf3, 100, "Recieved SPI bytes (lsm6dsv) 0x%02X, 0x%02X, 0x%02X\r\n", receive_data[0], receive_data[1], receive_data[2]);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) buf3, (uint16_t) strlen(buf3), HAL_MAX_DELAY);

        uint8_t i2c_transmit_data[] = {0x12, 0x34, 0x56};
        HAL_I2C_Master_Transmit(&lps25hbtr_i2c, lps25hbtr_i2c_addr << 1, i2c_transmit_data, 3, HAL_MAX_DELAY);
        uint8_t i2c_receive_data[3];
        HAL_I2C_Master_Receive(&lps25hbtr_i2c, lps25hbtr_i2c_addr << 1, i2c_receive_data, 3, HAL_MAX_DELAY);
        char buf2[100];
        snprintf(buf2, 100, "Recieved I2C bytes (lps25hbtr) 0x%02X, 0x%02X, 0x%02X\r\n", i2c_receive_data[0], i2c_receive_data[1], i2c_receive_data[2]);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) buf2, (uint16_t) strlen(buf2), HAL_MAX_DELAY);

        servo_set_angle(&servo_tim, servo_pitch_tim_channel, 30);
        servo_set_angle(&servo_tim, servo_yaw_tim_channel, -20);

        tim_set_duty(&rgb_led_tim, rgb_led_red_tim_channel, 0.5f);
        tim_set_duty(&rgb_led_tim, rgb_led_green_tim_channel, 0.3f);
        tim_set_duty(&rgb_led_tim, rgb_led_blue_tim_channel, 0.7f);

        int pyro1_cont = HAL_GPIO_ReadPin(PYRO1_CONT_GPIO_PORT, PYRO1_CONT_PIN);
        int pyro2_cont = HAL_GPIO_ReadPin(PYRO2_CONT_GPIO_PORT, PYRO2_CONT_PIN);
        int pyro_global_en = HAL_GPIO_ReadPin(PYRO_GLOBAL_EN_GPIO_PORT, PYRO_GLOBAL_EN_PIN);
        char buf4[200];
        snprintf(buf4, 200, "Pyro 1 cont %d, pyro 2 cont: %d, pyro global en: %d\r\n", pyro1_cont, pyro2_cont, pyro_global_en);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) buf4, (uint16_t) strlen(buf4), HAL_MAX_DELAY);

        vTaskDelay(400);
        HAL_GPIO_TogglePin(PYRO1_EN_GPIO_PORT, PYRO1_EN_PIN);

        HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_SET);
    }
        
}

// Utility function
static void uart_send_message(const char* str) {
    const uint16_t length = (uint16_t)strlen(str);
    const uint16_t size = sizeof(char) * length;
    
    HAL_UART_Transmit(&huart3, (const uint8_t*)str, size, HAL_MAX_DELAY);
}

/** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    // TaskHandle_t blink_led_1_task = xTaskCreateStatic(blink_led_1, "Blink LED 1", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_1_stack, &blink_led_1_buffer);
    // TaskHandle_t blink_led_2_task = xTaskCreateStatic(blink_led_2, "Blink LED 2", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_2_stack, &blink_led_2_buffer);
    // TaskHandle_t blink_led_3_task = xTaskCreateStatic(blink_led_3, "Blink LED 3", STACK_SIZE, NULL, tskIDLE_PRIORITY, blink_led_3_stack, &blink_led_3_buffer);

    // if (blink_led_1_task && blink_led_2_task && blink_led_3_task) {
    //     HAL_UART_Transmit(&debug_uart, (uint8_t *) "Successfully created tasks, starting scheduler.\r\n", 50, HAL_MAX_DELAY);
    //     vTaskStartScheduler();
    // } else {
    //     HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to create tasks\r\n", 25, HAL_MAX_DELAY);
    // }
    
    // Test stuff
    HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);
    uart_send_message("Before Init.\n");
    
    lps25hb_data lps25hb_data;
    lps25hb_initialize(&lps25hb_data, &hi2c1);
    
    uart_send_message("After Init.\n");
    HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);

    while (true) __asm__ volatile (""); // No return
} 
