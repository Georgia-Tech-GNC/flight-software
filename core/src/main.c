/** @file main.c
 * @brief General entry point for all targets
 */

#include "main.h"
#include "lsm6dso.h"

#include <FreeRTOS.h>
#include <task.h>
#include <portmacro.h>
#include <stdio.h>

#include <string.h>

/* HELPFUL HINTS:
 * 
 * The following peripheral handles have already been configured for you:
 *  - Led: LED_GREEN_GPIO_Port and LED_GREEN_Pin
 *  - SPI1: hspi1   (you should pass this to HAL methods as a reference, ie. &hspi1)
 *  - SPI1 chip-select pin: SPI1_CS_GPIO_Port and SPI1_CS_Pin
 *  - UART: huart1  (you should pass this to HAL methods as a reference, ie. &huart1)
 * 
 *
 * The following HAL methods may be useful to you:
 *  - GPIO Pins:
 *      - HAL_GPIO_WritePin(<GPIO_PORT>, <GPIO_PIN>, GPIO_PIN_SET or GPIO_PIN_RESET)
 *  - UART:
 *      - HAL_UART_Transmit(<UART_HANDLE>, <DATA POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *      - HAL_UART_Receive(<UART_HANDLE>, <DATA POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *  - SPI:
 *      - HAL_SPI_Transmit(<SPI_HANDLE>, <DATA_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *      - HAL_SPI_Receive(<SPI_HANDLE>, <DATA_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *      - HAL_SPI_TransmitReceive(<SPI_HANDLE>, <SEND_DATA_POINTER>, <RECIEVE_DATA_BUFFER_POINTER>, <DATA_SIZE>, HAL_MAX_DELAY)
 *  - Misc.
 *      - HAL_DELAY(<DELAY IN MS>)
 */


 /** 
 * The main methods of all targets are expected to call this method once they are fully initialized
 */
[[noreturn]] void shared_main(void) {
    // Don't worry about this line for now :)
    HAL_TIM_Base_Start(&htim2);

    char* hello = "hello world!\r\n"; 

/*
    while (true) {
	// Enables the GPIO (general-purpose input-output pin) connected to the green led.
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

	// Send some number of bytes over UART
	// Only transmit needed since sending for Part b
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, (uint16_t)strlen(msg), HAL_MAX_DELAY);

	//Wait for one sec
	HAL_Delay(1000);
	//Disables the GPIO (general-purpose input-output pin) connected to the green led.
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	//Wait for one sec
	HAL_Delay(1000);	

    }
*/
     HAL_UART_Transmit(&huart3, (uint8_t*)hello, (uint16_t)strlen(hello), HAL_MAX_DELAY);
     bool ok = lsm6dso_initialize(&hspi1); 
     char* msg = ok ? "PASS\r\n" : "FAIL\r\n";
     HAL_UART_Transmit(&huart3, (uint8_t*)msg, (uint16_t)strlen(msg), HAL_MAX_DELAY);

     char value[128];

     while (true) {
	if (ok) {
		int32_t p = (int32_t) (lsm6dso_get_pitch_rate(&hspi1) * 1000.0f);
		int32_t r = (int32_t) (lsm6dso_get_roll_rate(&hspi1) * 1000.0f);
		int32_t y = (int32_t) (lsm6dso_get_yaw_rate(&hspi1) * 1000.0f);

		const char* ps = (p < 0) ? "-" : "";
		const char* rs = (r < 0) ? "-" : "";
		const char* ys = (y < 0) ? "-" : "";

		if (p < 0) {
			p = -p;
		}
		if (r < 0) {
			r = -r;
		}
		if (y < 0) {
			y = -y;
		}

		snprintf(value, sizeof value, "pitch=%s%ld.%03ld, roll=%s%ld.%03ld, yaw=%s%ld.%03ld dps\r\n", ps, 
		(long)(p/1000), (long)(p%1000), rs, (long)(r/1000), (long)(r%1000), ys, (long)(y/1000), (long)(y%1000));
	        HAL_UART_Transmit(&huart3, (uint8_t*)value, (uint16_t)strlen(value), HAL_MAX_DELAY);
	}
	HAL_Delay(200);
     }
}
