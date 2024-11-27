#include "port_layer.h"

#ifdef USE_TESTS
TaskHandle_t g_test_task_handle;
StackType_t test_task_stack[2048];
StaticTask_t test_task_buff;
#endif

TaskHandle_t g_periph_io_task_handle;
StackType_t periph_io_task_stack[4096];
StaticTask_t periph_io_task_buff;

TaskHandle_t g_telemetry_tx_task_handle;
StackType_t telemetry_tx_task_stack[4096];
StaticTask_t telemetry_tx_task_buff;

TaskHandle_t g_telemetry_rx_task_handle;
StackType_t telemetry_rx_task_stack[4096];
StaticTask_t telemetry_rx_task_buff;

TaskHandle_t g_state_est_rx_task_handle;
StackType_t state_est_rx_task_stack[4096];
StaticTask_t state_est_rx_task_buff;

TaskHandle_t g_state_tx_task_handle;
StackType_t state_tx_task_stack[4096];
StaticTask_t state_tx_task_buff;

TaskHandle_t g_state_flash_task_handle;
StackType_t state_flash_task_stack[4096];
StaticTask_t state_flash_task_buff;

TaskHandle_t g_adc_convert_task_handle;
StackType_t adc_convert_task_stack[256];
StaticTask_t adc_convert_task_buff;

TaskHandle_t g_run_controls_task_handle;
StackType_t run_controls_task_stack[4096];
StaticTask_t run_controls_task_buff;

SemaphoreHandle_t g_state_mutex_handle;
StaticSemaphore_t state_mutex_buff;

StreamBufferHandle_t g_telemetry_rx_sb_handle;
uint8_t telemetry_rx_sb_storage[128 + 1];
StaticStreamBuffer_t telemetry_rx_sb_buff;

MessageBufferHandle_t g_telemetry_tx_mb_handle;
uint8_t telemetry_tx_mb_storage[TX_MESSAGE_BUFFER_SIZE];
StaticMessageBuffer_t telemetry_tx_mb_buff;

StreamBufferHandle_t g_state_rx_sb_handle;
uint8_t state_rx_sb_storage[STATE_ESTIMATION_BYTES * 2 + 1];
StaticMessageBuffer_t state_rx_sb_buff;

MessageBufferHandle_t g_periph_io_mb_handle;
uint8_t periph_io_mb_storage[IO_MB_SIZE + 2];
StaticMessageBuffer_t periph_io_mb_buff;

uint8_t telemetry_uart_rx_buf[MAX_PACKET_SIZE_TELEMETRY];
uint8_t state_uart_rx_buf[MAX_PACKET_SIZE_STATE];

uint16_t adc1_conv_ptr = 0;
uint16_t adc2_conv_ptr = 0;
uint16_t adc3_conv_ptr = 0;

uint16_t pyro_1_cont_avg_buf[10] = {0};
uint16_t pyro_2_cont_avg_buf[10] = {0};
uint16_t pyro_3_cont_avg_buf[10] = {0};
uint16_t current_fb_33_avg_buf[10] = {0};

uint16_t pyro_1_cont_avg_ptr = 0;
uint16_t pyro_2_cont_avg_ptr = 0;
uint16_t pyro_3_cont_avg_ptr = 0;
uint16_t current_fb_33_avg_ptr = 0;

RocketState g_current_state = {0};

int port_init(void) {

    /* Create mutexes */
    g_state_mutex_handle = xSemaphoreCreateMutexStatic(&state_mutex_buff);

    if (g_state_mutex_handle == NULL) {
        return 0;
    }

    /* Create stream/message buffers */
    g_telemetry_rx_sb_handle = xStreamBufferCreateStatic(128 + 1, 1, telemetry_rx_sb_storage, &telemetry_rx_sb_buff);
    if (g_telemetry_rx_sb_handle == NULL) return 0;

    g_telemetry_tx_mb_handle = xMessageBufferCreateStatic(TX_MESSAGE_BUFFER_SIZE, telemetry_tx_mb_storage, &telemetry_tx_mb_buff);
    if (g_telemetry_tx_mb_handle == NULL) return 0;

    g_state_rx_sb_handle = xStreamBufferCreateStatic(STATE_ESTIMATION_BYTES * 2 + 1, 1, state_rx_sb_storage, &state_rx_sb_buff);
    if (g_state_rx_sb_handle == NULL) return 0;

    g_periph_io_mb_handle = xMessageBufferCreateStatic(IO_MB_SIZE + 1, periph_io_mb_storage, &periph_io_mb_buff);
    if (g_periph_io_mb_handle == NULL) return 0;

    /* Create tasks */
    g_periph_io_task_handle = xTaskCreateStatic(periph_io_task, "flash_task", 4096, NULL, tskIDLE_PRIORITY, periph_io_task_stack, &periph_io_task_buff);
    if (g_periph_io_task_handle == NULL) return 0;
    
    g_telemetry_tx_task_handle = xTaskCreateStatic(telemetry_tx_task, "telemetry_tx_task", 4096, NULL, tskIDLE_PRIORITY, telemetry_tx_task_stack, &telemetry_tx_task_buff);
    if (g_telemetry_tx_task_handle == NULL) return 0;
    
    g_telemetry_rx_task_handle = xTaskCreateStatic(telemetry_rx_task, "telemetry_rx_task", 4096, NULL, tskIDLE_PRIORITY, telemetry_rx_task_stack, &telemetry_rx_task_buff);
    if (g_telemetry_rx_task_handle == NULL) return 0;
    
    g_state_est_rx_task_handle = xTaskCreateStatic(state_est_rx_task, "state_rx_task", 4096, NULL, tskIDLE_PRIORITY, state_est_rx_task_stack, &state_est_rx_task_buff);
    if (g_state_est_rx_task_handle == NULL) return 0;

    g_state_tx_task_handle = xTaskCreateStatic(state_tx_task, "state_tx_task", 4096, NULL, tskIDLE_PRIORITY, state_tx_task_stack, &state_tx_task_buff);
    if (g_state_tx_task_handle == NULL) return 0;

    g_state_flash_task_handle = xTaskCreateStatic(state_flash_task, "state_flash_task", 4096, NULL, tskIDLE_PRIORITY, state_flash_task_stack, &state_flash_task_buff);
    if (g_state_flash_task_handle == NULL) return 0;

    g_adc_convert_task_handle = xTaskCreateStatic(adc_convert_task, "adc_convert_task", 256, NULL, tskIDLE_PRIORITY, adc_convert_task_stack, &adc_convert_task_buff);
    if (g_adc_convert_task_handle == NULL) return 0;

    g_run_controls_task_handle = xTaskCreateStatic(run_controls_task, "run_controls_task", 4096, NULL, tskIDLE_PRIORITY, run_controls_task_stack, &run_controls_task_buff);
    if (g_run_controls_task_handle == NULL) return 0;
    
#ifdef USE_TESTS
    g_test_task_handle = xTaskCreateStatic(test_task, "test_task", 2048, NULL, tskIDLE_PRIORITY, test_task_stack, &test_task_buff);
    if (g_test_task_handle == NULL) return 0;
#endif

    /* Begin listening over uart */
    if (HAL_UARTEx_ReceiveToIdle_IT(&telemetry_uart, telemetry_uart_rx_buf, MAX_PACKET_SIZE_TELEMETRY) != HAL_OK) {
        return 0;
    }

    if (HAL_UARTEx_ReceiveToIdle_IT(&state_uart, state_uart_rx_buf, MAX_PACKET_SIZE_STATE) != HAL_OK) {
        return 0;
    }

    return 1;
}

void port_start(void) {
    vTaskStartScheduler();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Stack overflow\r\n", 16, HAL_MAX_DELAY);
    while(1);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == telemetry_uart.Instance) {
        xStreamBufferSendFromISR(g_telemetry_rx_sb_handle, telemetry_uart_rx_buf, size, &xHigherPriorityTaskWoken);
        HAL_UARTEx_ReceiveToIdle_IT(&telemetry_uart, telemetry_uart_rx_buf, MAX_PACKET_SIZE_TELEMETRY);
    } else if (huart->Instance == state_uart.Instance) {
        xStreamBufferSendFromISR(g_state_rx_sb_handle, state_uart_rx_buf, size, &xHigherPriorityTaskWoken);
        HAL_UARTEx_ReceiveToIdle_IT(&state_uart, state_uart_rx_buf, MAX_PACKET_SIZE_STATE);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t adc_val = HAL_ADC_GetValue(hadc);
    ADC_Channel channel;

    ADC_HandleTypeDef *to_start = NULL;
    
#ifdef USE_ADC1
    if (hadc->Instance == hadc1.Instance) {
        channel = ADC1_SEQUENCE[adc1_conv_ptr];
        adc1_conv_ptr = (adc1_conv_ptr + 1) % ADC1_N_CHANNELS;

        if (adc1_conv_ptr == 0) {
            to_start = ADC1_NEXT;
        }
    }
#endif

#ifdef USE_ADC2
    if (hadc->Instance == hadc2.Instance) {
        channel = ADC2_SEQUENCE[adc2_conv_ptr];
        adc2_conv_ptr = (adc2_conv_ptr + 1) % ADC2_N_CHANNELS;

        if (adc2_conv_ptr == 0) {
            to_start = ADC2_NEXT;
        }
    }
#endif

#ifdef USE_ADC3
    if (hadc->Instance == hadc3.Instance) {
        channel = ADC3_SEQUENCE[adc3_conv_ptr];
        adc3_conv_ptr = (adc3_conv_ptr + 1) % ADC3_N_CHANNELS;

        if (adc3_conv_ptr == 0) {
            to_start = ADC3_NEXT;
        }
    }
#endif



    if (xSemaphoreTakeFromISR(g_state_mutex_handle, &xHigherPriorityTaskWoken) == pdTRUE) {
        uint32_t rolling_avg = 0;

        switch (channel) {
            case ADC_PYRO_I_0:
                pyro_1_cont_avg_buf[pyro_1_cont_avg_ptr] = adc_val;
                pyro_1_cont_avg_ptr = (pyro_1_cont_avg_ptr + 1) % 10;

                for (int i = 0; i < 10; i++) {
                    rolling_avg += pyro_1_cont_avg_buf[i];
                }

                g_current_state.analog_feedback_data.pyro_0_cont = rolling_avg / 10;
                break;
            case ADC_PYRO_I_1:
                pyro_2_cont_avg_buf[pyro_2_cont_avg_ptr] = adc_val;
                pyro_2_cont_avg_ptr = (pyro_2_cont_avg_ptr + 1) % 10;

                for (int i = 0; i < 10; i++) {
                    rolling_avg += pyro_2_cont_avg_buf[i];
                }

                g_current_state.analog_feedback_data.pyro_1_cont = rolling_avg / 10;
                break;
            case ADC_PYRO_I_2:
                pyro_3_cont_avg_buf[pyro_3_cont_avg_ptr] = adc_val;
                pyro_3_cont_avg_ptr = (pyro_3_cont_avg_ptr + 1) % 10;

                for (int i = 0; i < 10; i++) {
                    rolling_avg += pyro_3_cont_avg_buf[i];
                }

                g_current_state.analog_feedback_data.pyro_2_cont = rolling_avg / 10;
                break;
            case ADC_VCC_I:
                current_fb_33_avg_buf[current_fb_33_avg_ptr] = adc_val;
                current_fb_33_avg_ptr = (current_fb_33_avg_ptr + 1) % 10;
                
                for (int i = 0; i < 10; i++) {
                    rolling_avg += current_fb_33_avg_buf[i];
                }

                g_current_state.analog_feedback_data.current_fb_33 = rolling_avg / 10;
                break;
        }

        g_current_state.analog_feedback_data.timestamp = xTaskGetTickCount();
        xSemaphoreGiveFromISR(g_state_mutex_handle, &xHigherPriorityTaskWoken);
    }

    if (to_start != NULL) {
        HAL_ADC_Start_IT(to_start);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
