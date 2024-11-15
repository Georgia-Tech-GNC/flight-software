#include "port_layer.h"

TaskHandle_t g_test_task_handle;
StackType_t test_task_stack[8192];
StaticTask_t test_task_buff;

TaskHandle_t g_sdio_task_handle;
StackType_t sdio_task_stack[4096];
StaticTask_t sdio_task_buff;

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

MessageBufferHandle_t g_sdio_mb_handle;
uint8_t sdio_mb_storage[256];
StaticMessageBuffer_t sdio_mb_buff;

uint8_t telemetry_uart_rx_buf[MAX_PACKET_SIZE_TELEMETRY];
uint8_t state_uart_rx_buf[MAX_PACKET_SIZE_STATE];

uint8_t adc1_conv_ptr = 0;
uint8_t adc2_conv_ptr = 0;
uint8_t adc3_conv_ptr = 0;

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

    g_sdio_mb_handle = xMessageBufferCreateStatic(SD_MB_SIZE + 1, sdio_mb_storage, &sdio_mb_buff);
    
    /* Create tasks */
    
    g_sdio_task_handle = xTaskCreateStatic(sdio_task, "flash_task", 4096, NULL, tskIDLE_PRIORITY, sdio_task_stack, &sdio_task_buff);
    if (g_sdio_task_handle == NULL) return 0;
    
    g_telemetry_tx_task_handle = xTaskCreateStatic(telemetry_tx_task, "telemetry_tx_task", 4096, NULL, tskIDLE_PRIORITY, telemetry_tx_task_stack, &telemetry_tx_task_buff);
    if (g_telemetry_tx_task_handle == NULL) return 0;
    
    g_telemetry_rx_task_handle = xTaskCreateStatic(telemetry_rx_task, "telemetry_rx_task", 4096, NULL, tskIDLE_PRIORITY, telemetry_rx_task_stack, &telemetry_rx_task_buff);
    if (g_telemetry_rx_task_handle == NULL) return 0;
    
    g_state_est_rx_task_handle = xTaskCreateStatic(state_est_rx_task, "state_rx_task", 4096, NULL, tskIDLE_PRIORITY, state_est_rx_task_stack, &state_est_rx_task_buff);
    if (g_state_est_rx_task_handle == NULL) return 0;

    g_state_tx_task_handle = xTaskCreateStatic(state_tx_task, "state_tx_task", 4096, NULL, tskIDLE_PRIORITY, state_tx_task_stack, &state_tx_task_buff);
    if (g_state_tx_task_handle == NULL) return 0;

    g_test_task_handle = xTaskCreateStatic(test_task, "test_task", 8192, NULL, tskIDLE_PRIORITY, test_task_stack, &test_task_buff);
    if (g_test_task_handle == NULL) return 0;

    /* Begin listening over uart */
    if (HAL_UARTEx_ReceiveToIdle_IT(&telemetry_uart, telemetry_uart_rx_buf, MAX_PACKET_SIZE_TELEMETRY) != HAL_OK) {
        return 0;
    }

    if (HAL_UARTEx_ReceiveToIdle_IT(&state_uart, state_uart_rx_buf, MAX_PACKET_SIZE_STATE) != HAL_OK) {
        return 0;
    }

    /* Begin ADC Conversions */
    if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
        return 0;
    }

    if (HAL_ADC_Start_IT(&hadc2) != HAL_OK) {
        return 0;
    }

    if (HAL_ADC_Start_IT(&hadc3) != HAL_OK) {
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

/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint16_t adc_val = 0;
    ADC_Channel channel;

    if (hadc->Instance == hadc1.Instance) {
        channel = ADC1_SEQUENCE[adc1_conv_ptr];
        adc_val = HAL_ADC_GetValue(&hadc1);
        adc1_conv_ptr = (adc1_conv_ptr + 1) % ADC1_N_CHANNELS;
    } else if (hadc->Instance == hadc2.Instance) {
        channel = ADC2_SEQUENCE[adc2_conv_ptr];
        adc_val = HAL_ADC_GetValue(&hadc2);
        adc2_conv_ptr = (adc2_conv_ptr + 1) % ADC2_N_CHANNELS;
    } else if (hadc->Instance == hadc3.Instance) {
        channel = ADC3_SEQUENCE[adc3_conv_ptr];
        adc_val = HAL_ADC_GetValue(&hadc3);
        adc3_conv_ptr = (adc3_conv_ptr + 1) % ADC3_N_CHANNELS;
    }

    switch (channel) {
        case ADC_I_SENSE_0:
            break;
        case ADC_I_SENSE_1:
            break;
        case ADC_I_SENSE_2:
            break;
        case ADC_I_SENSE_3:
            break;
        case ADC_I_SENSE_4:
            break;
        case ADC_SERVO_0:
            break;
        case ADC_SERVO_1:
            break;
        case ADC_SERVO_2:
            break;
        case ADC_SERVO_3:
            break;
        case ADC_SERVO_4:
            break;
        case ADC_PYRO_I_0:
            break;
        case ADC_PYRO_I_1:
            break;
        case ADC_PYRO_I_2:
            break;
        case ADC_VCC_I:
            break;
        case ADC_VCC_V:
            break;
        case ADC_BUCK_V:
            break;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
*/