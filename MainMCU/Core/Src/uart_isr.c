#include "uart_isr.h"

uint8_t telemetry_uart_rx_buf[TELEMETRY_RX_MAX_PROCESS_SIZE];
uint8_t state_uart_rx_buf[STATE_PACKET_SIZE];

uint8_t state_bytes[STATE_PACKET_SIZE];

uint8_t state_bytes_received = 0;

void process_state_bytes(uint16_t size, BaseType_t *xHigherPriorityTaskWoken);
void update_rocket_state(BaseType_t *xHigherPriorityTaskWoken);

int begin_uart_listen(void) {
    if (HAL_UARTEx_ReceiveToIdle_IT(&telemetry_uart, telemetry_uart_rx_buf, TELEMETRY_RX_MAX_PROCESS_SIZE) != HAL_OK) {
        return 0;
    }
    
    
    if (HAL_UARTEx_ReceiveToIdle_IT(&state_uart, state_uart_rx_buf, STATE_PACKET_SIZE) != HAL_OK) {
        return 0;
    }

    return 1;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == telemetry_uart.Instance) {
        HAL_GPIO_WritePin(LD2_GPIO_PORT, LD2_PIN, GPIO_PIN_SET);
        xStreamBufferSendFromISR(g_telemetry_rx_sb_handle, telemetry_uart_rx_buf, size, &xHigherPriorityTaskWoken);
        HAL_UARTEx_ReceiveToIdle_IT(&telemetry_uart, telemetry_uart_rx_buf, TELEMETRY_RX_MAX_PROCESS_SIZE);
    } else if (huart->Instance == state_uart.Instance) {
        HAL_GPIO_WritePin(LD1_GPIO_PORT, LD1_PIN, GPIO_PIN_SET);
        process_state_bytes(size, &xHigherPriorityTaskWoken);
        HAL_UARTEx_ReceiveToIdle_IT(&state_uart, state_uart_rx_buf, STATE_PACKET_SIZE);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void process_state_bytes(uint16_t size, BaseType_t *xHigherPriorityTaskWoken) {
    uint16_t size_to_recieve = size;

    if (state_bytes_received + size > STATE_PACKET_SIZE) {
        size_to_recieve = STATE_PACKET_SIZE - state_bytes_received;
    }

    memcpy(state_bytes + state_bytes_received, state_uart_rx_buf, size_to_recieve);

    state_bytes_received += size_to_recieve;

    if (state_bytes_received == STATE_PACKET_SIZE) {
        HAL_GPIO_TogglePin(LD2_GPIO_PORT, LD2_PIN);
        update_rocket_state(xHigherPriorityTaskWoken);
        state_bytes_received = 0;
    }

    uint16_t remaining_size = size - size_to_recieve;

    memcpy(state_bytes + state_bytes_received, state_uart_rx_buf + size_to_recieve, remaining_size);
    state_bytes_received += remaining_size;
}

void update_rocket_state(BaseType_t *xHigherPriorityTaskWoken) {
    if (xSemaphoreTakeFromISR(g_state_mutex_handle, xHigherPriorityTaskWoken) == pdTRUE) {
        uint8_t *serial_buffer = state_bytes + 37;
        uint8_t *sensors_buffer = state_bytes;

        uint16_t offset = 1;
        memcpy(&g_current_state.sensor_data.accelerometer_x, sensors_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.sensor_data.accelerometer_y, sensors_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.sensor_data.accelerometer_z, sensors_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.sensor_data.gyro_x, sensors_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.sensor_data.gyro_y, sensors_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.sensor_data.gyro_z, sensors_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.sensor_data.gps_x, sensors_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.sensor_data.gps_y, sensors_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.sensor_data.gps_z, sensors_buffer + offset, 4);
        
        offset = 0;

        memcpy(&g_current_state.rocket_state.rocket_state, serial_buffer + offset, sizeof(uint8_t));
        offset += sizeof(uint8_t);
        memcpy(&g_current_state.state_vector.position_x, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.state_vector.position_y, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.state_vector.position_z, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.state_vector.velocity_x, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.state_vector.velocity_y, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.state_vector.velocity_z, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.state_vector.attitude_w, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.state_vector.attitude_x, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.state_vector.attitude_y, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.state_vector.attitude_z, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.state_vector.world_x, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.state_vector.world_y, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.state_vector.world_z, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.ground_ekf.pn_matrix_d1, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.ground_ekf.pn_matrix_d2, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.ground_ekf.pn_matrix_d3, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.ground_ekf.pn_matrix_d4, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.ground_ekf.pn_matrix_d5, serial_buffer + offset, 4);
        offset += 4;
        memcpy(&g_current_state.ground_ekf.pn_matrix_d6, serial_buffer + offset, 4);
        offset += 4;

        TickType_t ticks = xTaskGetTickCount();

        g_current_state.analog_feedback_data.timestamp = ticks;
        g_current_state.ground_ekf.timestamp = ticks;
        g_current_state.state_vector.timestamp = ticks;
        g_current_state.sensor_data.timestamp = ticks;
        g_current_state.rocket_state.timestamp = ticks;
        g_current_state.servo_deflection.timestamp = ticks;

        xSemaphoreGiveFromISR(g_state_mutex_handle, xHigherPriorityTaskWoken);

        xTaskNotifyFromISR(g_state_tx_task_handle, SEND_STATE_NOTIFICATION_BIT, eSetBits, xHigherPriorityTaskWoken);
        xTaskNotifyFromISR(g_state_flash_task_handle, FLASH_STATE_NOTIFICATION_BIT, eSetBits, xHigherPriorityTaskWoken);
    }
}

void check_state_markers(BaseType_t *xHigherPriorityTaskWoken) {
    static uint8_t launched = 0;
    static uint8_t landed = 0;

    static uint8_t drogue_parachute_deploy = 0;
    static uint8_t main_parachute_deploy = 0;

    /* Launched */
    if (launched == 0 && g_current_state.rocket_state.rocket_state >= 2) {
        launched = 1;
        g_current_state.launch_timestamp = xTaskGetTickCount();
        xTaskNotifyFromISR(g_run_controls_task_handle, BEGIN_CONTROLS_NOTIFICATION_BIT, eSetBits, xHigherPriorityTaskWoken);
    }

    if (landed == 0 && g_current_state.rocket_state.rocket_state == 6) {
        landed = 1;        
        xTaskNotifyFromISR(g_state_flash_task_handle, FLASH_SD_CARD_NOTIFICATION_BIT, eSetBits, xHigherPriorityTaskWoken);
    }

    if (drogue_parachute_deploy == 0 && g_current_state.rocket_state.rocket_state == 5) {
        drogue_parachute_deploy = 1;
        g_current_state.rocket_state.firing_channel_1 = 1;
    }

    if (main_parachute_deploy == 0 && (g_current_state.rocket_state.rocket_state == 5 && (g_current_state.state_vector.position_z < 250 || g_current_state.state_vector.position_z > -250))) {
        main_parachute_deploy = 1;
        g_current_state.rocket_state.firing_channel_2 = 1;
    }
}