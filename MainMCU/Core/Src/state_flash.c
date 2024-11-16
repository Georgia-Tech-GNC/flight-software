#include "state_flash.h"

void write_to_flash(IOChannel *flash_write_channel, RocketState *rocket_state);
void flash_sd_card(IOChannel *flash_read_channel, IOChannel *sd_write_channel, size_t n_states);
size_t to_csv_line(RocketState *rocket_state, char *line);

void state_flash_task(void *args) {
    RocketState rocket_state;

    IOChannel flash_write_channel;
    IOChannel flash_read_channel;
    IOChannel sd_write_channel;

    StaticStreamBuffer_t flash_write_sb_buff;
    uint8_t flash_write_sb_storage_area[FLASH_MAX_READ_WRITE_SIZE + 1];

    StaticStreamBuffer_t flash_read_sb_buff;
    uint8_t flash_read_sb_storage_area[FLASH_MAX_READ_WRITE_SIZE + 1];

    StaticStreamBuffer_t sd_write_sb_buff;
    uint8_t sd_write_sb_storage_area[SD_MAX_READ_WRITE_SIZE + 1];

    size_t n_states = 0;

    flash_channel_init(&flash_write_channel, IO_MODE_WRITE, FLASH_WRITE_CHANNEL_ID, &flash_write_sb_buff, flash_write_sb_storage_area, FLASH_MAX_READ_WRITE_SIZE);
    flash_channel_init(&flash_read_channel, IO_MODE_READ, FLASH_READ_CHANNEL_ID, &flash_read_sb_buff, flash_read_sb_storage_area, FLASH_MAX_READ_WRITE_SIZE);
    sd_channel_init(&sd_write_channel, "data.csv", IO_MODE_WRITE, SD_WRITE_CHANNEL_ID, &sd_write_sb_buff, sd_write_sb_storage_area, SD_MAX_READ_WRITE_SIZE);

    uint32_t notification_value = 0;
    while ((notification_value & BEGIN_STATE_FLASH_NOTIFICATION_BIT) == 0) {
        xTaskNotifyWait(0, BEGIN_STATE_FLASH_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
    }

    while (1) {
        if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
            memcpy(&rocket_state, &g_current_state, sizeof(RocketState));
            xSemaphoreGive(g_state_mutex_handle);
        }

        write_to_flash(&flash_write_channel, &rocket_state);
        n_states ++;

        notification_value = 0;
        if (xTaskNotifyWaitIndexed(1, 0, FLASH_SD_CARD_NOTIFICATION_BIT, &notification_value, 10) == pdTRUE) {
            if (notification_value & FLASH_SD_CARD_NOTIFICATION_BIT) {
                HAL_UART_Transmit(&debug_uart, (uint8_t *) "Flashing to SD card...\r\n", 24, HAL_MAX_DELAY);
                flash_sd_card(&flash_read_channel, &sd_write_channel, n_states);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000 / FLASH_FREQ_HZ));
    }
}

void write_to_flash(IOChannel *flash_write_channel, RocketState *rocket_state) {
    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Writing to flash...\r\n", 21, HAL_MAX_DELAY);

    uint8_t raw_bytes[sizeof(RocketState)];
    memcpy(raw_bytes, rocket_state, sizeof(RocketState));

    io_write_channel(flash_write_channel, raw_bytes, sizeof(RocketState));
    io_save_channel(flash_write_channel);

    xTaskNotifyWait(0, FLASH_WRITE_COMPLETE_NOTIFICATION_BIT, NULL, portMAX_DELAY);
}

void flash_sd_card(IOChannel *flash_read_channel, IOChannel *sd_write_channel, size_t n_states) {
    RocketState *rocket_state;
    uint8_t data_buffer[FLASH_MAX_READ_WRITE_SIZE];
    size_t offset = 0;

    char line_buf[2048];

    for (size_t i = 0; i < n_states; i ++) {
        io_load_channel(flash_read_channel, offset, sizeof(RocketState));
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Loading from flash...\r\n", 23, HAL_MAX_DELAY);
        xTaskNotifyWait(0, FLASH_READ_COMPLETE_NOTIFICATION_BIT, NULL, portMAX_DELAY);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Loaded from flash\r\n", 19, HAL_MAX_DELAY);

        size_t n_bytes = io_channel_get_full(flash_read_channel);
        offset += n_bytes;

        io_read_channel(flash_read_channel, data_buffer, n_bytes);

        rocket_state = (RocketState *) data_buffer;

        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Converting to CSV line...\r\n", 26, HAL_MAX_DELAY);
        size_t line_len = to_csv_line(rocket_state, line_buf);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Converted to CSV line\r\n", 24, HAL_MAX_DELAY);
        size_t written = 0;
        size_t remaining = line_len;

        while (remaining > 0) {
            HAL_UART_Transmit(&debug_uart, (uint8_t *) "Writing to SD card...\r\n", 23, HAL_MAX_DELAY);

            size_t to_write = (remaining > SD_MAX_READ_WRITE_SIZE) ? SD_MAX_READ_WRITE_SIZE : remaining;

            io_write_channel(sd_write_channel, (uint8_t *) (line_buf + written), to_write);
            io_save_channel(sd_write_channel);

            remaining -= to_write;
            written += to_write;

            xTaskNotifyWait(0, SD_WRITE_COMPLETE_NOTIFICATION_BIT, NULL, portMAX_DELAY);
        }

        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Wrote to SD card\r\n", 18, HAL_MAX_DELAY);
    }
}

size_t to_csv_line(RocketState *rocket_state, char *line) {
    size_t len = 0;
    
    len += sprintf(line + len, "%ld,", rocket_state->state_vector.timestamp);
    len += sprintf(line + len, "%f,", rocket_state->state_vector.velocity_x);
    len += sprintf(line + len, "%f,", rocket_state->state_vector.velocity_y);
    len += sprintf(line + len, "%f,", rocket_state->state_vector.velocity_z);
    len += sprintf(line + len, "%f,", rocket_state->state_vector.attitude_w);
    len += sprintf(line + len, "%f,", rocket_state->state_vector.attitude_x);
    len += sprintf(line + len, "%f,", rocket_state->state_vector.attitude_y);
    len += sprintf(line + len, "%f,", rocket_state->state_vector.attitude_z);
    len += sprintf(line + len, "%f,", rocket_state->state_vector.position_x);
    len += sprintf(line + len, "%f,", rocket_state->state_vector.position_y);
    len += sprintf(line + len, "%f,", rocket_state->state_vector.position_z);

    len += sprintf(line + len, "%ld,", rocket_state->servo_deflection.timestamp);
    len += sprintf(line + len, "%f,", rocket_state->servo_deflection.servo_deflection_1);
    len += sprintf(line + len, "%f,", rocket_state->servo_deflection.servo_deflection_2);
    len += sprintf(line + len, "%f,", rocket_state->servo_deflection.servo_deflection_3);
    len += sprintf(line + len, "%f,", rocket_state->servo_deflection.servo_deflection_4);

    len += sprintf(line + len, "%ld,", rocket_state->rocket_state.timestamp);
    len += sprintf(line + len, "%d,", rocket_state->rocket_state.rocket_state);
    len += sprintf(line + len, "%d,", rocket_state->rocket_state.firing_channel_1);
    len += sprintf(line + len, "%d,", rocket_state->rocket_state.firing_channel_2);
    len += sprintf(line + len, "%d,", rocket_state->rocket_state.firing_channel_3);

    len += sprintf(line + len, "%ld,", rocket_state->ground_ekf.timestamp);
    len += sprintf(line + len, "%f,", rocket_state->ground_ekf.pn_matrix_d1);
    len += sprintf(line + len, "%f,", rocket_state->ground_ekf.pn_matrix_d2);
    len += sprintf(line + len, "%f,", rocket_state->ground_ekf.pn_matrix_d3);
    len += sprintf(line + len, "%f,", rocket_state->ground_ekf.pn_matrix_d4);
    len += sprintf(line + len, "%f,", rocket_state->ground_ekf.pn_matrix_d5);
    len += sprintf(line + len, "%f,", rocket_state->ground_ekf.pn_matrix_d6);

    len += sprintf(line + len, "%ld,", rocket_state->sensor_data.timestamp);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.accelerometer_x);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.accelerometer_y);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.accelerometer_z);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.gyro_x);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.gyro_y);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.gyro_z);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.gps_x);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.gps_y);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.gps_z);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.barometer);

    len += sprintf(line + len, "%ld,", rocket_state->analog_feedback_data.timestamp);
    len += sprintf(line + len, "%f,", rocket_state->analog_feedback_data.voltage_fb_33);
    len += sprintf(line + len, "%f,", rocket_state->analog_feedback_data.current_fb_33);
    len += sprintf(line + len, "%f,", rocket_state->analog_feedback_data.pyro_0_cont);
    len += sprintf(line + len, "%f,", rocket_state->analog_feedback_data.pyro_1_cont);
    len += sprintf(line + len, "%f,", rocket_state->analog_feedback_data.pyro_2_cont);
    len += sprintf(line + len, "%d", rocket_state->analog_feedback_data.pyro_channel_deploy);
    
    line[len++] = '\n';
    line[len] = '\r';
    HAL_UART_Transmit(&debug_uart, (uint8_t *) line, len + 1, HAL_MAX_DELAY);

    return len;
}