#include "state_flash.h"

int flash_test(void);
int sd_test(void);

void write_to_flash(IOChannel *flash_write_channel, RocketState *rocket_state);
void flash_sd_card(IOChannel *flash_read_channel, IOChannel *sd_write_channel, size_t n_states);
size_t to_csv_line(RocketState *rocket_state, char *line);

void state_flash_task(void *args) {
    if (flash_test()) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Flash test PASS\r\n", 17, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Flash test FAIL\r\n", 19, HAL_MAX_DELAY);
    }

    if (sd_test()) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "SD test PASS\r\n", 14, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "SD test FAIL\r\n", 16, HAL_MAX_DELAY);
    }

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
        if (xTaskNotifyWaitIndexed(1, 0, FLASH_SD_CARD_NOTIFICATION_BIT, &notification_value, 0) == pdTRUE) {
            if (notification_value & FLASH_SD_CARD_NOTIFICATION_BIT) {
                flash_sd_card(&flash_read_channel, &sd_write_channel, n_states);
                
                /* Stop everything */
                vTaskSuspendAll();
                
                while (1);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000 / FLASH_FREQ_HZ));
    }

    while (1) {
        vTaskDelay(10000);
    }
}

int flash_test(void) {
    StaticStreamBuffer_t flash_write_sb_buff;
    uint8_t flash_write_sb_storage_area[FLASH_MAX_READ_WRITE_SIZE + 1];
    StaticStreamBuffer_t flash_read_sb_buff;
    uint8_t flash_read_sb_storage_area[FLASH_MAX_READ_WRITE_SIZE + 1];

    IOChannel flash_write_channel;
    IOChannel flash_read_channel;

    flash_channel_init(&flash_write_channel, IO_MODE_WRITE, FLASH_WRITE_CHANNEL_ID, &flash_write_sb_buff, flash_write_sb_storage_area, FLASH_MAX_READ_WRITE_SIZE);
    flash_channel_init(&flash_read_channel, IO_MODE_READ, FLASH_READ_CHANNEL_ID, &flash_read_sb_buff, flash_read_sb_storage_area, FLASH_MAX_READ_WRITE_SIZE);

    const uint8_t offset = xTaskGetTickCount() % 256;

    uint8_t test_bytes[512];
    for (size_t i = 0; i < 512; i ++) {
        test_bytes[i] = (i + offset) % 256;
    }

    io_write_channel(&flash_write_channel, test_bytes, 256);
    io_save_channel(&flash_write_channel);

    if (xTaskNotifyWait(0, FLASH_WRITE_COMPLETE_NOTIFICATION_BIT, NULL, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    io_write_channel(&flash_write_channel, test_bytes + 256, 256);
    io_save_channel(&flash_write_channel);

    if (xTaskNotifyWait(0, FLASH_WRITE_COMPLETE_NOTIFICATION_BIT, NULL, 1000) != pdTRUE) {
        return 0;
    }

    for (size_t i = 0; i < 256; i ++) {
        test_bytes[i] = 0;
    }

    io_load_channel(&flash_read_channel, 0, 256);

    if (xTaskNotifyWait(0, FLASH_READ_COMPLETE_NOTIFICATION_BIT, NULL, 1000) != pdTRUE) {
        return 0;
    }

    io_read_channel(&flash_read_channel, test_bytes, 256);

    for (size_t i = 0; i < 256; i ++) {
        if (test_bytes[i] != (i + offset) % 256) {
            return 0;
        }
    }

    io_load_channel(&flash_read_channel, 256, 256);

    if (xTaskNotifyWait(0, FLASH_READ_COMPLETE_NOTIFICATION_BIT, NULL, 1000) != pdTRUE) {
        return 0;
    }
    
    for (size_t i = 0; i < 256; i ++) {
        test_bytes[i] = 0;
    }

    io_read_channel(&flash_read_channel, test_bytes, 256);

    for (size_t i = 0; i < 256; i ++) {
        if (test_bytes[i] != (i + offset) % 256) {
            return 0;
        }
    }

    io_reset_channel(&flash_write_channel);
    
    if (xTaskNotifyWait(0, FLASH_RESET_COMPLETE_NOTIFICATION_BIT, NULL, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    return 1;
}

int sd_test(void) {
    StaticStreamBuffer_t sd_write_sb_buff;
    uint8_t sd_write_sb_storage_area[SD_MAX_READ_WRITE_SIZE + 1];
    StaticStreamBuffer_t sd_read_sb_buff;
    uint8_t sd_read_sb_storage_area[SD_MAX_READ_WRITE_SIZE + 1];

    IOChannel sd_write_channel;
    IOChannel sd_read_channel;

    sd_channel_init(&sd_write_channel, "post.txt", IO_MODE_WRITE, SD_WRITE_CHANNEL_ID, &sd_write_sb_buff, sd_write_sb_storage_area, SD_MAX_READ_WRITE_SIZE);
    sd_channel_init(&sd_read_channel, "post.txt", IO_MODE_READ, SD_READ_CHANNEL_ID, &sd_read_sb_buff, sd_read_sb_storage_area, SD_MAX_READ_WRITE_SIZE);

    uint8_t test_bytes[1024];
    for (size_t i = 0; i < 1024; i ++) {
        test_bytes[i] = i % 256;
    }

    io_write_channel(&sd_write_channel, test_bytes, 512);
    io_save_channel(&sd_write_channel);

    if (xTaskNotifyWait(0, SD_WRITE_COMPLETE_NOTIFICATION_BIT, NULL, 1000) != pdTRUE) {
        return 0;
    }

    io_write_channel(&sd_write_channel, test_bytes + 512, 512);
    io_save_channel(&sd_write_channel);
    
    if (xTaskNotifyWait(0, SD_WRITE_COMPLETE_NOTIFICATION_BIT, NULL, 1000) != pdTRUE) {
        return 0;
    }

    for (size_t i = 0; i < 1024; i ++) {
        test_bytes[i] = 0;
    }

    io_load_channel(&sd_read_channel, 0, 512);

    if (xTaskNotifyWait(0, SD_READ_COMPLETE_NOTIFICATION_BIT, NULL, 1000) != pdTRUE) {
        return 0;
    }

    io_read_channel(&sd_read_channel, test_bytes, 512);

    for (size_t i = 0; i < 512; i ++) {
        if (test_bytes[i] != i % 256) {
            return 0;
        }
    }

    io_load_channel(&sd_read_channel, 512, 512);

    if (xTaskNotifyWait(0, SD_READ_COMPLETE_NOTIFICATION_BIT, NULL, 1000) != pdTRUE) {
        return 0;
    }

    io_read_channel(&sd_read_channel, test_bytes, 512);

    for (size_t i = 0; i < 512; i ++) {
        if (test_bytes[i] != i % 256) {
            return 0;
        }
    }

    io_reset_channel(&sd_write_channel);

    if (xTaskNotifyWait(0, SD_RESET_COMPLETE_NOTIFICATION_BIT, NULL, 1000) != pdTRUE) {
        return 0;
    }

    return 1;
}

void write_to_flash(IOChannel *flash_write_channel, RocketState *rocket_state) {
    uint8_t raw_bytes[256];
    memcpy(raw_bytes, rocket_state, sizeof(RocketState));

    io_write_channel(flash_write_channel, raw_bytes, 256);
    io_save_channel(flash_write_channel);

    xTaskNotifyWait(0, FLASH_WRITE_COMPLETE_NOTIFICATION_BIT, NULL, portMAX_DELAY);
}

void flash_sd_card(IOChannel *flash_read_channel, IOChannel *sd_write_channel, size_t n_states) {
    RocketState rocket_state;
    uint8_t data_buffer[FLASH_MAX_READ_WRITE_SIZE];
    size_t offset = 0;

    char line_buf[2048];
    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Writing to SD card...\r\n", 23, HAL_MAX_DELAY);

    for (size_t i = 0; i < n_states; i ++) {
        char progress[100];
        sprintf(progress, "Writing to SD card: %d/%d\r\n", i + 1, n_states);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) progress, strlen(progress), HAL_MAX_DELAY);

        io_load_channel(flash_read_channel, offset, 256);

        xTaskNotifyWait(0, FLASH_READ_COMPLETE_NOTIFICATION_BIT, NULL, portMAX_DELAY);

        size_t n_bytes = io_channel_get_full(flash_read_channel);
        offset += n_bytes;

        io_read_channel(flash_read_channel, data_buffer, 256);

        memcpy(&rocket_state, data_buffer, sizeof(RocketState));

        size_t line_len = to_csv_line(&rocket_state, line_buf);
        size_t written = 0;
        size_t remaining = line_len;

        while (remaining > 0) {

            size_t to_write = (remaining > SD_MAX_READ_WRITE_SIZE) ? SD_MAX_READ_WRITE_SIZE : remaining;

            io_write_channel(sd_write_channel, (uint8_t *) (line_buf + written), to_write);
            io_save_channel(sd_write_channel);

            remaining -= to_write;
            written += to_write;

            xTaskNotifyWait(0, SD_WRITE_COMPLETE_NOTIFICATION_BIT, NULL, portMAX_DELAY);
        }
    }

    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Wrote to SD card\r\n", 18, HAL_MAX_DELAY);
}

size_t to_csv_line(RocketState *rocket_state, char *line) {
    size_t len = 0;
    
    len += sprintf(line + len, "%lu,", (uint32_t) rocket_state->launch_timestamp);

    len += sprintf(line + len, "%lu,", (uint32_t) (rocket_state->state_vector.timestamp));
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
    len += sprintf(line + len, "%f,", rocket_state->state_vector.world_x);
    len += sprintf(line + len, "%f,", rocket_state->state_vector.world_y);
    len += sprintf(line + len, "%f,", rocket_state->state_vector.world_z);

    len += sprintf(line + len, "%lu,", (uint32_t) (rocket_state->servo_deflection.timestamp));
    len += sprintf(line + len, "%f,", rocket_state->servo_deflection.servo_deflection_1);
    len += sprintf(line + len, "%f,", rocket_state->servo_deflection.servo_deflection_2);
    len += sprintf(line + len, "%f,", rocket_state->servo_deflection.servo_deflection_3);
    len += sprintf(line + len, "%f,", rocket_state->servo_deflection.servo_deflection_4);

    len += sprintf(line + len, "%lu,", (uint32_t) (rocket_state->rocket_state.timestamp));
    len += sprintf(line + len, "%d,", rocket_state->rocket_state.rocket_state);
    len += sprintf(line + len, "%d,", rocket_state->rocket_state.firing_channel_1);
    len += sprintf(line + len, "%d,", rocket_state->rocket_state.firing_channel_2);
    len += sprintf(line + len, "%d,", rocket_state->rocket_state.firing_channel_3);

    len += sprintf(line + len, "%lu,", (uint32_t) (rocket_state->ground_ekf.timestamp));
    len += sprintf(line + len, "%f,", rocket_state->ground_ekf.pn_matrix_d1);
    len += sprintf(line + len, "%f,", rocket_state->ground_ekf.pn_matrix_d2);
    len += sprintf(line + len, "%f,", rocket_state->ground_ekf.pn_matrix_d3);
    len += sprintf(line + len, "%f,", rocket_state->ground_ekf.pn_matrix_d4);
    len += sprintf(line + len, "%f,", rocket_state->ground_ekf.pn_matrix_d5);
    len += sprintf(line + len, "%f,", rocket_state->ground_ekf.pn_matrix_d6);

    len += sprintf(line + len, "%lu,", (uint32_t) (rocket_state->sensor_data.timestamp));
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.accelerometer_x);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.accelerometer_y);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.accelerometer_z);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.gyro_x);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.gyro_y);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.gyro_z);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.gps_x);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.gps_y);
    len += sprintf(line + len, "%f,", rocket_state->sensor_data.gps_z);

    len += sprintf(line + len, "%lu,", (uint32_t) (rocket_state->analog_feedback_data.timestamp));
    len += sprintf(line + len, "%d,", rocket_state->analog_feedback_data.current_fb_33);
    len += sprintf(line + len, "%d,", rocket_state->analog_feedback_data.pyro_0_cont);
    len += sprintf(line + len, "%d,", rocket_state->analog_feedback_data.pyro_1_cont);
    len += sprintf(line + len, "%d,", rocket_state->analog_feedback_data.pyro_2_cont);
    len += sprintf(line + len, "%d", rocket_state->analog_feedback_data.pyro_channel_deploy);
    
    line[len++] = '\n';

    return len;
}