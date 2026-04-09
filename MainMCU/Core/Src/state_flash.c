#include "state_flash.h"

int flash_test(void);
int sd_test(void);

void write_to_flash(FlashBlock *flash_block, RocketState *rocket_state, size_t page_index);
void flash_sd_card(FlashBlock *flash_block, SDFile *sd_file, size_t n_states);
size_t to_csv_line(RocketState *rocket_state, char *line);
size_t printf_fixed_float(char *buf, float f);

/**
 * @brief Task to handle writing state to flash chip and SD card
 * @param args Unused
 */
void state_flash_task(void *args) {
    /* Initialize flash chip and SD card */
    if (!io_init()) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to initialize IO\r\n", 26, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "IO initialized\r\n", 17, HAL_MAX_DELAY);
    }

    /* Run tests */
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

    /* Wait for notification before beginning */
    uint32_t notification_value = 0;
    while ((notification_value & BEGIN_STATE_FLASH_NOTIFICATION_BIT) == 0) {
        xTaskNotifyWait(0, BEGIN_STATE_FLASH_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);
    }

    FlashBlock flash_block;
    SDFile sd_file;

    flash_init_block(&flash_block, STATE_FLASH_START_SECTOR, STATE_FLASH_N_SECTORS);
    sd_init_file(&sd_file, "/data.csv");

    size_t flash_page_index = 0;

    while (1) {
        /* Wait for next notification */
        xTaskNotifyWait(0, FLASH_STATE_NOTIFICATION_BIT | FLASH_SD_CARD_NOTIFICATION_BIT, &notification_value, portMAX_DELAY);

        /* Flash state to SD card */
        if (notification_value & FLASH_SD_CARD_NOTIFICATION_BIT) {
            HAL_UART_Transmit(&debug_uart, (uint8_t *) "Flashing SD card...\r\n", 20, HAL_MAX_DELAY);
            flash_sd_card(&flash_block, &sd_file, flash_page_index);

            /* Stop everything */
            vTaskSuspendAll();

            /* Tasks aren't allowed to exit, so stall here */
            vTaskDelay(portMAX_DELAY);
        }

        /* Write state to flash chip */
        if (notification_value & FLASH_STATE_NOTIFICATION_BIT) {
            uint8_t state_bytes[EXT_FLASH_PAGE_SIZE];

            /* Always use mutex on g_current_state */
            if (xSemaphoreTake(g_state_lock.handle, portMAX_DELAY) == pdTRUE) {
                /* Memcpy out so we can give back the mutex as fast as possible */
                memcpy(state_bytes, &g_current_state, sizeof(RocketState));
                xSemaphoreGive(g_state_lock.handle);
            }

            /* Flash state */
            flash_write_block(&flash_block, flash_page_index * EXT_FLASH_PAGE_SIZE, state_bytes, EXT_FLASH_PAGE_SIZE);
            flash_page_index ++;
        }
    }
}

/**
 * @brief Do a simple test of the flash chip.
 * Writes a test pattern to the flash chip, reads it back, and erases the block.
 * @return 1 if the test passes, 0 otherwise
 */
int flash_test(void) {
    FlashBlock test_block;

    /* Allocate a new 2-sector block starting at sector 1 */
    if (!flash_init_block(&test_block, 1, 2)) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to initialize flash block\r\n", 32, HAL_MAX_DELAY);
        return 0;
    }

    uint8_t test_bytes[FLASH_TEST_SIZE];
    const uint8_t offset = xTaskGetTickCount() % 256;

    /* Create test pattern */
    for (size_t i = 0; i < FLASH_TEST_SIZE; i ++) {
        test_bytes[i] = (i + offset) % 256;
    }

    /* Write pattern to flash chip */
    if (!flash_write_block(&test_block, 0, test_bytes, FLASH_TEST_SIZE)) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to write to flash\r\n", 26, HAL_MAX_DELAY);
        return 0;
    }

    memset(test_bytes, 0, FLASH_TEST_SIZE);

    /* Read it back */
    if (!flash_read_block(&test_block, 0, test_bytes, FLASH_TEST_SIZE)) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to read from flash\r\n", 27, HAL_MAX_DELAY);
        return 0;
    }

    /* Check if the pattern matches */
    for (size_t i = 0; i < FLASH_TEST_SIZE; i ++) {
        if (test_bytes[i] != (i + offset) % 256) {
            return 0;
        }
    }

    /* Erase what we just wroe */
    if (!flash_erase_block(&test_block)) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to erase flash\r\n", 29, HAL_MAX_DELAY);
        return 0;
    }

    return 1;
}

/**
 * @brief Do a simple test of the SD card.
 * Writes a test pattern to the SD card, reads it back, and deletes the file.
 * @return 1 if the test passes, 0 otherwise
 */
int sd_test(void) {
    SDFile test_file;

    if (!sd_init_file(&test_file, "/test.txt")) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to initialize SD card file\r\n", 35, HAL_MAX_DELAY);
        return 0;
    }

    if (!sd_open_file(&test_file, FA_READ | FA_WRITE | FA_CREATE_ALWAYS)) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to open SD card file\r\n", 28, HAL_MAX_DELAY);
        return 0;
    }

    uint8_t test_bytes[SD_TEST_SIZE + 1];
    const uint8_t offset = xTaskGetTickCount() % 256;

    /* Create test pattern */
    for (size_t i = 0; i < SD_TEST_SIZE; i ++) {
        test_bytes[i] = (i + offset) % 256;
    }

    /* Write pattern to SD card */
    if (!sd_write_file(&test_file, 0, test_bytes, SD_TEST_SIZE)) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to write to SD card\r\n", 28, HAL_MAX_DELAY);
        return 0;
    }

    memset(test_bytes, 0, SD_TEST_SIZE);

    /* Read it back */
    if (!sd_read_file(&test_file, 0, test_bytes, SD_TEST_SIZE)) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to read from SD card\r\n", 29, HAL_MAX_DELAY);
        return 0;
    }

    /* Check if the pattern matches */
    for (size_t i = 0; i < SD_TEST_SIZE; i ++) {
        if (test_bytes[i] != (i + offset) % 256) {
            HAL_UART_Transmit(&debug_uart, (uint8_t *) "SD card read error\r\n", 20, HAL_MAX_DELAY);
            return 0;
        }
    }

    if (!sd_close_file(&test_file)) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to close SD card file\r\n", 30, HAL_MAX_DELAY);
        return 0;
    }

    /* Erase what we just wrote */
    if (!sd_delete_file(&test_file)) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Failed to delete SD card file\r\n", 31, HAL_MAX_DELAY);
        return 0;
    }

    return 1;
}

/**
 * @brief Copy the state from the flash chip to the SD card
 * @param flash_block Flash block to read from
 * @param sd_file SD card file to write to
 * @param n_states Number of states to copy
 */
void flash_sd_card(FlashBlock *flash_block, SDFile *sd_file, size_t n_states) {
    if (sd_open_file(sd_file, FA_READ | FA_WRITE | FA_CREATE_ALWAYS)) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Successfully opened SD card file\r\n", 34, HAL_MAX_DELAY);
    }

    RocketState rocket_state;
    uint8_t data_buffer[EXT_FLASH_PAGE_SIZE];

    char line_buf[2048];
    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Writing to SD card...\r\n", 23, HAL_MAX_DELAY);

    size_t sd_bytes_written = 0;

    for (size_t i = 0; i < n_states; i ++) {
        char progress[100];
        sprintf(progress, "Writing to SD card: %d/%d\r\n", i + 1, n_states);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) progress, strlen(progress), HAL_MAX_DELAY);

        flash_read_block(flash_block, i * EXT_FLASH_PAGE_SIZE, data_buffer, EXT_FLASH_PAGE_SIZE);

        memcpy(&rocket_state, data_buffer, sizeof(RocketState));

        size_t line_len = to_csv_line(&rocket_state, line_buf);

        if (sd_write_file(sd_file, sd_bytes_written, (uint8_t *) line_buf, line_len)) {
            HAL_UART_Transmit(&debug_uart, (uint8_t *) "writing to SD card\r\n", 20, HAL_MAX_DELAY);
        }

        sd_bytes_written += line_len;
    }

    if (sd_close_file(sd_file)) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Closed SD card file\r\n", 21, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Wrote to SD card\r\n", 18, HAL_MAX_DELAY);
}

/**
 * @brief Write a RocketState to a CSV line
 * @param rocket_state RocketState to write
 * @param line Buffer to write to
 * @return Number of bytes written
 */
size_t to_csv_line(RocketState *rocket_state, char *line) {
    size_t len = 0;
    
    len += sprintf(line + len, "%lu,", (uint32_t) rocket_state->timestamp);

#ifdef DO_NOT_RUN
    len += sprintf(line + len, "%lu,", (uint32_t) (rocket_state->state_vector.timestamp));
    len += printf_fixed_float(line + len, rocket_state->state_vector.velocity_x);
    len += printf_fixed_float(line + len, rocket_state->state_vector.velocity_y);
    len += printf_fixed_float(line + len, rocket_state->state_vector.velocity_z);
    len += printf_fixed_float(line + len, rocket_state->state_vector.attitude_w);
    len += printf_fixed_float(line + len, rocket_state->state_vector.attitude_x);
    len += printf_fixed_float(line + len, rocket_state->state_vector.attitude_y);
    len += printf_fixed_float(line + len, rocket_state->state_vector.attitude_z);
    len += printf_fixed_float(line + len, rocket_state->state_vector.position_x);
    len += printf_fixed_float(line + len, rocket_state->state_vector.position_y);
    len += printf_fixed_float(line + len, rocket_state->state_vector.position_z);
    len += printf_fixed_float(line + len, rocket_state->state_vector.world_x);
    len += printf_fixed_float(line + len, rocket_state->state_vector.world_y);
    len += printf_fixed_float(line + len, rocket_state->state_vector.world_z);
    len += sprintf(line + len, "%lu,", (uint32_t) (rocket_state->servo_deflection.timestamp));
    len += printf_fixed_float(line + len, rocket_state->servo_deflection.servo_deflection_1);
    len += printf_fixed_float(line + len, rocket_state->servo_deflection.servo_deflection_2);
    len += printf_fixed_float(line + len, rocket_state->servo_deflection.servo_deflection_3);
    len += printf_fixed_float(line + len, rocket_state->servo_deflection.servo_deflection_4);

    len += sprintf(line + len, "%lu,", (uint32_t) rocket_state->servo_deflections.timestamp);
    len += sprintf(line + len, "%d", rocket_state->servo_deflections.servo_1_desired);
    len += sprintf(line + len, "%d", rocket_state->servo_deflections.servo_1_actual);
    len += sprintf(line + len, "%d", rocket_state->servo_deflections.servo_2_desired);
    len += sprintf(line + len, "%d", rocket_state->servo_deflections.servo_2_actual);
    len += sprintf(line + len, "%d", rocket_state->servo_deflections.servo_3_desired);
    len += sprintf(line + len, "%d", rocket_state->servo_deflections.servo_3_actual);
    len += sprintf(line + len, "%d", rocket_state->servo_deflections.servo_4_desired);
    len += sprintf(line + len, "%d", rocket_state->servo_deflections.servo_4_actual);

    len += sprintf(line + len, "%lu,", (uint32_t) (rocket_state->rocket_state.timestamp));
    len += sprintf(line + len, "%d,", rocket_state->rocket_state.rocket_state);
    
#ifndef STATIC_FIRE
    len += sprintf(line + len, "%d,", rocket_state->rocket_state.firing_channel_1);
    len += sprintf(line + len, "%d,", rocket_state->rocket_state.firing_channel_2);
    len += sprintf(line + len, "%d,", rocket_state->rocket_state.firing_channel_3);

    len += sprintf(line + len, "%lu,", (uint32_t) (rocket_state->ground_ekf.timestamp));
    len += printf_fixed_float(line + len, rocket_state->ground_ekf.pn_matrix_d1);
    len += printf_fixed_float(line + len, rocket_state->ground_ekf.pn_matrix_d2);
    len += printf_fixed_float(line + len, rocket_state->ground_ekf.pn_matrix_d3);
    len += printf_fixed_float(line + len, rocket_state->ground_ekf.pn_matrix_d4);
    len += printf_fixed_float(line + len, rocket_state->ground_ekf.pn_matrix_d5);
    len += printf_fixed_float(line + len, rocket_state->ground_ekf.pn_matrix_d6);

    len += sprintf(line + len, "%lu,", (uint32_t) (rocket_state->sensor_data.timestamp));
    len += printf_fixed_float(line + len, rocket_state->sensor_data.accelerometer_x);
    len += printf_fixed_float(line + len, rocket_state->sensor_data.accelerometer_y);
    len += printf_fixed_float(line + len, rocket_state->sensor_data.accelerometer_z);
    len += printf_fixed_float(line + len, rocket_state->sensor_data.gyro_x);
    len += printf_fixed_float(line + len, rocket_state->sensor_data.gyro_y);
    len += printf_fixed_float(line + len, rocket_state->sensor_data.gyro_z);
    len += printf_fixed_float(line + len, rocket_state->sensor_data.gps_x);
    len += printf_fixed_float(line + len, rocket_state->sensor_data.gps_y);
    len += printf_fixed_float(line + len, rocket_state->sensor_data.gps_z);

    len += sprintf(line + len, "%lu,", (uint32_t) (rocket_state->analog_feedback_data.timestamp));
    len += sprintf(line + len, "%d,", rocket_state->analog_feedback_data.current_fb_33);
    len += sprintf(line + len, "%d,", rocket_state->analog_feedback_data.pyro_0_cont);
    len += sprintf(line + len, "%d,", rocket_state->analog_feedback_data.pyro_1_cont);
    len += sprintf(line + len, "%d,", rocket_state->analog_feedback_data.pyro_2_cont);
    len += sprintf(line + len, "%d", rocket_state->analog_feedback_data.pyro_channel_deploy);
#endif
#endif


    line[len++] = '\n';

    return len;
}

/**
 * @brief Print a float to a buffer with fixed precision (3 decimal places)
 * This helps us avoid adding floating point support to printf with -u _printf_float
 * @param buf Buffer to write to
 * @param f Float to write
 * @return Number of bytes written
 */
size_t printf_fixed_float(char *buf, float f) {
    int i = (int) f;
    int d = (int) ((f - i) * 1000);
    return sprintf(buf, "%d.%03d,", i, d);
}