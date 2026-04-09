#include "state_flash.h"

int flash_test(void);
int sd_test(void);

void write_to_flash(FlashBlock *flash_block, RocketState *rocket_state, size_t page_index);
void flash_sd_card(SDFile *sd_file, size_t n_states);
size_t to_csv_line(RocketState *rocket_state, char *line);
size_t printf_fixed_float(char *buf, float f);

RocketState local_buffer[800];

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
    if (sd_test()) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "SD test PASS\r\n", 14, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "SD test FAIL\r\n", 16, HAL_MAX_DELAY);
    }
    
    /* Wait for notification before beginning */
    SDFile sd_file;

    sd_init_file(&sd_file, "/data.csv");

    size_t flash_index = 0;

    while (1) {

        RocketState state;
        bool should_flash = false;
        bool should_write_sd = false;
        // Update global state
        if (xSemaphoreTake(g_state_lock.handle, pdMS_TO_TICKS(100)) == pdTRUE) {
            state = g_current_state;
            if (g_current_state.state == SD_FLASH) {
                should_write_sd = true;
            } else if (g_current_state.state != GROUND && g_current_state.state != ARMED && g_current_state.state != FREEFALL) {
                should_flash = true;
            } 
            xSemaphoreGive(g_state_lock.handle);
        }

        if (should_write_sd) {
            HAL_UART_Transmit(&debug_uart, (uint8_t *) "Flashing SD card...\r\n", 20, HAL_MAX_DELAY);
            flash_sd_card(&sd_file, flash_index);
            HAL_UART_Transmit(&debug_uart, (uint8_t *) "Finished. Suspending...\r\n", 25, HAL_MAX_DELAY);

            // Stop everything 
            vTaskSuspendAll();
            HAL_Delay(HAL_MAX_DELAY);
            // we cannot use vTaskDelay(portMAX_DELAY) while the scheduler is suspended

        } else if (should_flash) {
            if (flash_index < 800) {
                local_buffer[flash_index++] = state;
            }
            //HAL_UART_Transmit(&debug_uart, "Flashed state\r\n", 15, HAL_MAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        

        /* Flash state to SD card 
        if (notification_value & FLASH_SD_CARD_NOTIFICATION_BIT) {
            HAL_UART_Transmit(&debug_uart, (uint8_t *) "Flashing SD card...\r\n", 20, HAL_MAX_DELAY);
            flash_sd_card(&flash_block, &sd_file, flash_page_index);

            /* Stop everything 
            vTaskSuspendAll();

            /* Tasks aren't allowed to exit, so stall here 
            vTaskDelay(portMAX_DELAY);
        }
        */

        /* Write state to flash chip 
        if (notification_value & FLASH_STATE_NOTIFICATION_BIT) {
            uint8_t state_bytes[EXT_FLASH_PAGE_SIZE];

            /* Always use mutex on g_current_state 
            if (xSemaphoreTake(g_state_lock.handle, portMAX_DELAY) == pdTRUE) {
                /* Memcpy out so we can give back the mutex as fast as possible 
                memcpy(state_bytes, &g_current_state, sizeof(RocketState));
                xSemaphoreGive(g_state_lock.handle);
            }

            /* Flash state 
            flash_write_block(&flash_block, flash_page_index * EXT_FLASH_PAGE_SIZE, state_bytes, EXT_FLASH_PAGE_SIZE);
            flash_page_index ++;
        }
        */
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
            HAL_UART_Transmit(&debug_uart, (uint8_t *) "Invalid read from flash\r\n", 27, HAL_MAX_DELAY);
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
void flash_sd_card(SDFile *sd_file, size_t n_states) {
    if (sd_open_file(sd_file, FA_READ | FA_WRITE | FA_CREATE_ALWAYS)) {
        HAL_UART_Transmit(&debug_uart, (uint8_t *) "Successfully opened SD card file\r\n", 34, HAL_MAX_DELAY);
    }

    char line_buf[2048];
    HAL_UART_Transmit(&debug_uart, (uint8_t *) "Writing to SD card...\r\n", 23, HAL_MAX_DELAY);

    size_t sd_bytes_written = 0;

    for (size_t i = 0; i < n_states; i ++) {
        char progress[100];
        sprintf(progress, "Writing to SD card: %d/%d\r\n", i + 1, n_states);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) progress, strlen(progress), HAL_MAX_DELAY);

        size_t line_len = to_csv_line(&local_buffer[i], line_buf);

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
    
    len += sprintf(line + len, "%lu,%lu,", (uint32_t)rocket_state->timestamp, (uint32_t)rocket_state->state);

    len += printf_fixed_float(line + len, rocket_state->orientation.w);
    len += printf_fixed_float(line + len, rocket_state->orientation.x);
    len += printf_fixed_float(line + len, rocket_state->orientation.y);
    len += printf_fixed_float(line + len, rocket_state->orientation.z);

    len += printf_fixed_float(line + len, rocket_state->servo_cmd_1);
    len += printf_fixed_float(line + len, rocket_state->servo_cmd_2);

    len += printf_fixed_float(line + len, rocket_state->w_x);
    len += printf_fixed_float(line + len, rocket_state->w_y);
    len += printf_fixed_float(line + len, rocket_state->w_z);
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