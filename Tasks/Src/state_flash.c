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
    if (io_init()) {
        log_printf(LOG_INFO, "IO initialized");
    } else {
        log_printf(LOG_ERROR, "Failed to initialize IO");
    }

    /* Run tests */
    if (flash_test()) {
        log_printf(LOG_INFO, "Flash test PASS");
    } else {
        log_printf(LOG_ERROR, "Flash test PASS");
    }

    if (sd_test()) {
        log_printf(LOG_INFO, "SD test PASS");
    } else {
        log_printf(LOG_ERROR, "SD test FAIL")
    }

    /* Wait for notification before beginning */
    await_notification(BEGIN_STATE_FLASH_NOTIFICATION_BIT, portMAX_DELAY);

    FlashBlock flash_block;
    SDFile sd_file;

    flash_init_block(&flash_block, STATE_FLASH_START_SECTOR, STATE_FLASH_N_SECTORS);
    sd_init_file(&sd_file, "/data.csv");

    size_t flash_page_index = 0;

    while (1) {
        /* Wait for next notification */
        uint32_t notification_value = await_notification(FLASH_STATE_NOTIFICATION_BIT | FLASH_SD_CARD_NOTIFICATION_BIT, portMAX_DELAY);

        /* Flash state to SD card */
        if (notification_value & FLASH_SD_CARD_NOTIFICATION_BIT) {
            log_printf(LOG_INFO, "Flashing SD card...");
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
            memcpy_state_bytes(state_bytes);

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
        log_printf(LOG_ERROR, "Failed to initialize flash block");
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
        log_printf(LOG_ERROR, "Failed to write to flash");
        return 0;
    }

    memset(test_bytes, 0, FLASH_TEST_SIZE);

    /* Read it back */
    if (!flash_read_block(&test_block, 0, test_bytes, FLASH_TEST_SIZE)) {
        log_printf(LOG_ERROR, "Failed to read from flash");
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
        log_printf(LOG_ERROR, "Failed to erase flash");
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
        log_printf(LOG_ERROR, "Failed to initialize SD card file");
        return 0;
    }

    if (!sd_open_file(&test_file, FA_READ | FA_WRITE | FA_CREATE_ALWAYS)) {
        log_printf(LOG_ERROR, "Failed to open SD card file");
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
        log_printf(LOG_ERROR, "Failed to write to SD card");
        return 0;
    }

    memset(test_bytes, 0, SD_TEST_SIZE);

    /* Read it back */
    if (!sd_read_file(&test_file, 0, test_bytes, SD_TEST_SIZE)) {
        log_printf(LOG_ERROR, "Failed to read from SD card");
        return 0;
    }

    /* Check if the pattern matches */
    for (size_t i = 0; i < SD_TEST_SIZE; i ++) {
        if (test_bytes[i] != (i + offset) % 256) {
            log_printf(LOG_ERROR, "SD card read error");
            return 0;
        }
    }

    if (!sd_close_file(&test_file)) {
        log_printf(LOG_ERROR, "Failed to close SD card file");
        return 0;
    }

    /* Erase what we just wrote */
    if (!sd_delete_file(&test_file)) {
        log_printf(LOG_ERROR, "Failed to delete SD card file");
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
        log_printf(LOG_INFO, "Successfully opened SD card file");
    }

    RocketState rocket_state;
    uint8_t data_buffer[EXT_FLASH_PAGE_SIZE];

    char line_buf[2048];
    log_printf(LOG_INFO, "Writing to SD card...");

    size_t sd_bytes_written = 0;

    for (size_t i = 0; i < n_states; i ++) {
        log_printf(LOG_INFO, "Writing to SD card: %d/%d", i + 1, n_states);

        flash_read_block(flash_block, i * EXT_FLASH_PAGE_SIZE, data_buffer, EXT_FLASH_PAGE_SIZE);

        memcpy(&rocket_state, data_buffer, sizeof(RocketState));

        size_t bytes_written;
        
        if (!lib_csv_encode(&rocket_state, line_buf, 2048, &bytes_written)) {
            log_printf(LOG_ERROR, "Error encoding rocket CSV line");
        }

        if (sd_write_file(sd_file, sd_bytes_written, (uint8_t *) line_buf, line_len)) {
            log_printf(LOG_INFO, "writing to SD card");
        }

        sd_bytes_written += line_len;
    }

    if (sd_close_file(sd_file)) {
        log_printf(LOG_INFO, "Closed SD card file");
    }

    log_printf(LOG_INFO, "Wrote to SD card");
}