#include "sd_test.h"

#include "FreeRTOS.h"
#include "task.h"

#include "port_config.h"

#include "main.h"
#include "periph_io.h"

#include "globals.h"

#include "ff.h"

#define CHUNK_SIZE 512

void sd_test_task(void *args) {
    
    IOChannel write_channel;
    IOChannel read_channel;

    StaticStreamBuffer_t sb_write_buff;
    uint8_t sb_write_storage_area[CHUNK_SIZE + 1];


    StaticStreamBuffer_t sb_read_buff;
    uint8_t sb_read_storage_area[CHUNK_SIZE + 1];

    sd_channel_init(&write_channel, "test_cpy.txt", IO_MODE_WRITE, 0, &sb_write_buff, sb_write_storage_area, CHUNK_SIZE);
    sd_channel_init(&read_channel, "test.txt", IO_MODE_READ, 1, &sb_read_buff, sb_read_storage_area, CHUNK_SIZE);

    uint8_t transfer_buf[CHUNK_SIZE + 1];
    uint32_t read_ptr = 0;

    uint32_t notify_value = 0;

    char *msg1 = "Doing things...\r\n";
    char *msg2 = "Done\r\n";

    while (1) {
        io_load_channel(&read_channel, read_ptr, CHUNK_SIZE);
        // wait for read to complete
        xTaskNotifyWait(0, SD_TASK_READ_COMPLETE_BIT, &notify_value, portMAX_DELAY);

        uint16_t n_bytes = io_channel_get_full(&read_channel);
        read_ptr += n_bytes;
        
        if (n_bytes == 0) {
            HAL_UART_Transmit(&debug_uart, (uint8_t *) msg2, 6, HAL_MAX_DELAY);
            while (1);
        }

        io_read_channel(&read_channel, transfer_buf, n_bytes);
        io_write_channel(&write_channel, transfer_buf, n_bytes);

        io_save_channel(&write_channel);

        // wait for write to complete
        xTaskNotifyWait(0, SD_TASK_WRITE_COMPLETE_BIT, &notify_value, portMAX_DELAY);
        HAL_UART_Transmit(&debug_uart, (uint8_t *) msg1, 17, HAL_MAX_DELAY);
    }

    while (1);
    
}