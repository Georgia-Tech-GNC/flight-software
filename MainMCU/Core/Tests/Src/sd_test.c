#include "sd_test.h"

#include "FreeRTOS.h"
#include "task.h"

#include "port_config.h"

#include "main.h"
#include "sdio.h"

#include "ff.h"

#define CHUNK_SIZE 512

void sd_test_task(void *args) {
    
    SDChannel write_channel;
    SDChannel read_channel;

    StaticStreamBuffer_t sb_write_buff;
    uint8_t sb_write_storage_area[CHUNK_SIZE + 1];


    StaticStreamBuffer_t sb_read_buff;
    uint8_t sb_read_storage_area[CHUNK_SIZE + 1];

    sd_channel_init(&write_channel, "test_cpy.txt", SD_MODE_WRITE, 0, &sb_write_buff, sb_write_storage_area, CHUNK_SIZE);
    sd_channel_init(&read_channel, "test.txt", SD_MODE_READ, 1, &sb_read_buff, sb_read_storage_area, CHUNK_SIZE);

    uint8_t transfer_buf[CHUNK_SIZE + 1];
    uint32_t read_ptr = 0;

    uint32_t notify_value = 0;

    /*
    uint8_t dummy_data[CHUNK_SIZE];

    for (int i = 0; i < CHUNK_SIZE - 1; i ++) {
        dummy_data[i] = 'f';
    }

    dummy_data[CHUNK_SIZE - 1] = '\n';

    FATFS fs;

    if (f_mount(&fs, "/", 1) != FR_OK) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
    }
    */

    while (1) {
        sd_load_channel(&read_channel, read_ptr, CHUNK_SIZE);
        // wait for read to complete
        xTaskNotifyWait(0, SD_TASK_READ_COMPLETE_BIT, &notify_value, portMAX_DELAY);

        uint16_t n_bytes = sd_channel_get_full(&read_channel);
        read_ptr += n_bytes;
        
        if (n_bytes == 0) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
            while (1);
        }

        sd_read_channel(&read_channel, transfer_buf, n_bytes);
        sd_write_channel(&write_channel, transfer_buf, n_bytes);

        // wait for write to complete
        xTaskNotifyWait(0, SD_TASK_WRITE_COMPLETE_BIT, &notify_value, portMAX_DELAY);

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

       /*
        FIL fil;
        if (f_open(&fil, "test_cpy.txt", FA_WRITE | FA_OPEN_APPEND) != FR_OK) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
        } 

        UINT written;
        if (f_write(&fil, dummy_data, CHUNK_SIZE, &written) != FR_OK) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
        }
        
        if (f_close(&fil) != FR_OK) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
        }

        vTaskDelay(1000);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
    */
    }

    while (1);
    
}