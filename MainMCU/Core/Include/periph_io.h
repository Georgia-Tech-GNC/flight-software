#ifndef periph_io_H
#define periph_io_H

#include "stdint.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "state_flash.h"
#include "w25q.h"

#define MAX_IO_OPERATIONS_QUEUED 4
#define IO_MB_SIZE (sizeof(IOOperation) * MAX_IO_OPERATIONS_QUEUED)

#define SD_MAX_READ_WRITE_SIZE 512
#define FLASH_MAX_READ_WRITE_SIZE 256

#define IO_MODE_READ 0
#define IO_MODE_WRITE 1

#define IO_OPERATION_LOAD 0
#define IO_OPERATION_SAVE 1
#define IO_OPERATION_RESET 2

#define IO_CHANNEL_TYPE_SD 0
#define IO_CHANNEL_TYPE_FLASH 1

typedef struct {
    const char *file_path;
    uint8_t type;
    uint8_t mode;
    uint8_t id;
    StreamBufferHandle_t sb_handle;
} IOChannel;

typedef struct {
    uint8_t type;
    IOChannel *channel;
    size_t offset; /* Only used if type is IO_MODE_READ */
    size_t n_bytes; /* Only used if type is IO_MODE_READ */
} IOOperation;

int sd_channel_init(IOChannel *channel, const char *file_name, uint8_t mode, uint8_t id, StaticStreamBuffer_t *sb_buff, uint8_t *sb_storage_area, size_t sb_size);
int flash_channel_init(IOChannel *channel, uint8_t mode, uint8_t id, StaticStreamBuffer_t *sb_buff, uint8_t *sb_storage_area, size_t sb_size);

size_t io_channel_get_free(IOChannel *channel);
size_t io_channel_get_full(IOChannel *channel);
int io_write_channel(IOChannel *channel, uint8_t *data, size_t len);
int io_save_channel(IOChannel *channel);
int io_reset_channel(IOChannel *channel);
int io_load_channel(IOChannel *channel, size_t offset, size_t bytes_to_read);
int io_read_channel(IOChannel *channel, uint8_t *data, size_t len);
void periph_io_task(void *args);

#endif