#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include <stdint.h>

struct ring_buffer {
  uint8_t *buf;
  uint32_t size;
  uint32_t w_ptr;
  uint32_t r_ptr;
};

uint8_t ring_buffer_init(struct ring_buffer *ring_buffer, void *data_buffer,
                      uint32_t len);

uint32_t ring_buffer_write(struct ring_buffer *ring_buffer, void *data,
                       uint32_t len);

uint32_t ring_buffer_read(struct ring_buffer *ring_buffer, void *data,
                          uint32_t len);

uint32_t ring_buffer_peek(const struct ring_buffer *ring_buffer,
                          uint32_t skip, void *data, uint32_t len);

uint32_t ring_buffer_get_free(const struct ring_buffer *ring_buffer);

uint32_t ring_buffer_get_full(const struct ring_buffer *ring_buffer);

void *ring_buffer_get_logical_block_read_address(const struct ring_buffer *ring_buffer);

uint32_t ring_buffer_get_logical_block_read_length(const struct ring_buffer *ring_buffer);

uint32_t ring_buffer_skip(struct ring_buffer *ring_buffer, uint32_t len);

void *ring_buffer_get_logical_block_write_address(const struct ring_buffer *ring_buffer);

uint32_t ring_buffer_get_logical_block_write_length(const struct ring_buffer *ring_buffer);

uint32_t ring_buffer_advance(struct ring_buffer *ring_buffer, uint32_t len);

#endif /* __RING_BUFFER_H__ */