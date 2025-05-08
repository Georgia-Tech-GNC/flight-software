#include "ring_buffer.h"

#include <string.h>

#define BUF_IS_VALID(b) ((b) != NULL && (b)->buf != NULL && (b)->size > 0)
#define BUF_MIN(x, y)   ((x) < (y) ? (x) : (y))

uint8_t ring_buffer_init(struct ring_buffer *ring_buffer, void *data_buffer,
                      uint32_t len) {
  if (ring_buffer == NULL || data_buffer == NULL || len == 0) {
    return 0;
  }

  ring_buffer->size = len;
  ring_buffer->buf = data_buffer;
  ring_buffer->w_ptr = 0;
  ring_buffer->r_ptr = 0;

  return 1;
}

uint32_t ring_buffer_write(struct ring_buffer *ring_buffer, void *data,
                       uint32_t len) {
  uint32_t to_copy = 0;
  uint32_t free = 0;
  uint32_t w_ptr = 0;
  const uint8_t *d_ptr = (uint8_t *) data;

  if (!BUF_IS_VALID(ring_buffer) || data == NULL || len == 0) {
    return 0;
  }

  free = ring_buffer_get_free(ring_buffer);

  if ((free == 0) || (free < len)) {
    return 0;
  }

  len = BUF_MIN(free, len);
  w_ptr = ring_buffer->w_ptr;

  to_copy = BUF_MIN(ring_buffer->size - w_ptr, len);
  memcpy(&ring_buffer->buf[w_ptr], d_ptr, to_copy);
  d_ptr += to_copy;
  w_ptr += to_copy;
  len -= to_copy;

  if (len > 0) {
    memcpy(ring_buffer->buf, d_ptr, len);
    w_ptr = len;
  }

  if (w_ptr >= ring_buffer->size) {
    w_ptr = 0;
  }

  ring_buffer->w_ptr = w_ptr;

  return to_copy + len;
}

uint32_t ring_buffer_read(struct ring_buffer *ring_buffer, void *data,
                          uint32_t len) {
  uint32_t to_copy = 0;
  uint32_t full = 0;
  uint32_t r_ptr = 0;
  uint8_t *d_ptr = (uint8_t *) data;

  if (!BUF_IS_VALID(ring_buffer) || data == NULL || len == 0) {
    return 0;
  }

  full = ring_buffer_get_full(ring_buffer);
  if ((full == 0) || (full < len)) {
    return 0;
  }

  len = BUF_MIN(full, len);
  r_ptr = ring_buffer->r_ptr;

  to_copy = BUF_MIN(ring_buffer->size - r_ptr, len);
  memcpy(d_ptr, &ring_buffer->buf[r_ptr], to_copy);
  d_ptr += to_copy;
  r_ptr += to_copy;
  len -= to_copy;

  if (len > 0) {
    memcpy(d_ptr, ring_buffer->buf, len);
    r_ptr = len;
  }

  if (r_ptr >= ring_buffer->size) {
    r_ptr = 0;
  }

  ring_buffer->r_ptr = r_ptr;

  return to_copy + len;
}

uint32_t ring_buffer_peek(const struct ring_buffer *ring_buffer,
                          uint32_t skip, void *data, uint32_t len) {
  uint32_t full = 0;
  uint32_t to_copy = 0;
  uint32_t r_ptr = 0;
  uint8_t* d_ptr = (uint8_t *) data;

  if (!BUF_IS_VALID(ring_buffer) || data == NULL || len == 0) {
    return 0;
  }

  full = ring_buffer_get_full(ring_buffer);
  if (skip >= full) {
    return 0;
  }

  r_ptr = ring_buffer->r_ptr;
  r_ptr += skip;
  full -= skip;
  if (r_ptr >= ring_buffer->size) {
    r_ptr -= ring_buffer->size;
  }

  len = BUF_MIN(full, len);
  if (len == 0) {
    return 0;
  }

  to_copy = BUF_MIN(ring_buffer->size - r_ptr, len);
  memcpy(d_ptr, &ring_buffer->buf[r_ptr], to_copy);
  d_ptr += to_copy;
  len -= to_copy;

  if (len > 0) {
    memcpy(d_ptr, ring_buffer->buf, len);
  }

  return to_copy + len;
}

uint32_t ring_buffer_get_free(const struct ring_buffer *ring_buffer) {
  uint32_t size = 0;
  uint32_t w_ptr = 0;
  uint32_t r_ptr = 0;

  if (!BUF_IS_VALID(ring_buffer)) {
    return 0;
  }

  w_ptr = ring_buffer->w_ptr;
  r_ptr = ring_buffer->r_ptr;


  if (w_ptr >= r_ptr) {
    size = ring_buffer->size - (w_ptr - r_ptr);
  } else {
    size = r_ptr - w_ptr;
  }

  return size - 1;
}

uint32_t ring_buffer_get_full(const struct ring_buffer *ring_buffer) {
  uint32_t size = 0;
  uint32_t w_ptr = 0;
  uint32_t r_ptr = 0;

  if (!BUF_IS_VALID(ring_buffer)) {
    return 0; 
  }

  w_ptr = ring_buffer->w_ptr;
  r_ptr = ring_buffer->r_ptr;
  
  if (w_ptr > r_ptr) {
    size = w_ptr - r_ptr;
  } else {
    size = ring_buffer->size - (r_ptr - w_ptr);
  }

  return size;
}

void *ring_buffer_get_logical_block_read_address(const struct ring_buffer *ring_buffer) {
  uint32_t ptr = 0;

  if (!BUF_IS_VALID(ring_buffer)) {
    return NULL;
  }

  ptr = ring_buffer->r_ptr;
  return &ring_buffer->buf[ptr];
}

uint32_t ring_buffer_get_logical_block_read_length(const struct ring_buffer *ring_buffer) {
  uint32_t len = 0;
  uint32_t w_ptr = 0;
  uint32_t r_ptr = 0;

  if (!BUF_IS_VALID(ring_buffer)) {
    return 0;
  }

  w_ptr = ring_buffer->w_ptr;
  r_ptr = ring_buffer->r_ptr;

  if (w_ptr > r_ptr) {
    len = w_ptr - r_ptr;
  } else if (r_ptr > w_ptr) {
    len = ring_buffer->size - r_ptr;
  } else {
    len = 0;
  }

  return len;
}

uint32_t ring_buffer_skip(struct ring_buffer *ring_buffer, uint32_t len) {
  uint32_t full = 0;
  uint32_t r_ptr = 0;

  if (!BUF_IS_VALID(ring_buffer) || len == 0) {
    return 0;
  }

  full = ring_buffer_get_full(ring_buffer);
  len = BUF_MIN(len, full);
  r_ptr = ring_buffer->r_ptr;
  r_ptr += len;

  if (r_ptr >= ring_buffer->size) {
    r_ptr -= ring_buffer->size;
  }

  ring_buffer->r_ptr = r_ptr;

  return len;
}

void *ring_buffer_get_logical_block_write_address(const struct ring_buffer *ring_buffer) {
  uint32_t ptr = 0;

  if (!BUF_IS_VALID(ring_buffer)) {
    return NULL;
  }

  ptr = ring_buffer->w_ptr;
  return &ring_buffer->buf[ptr];
}

uint32_t ring_buffer_get_logical_block_write_length(const struct ring_buffer *ring_buffer) {
 uint32_t len = 0;
 uint32_t w_ptr = 0;
 uint32_t r_ptr = 0;

  if (!BUF_IS_VALID(ring_buffer)) {
    return 0;
  }

  w_ptr = ring_buffer->w_ptr;
  r_ptr = ring_buffer->r_ptr;

  if (w_ptr >= r_ptr) {
    len = ring_buffer->size - w_ptr;

    if (r_ptr == 0) {
      --len;
    }
  } else {
    len = r_ptr - w_ptr - 1;
  }

  return len;
}

uint32_t ring_buffer_advance(struct ring_buffer *ring_buffer, uint32_t len) {
  uint32_t free = 0;
  uint32_t w_ptr = 0;

  if (!BUF_IS_VALID(ring_buffer) || len == 0) {
    return 0;
  }

  free = ring_buffer_get_free(ring_buffer);
  len = BUF_MIN(len, free);
  w_ptr = ring_buffer->w_ptr;
  w_ptr += len;

  if (w_ptr >= ring_buffer->size) {
    w_ptr -= ring_buffer->size;
  }
  ring_buffer->w_ptr = w_ptr;

  return len;
}