#include "w25q.h"

#include <string.h>
#include "globals.h"

enum w25q_err w25q_init(struct w25q_device *device) {
  enum w25q_err err;

  uint8_t id;
  err = w25q_read_id(device, &id);
  if (err != W25Q_ERR_OK) {
    return err;
  }

  if (id != 0x15) {
    return W25Q_ERR_INVALID_ID;
  }

  err = w25q_update_status_struct(device);
  if (err != W25Q_ERR_OK) {
    return err;
  }

#if W25Q_MEM_FLASH_SIZE > 128

  if (!device->status.ADP) {
    uint8_t reg;
    err = w25q_read_status_reg(device, &reg, 3);
    if (err != W25Q_ERR_OK) {
      return err;
    }
    reg |= 0x02;
    err = w25q_write_status_reg(device, reg, 3);
    if (err != W25Q_ERR_OK) {
      return err;
    }
  }

  if (!device->status.ADS) {
    err = w25q_enter_4_byte_mode(device, 1);
    if (err != W25Q_ERR_OK) {
      return err;
    }
  }

#endif

  if (!device->status.QE) {
    uint8_t reg;

    err = w25q_read_status_reg(device, &reg, 2);
    if (err != W25Q_ERR_OK) {
      return err;
    }

    reg |= 0x02;
    err = w25q_write_status_reg(device, reg, 2);
    if (err != W25Q_ERR_OK) {
      return err;
    }

  }

  err = w25q_update_status_struct(device);

  return err;
}

enum w25q_err w25q_update_status_struct(struct w25q_device *device) {
  enum w25q_err err;

  uint8_t sr[3];

  err = w25q_read_status_reg(device, &sr[0], 1);
  if (err != W25Q_ERR_OK) {
    return err;
  }

  err = w25q_read_status_reg(device, &sr[1], 2);
  if (err != W25Q_ERR_OK) {
    return err;
  }

  err = w25q_read_status_reg(device, &sr[2], 3);
  if (err != W25Q_ERR_OK) {
    return err;
  }

  memcpy(&(device->status), sr, 3);

  return err;
}

enum w25q_err w25q_is_busy(struct w25q_device *device) {
  enum w25q_err err;
  uint8_t sr;

  err = w25q_read_status_reg(device, &sr, 1);
  if (err != W25Q_ERR_OK) {
    return err;
  }

  device->status.BUSY = sr & 1;

  return device->status.BUSY ? W25Q_ERR_BUSY : W25Q_ERR_OK;
}

enum w25q_err w25q_read_data(struct w25q_device *device, uint8_t *buf,
                             uint32_t len, uint8_t page_offset,
                             uint32_t page_num) {
  if (page_num >= W25Q_PAGE_COUNT || len == 0 || len > 256 ||
      page_offset > 256 - len) {
    return W25Q_ERR_PARAM;
  }

  uint32_t raw_addr = page_num * W25Q_MEM_PAGE_SIZE + page_offset;
  return w25q_read_raw(device, buf, len, raw_addr);
}

__attribute__((weak)) void w25q_delay(uint32_t ms) { return; }

__attribute__((weak)) enum w25q_err w25q_read_id(struct w25q_device *device,
                                                 uint8_t *buf) {
  return W25Q_ERR_OK;
}

__attribute__((weak)) enum w25q_err
w25q_write_enable(struct w25q_device *device, bool enable) {
  return W25Q_ERR_OK;
}

__attribute__((weak)) enum w25q_err
w25q_enter_4_byte_mode(struct w25q_device *device, bool enable) {
  return W25Q_ERR_OK;
}

__attribute__((weak)) enum w25q_err
w25q_read_status_reg(struct w25q_device *device, uint8_t *reg_data,
                     uint8_t reg_num) {
  return W25Q_ERR_OK;
}

__attribute__((weak)) enum w25q_err
w25q_write_status_reg(struct w25q_device *device, uint8_t reg_data,
                      uint8_t reg_num) {
  return W25Q_ERR_OK;
}

__attribute__((weak)) enum w25q_err w25q_read_raw(struct w25q_device *device,
                                                  uint8_t *buf, uint32_t len,
                                                  uint32_t addr) {
  return W25Q_ERR_OK;
}

__attribute__((weak)) enum w25q_err w25q_write_raw(struct w25q_device *device,
                                                   uint8_t *buf, uint32_t len,
                                                   uint32_t addr) {
  return W25Q_ERR_OK;
}

__attribute__((weak)) enum w25q_err
w25q_erase_sector(struct w25q_device *device, uint32_t sector_addr) {
  return W25Q_ERR_OK;
}

__attribute__((weak)) enum w25q_err w25q_erase_block(struct w25q_device *device,
                                                     uint32_t block_addr,
                                                     uint8_t size) {
  return W25Q_ERR_OK;
}

__attribute__((weak)) enum w25q_err
w25q_erase_chip(struct w25q_device *device) {
  return W25Q_ERR_OK;
}

__attribute__((weak)) enum w25q_err w25q_reset(struct w25q_device *device) {
  return W25Q_ERR_OK;
}