#ifndef __W25Q_H__
#define __W25Q_H__

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define W25Q_MEM_FLASH_SIZE  32U                      // 32 MB-bit
#define W25Q_MEM_BLOCK_SIZE  64U                       // 64 KB: 256 pages
#define W25Q_MEM_SBLOCK_SIZE 32U                       // 32 KB: 128 pages
#define W25Q_MEM_SECTOR_SIZE 4U                        // 4 KB : 16 pages
#define W25Q_MEM_PAGE_SIZE   256U                      // 256 byte : 1 page
#define W25Q_BLOCK_COUNT     (W25Q_MEM_FLASH_SIZE * 2) // 512 blocks
#define W25Q_SECTOR_COUNT    (W25Q_BLOCK_COUNT * 16)   // 8192 sectors
#define W25Q_PAGE_COUNT      (W25Q_SECTOR_COUNT * 16)  // 131'072 pages

#define W25Q_WRITE_ENABLE             0x06U
#define W25Q_WRITE_DISABLE            0x04U
#define W25Q_ENABLE_VOLATILE_SR       0x50U
#define W25Q_READ_SR1                 0x05U
#define W25Q_READ_SR2                 0x35U
#define W25Q_READ_SR3                 0x15U
#define W25Q_WRITE_SR1                0x01U
#define W25Q_WRITE_SR2                0x31U
#define W25Q_WRITE_SR3                0x11U
#define W25Q_READ_EXT_ADDR_REG        0xC8U
#define W25Q_WRITE_EXT_ADDR_REG       0xC8U
#define W25Q_ENABLE_4B_MODE           0xB7U
#define W25Q_DISABLE_4B_MODE          0xE9U
#define W25Q_READ_DATA                0x03U
#define W25Q_READ_DATA_4B             0x13U
#define W25Q_FAST_READ                0x0BU
#define W25Q_FAST_READ_4B             0x0CU
#define W25Q_FAST_READ_DUAL_OUT       0x3BU
#define W25Q_FAST_READ_DUAL_OUT_4B    0x3CU
#define W25Q_FAST_READ_QUAD_OUT       0x6BU
#define W25Q_FAST_READ_QUAD_OUT_4B    0x6CU
#define W25Q_FAST_READ_DUAL_IO        0xBBU
#define W25Q_FAST_READ_DUAL_IO_4B     0xBCU
#define W25Q_FAST_READ_QUAD_IO        0xEBU
#define W25Q_FAST_READ_QUAD_IO_4B     0xECU
#define W25Q_SET_BURST_WRAP           0x77U
#define W25Q_PAGE_PROGRAM             0x02U
#define W25Q_PAGE_PROGRAM_4B          0x12U
#define W25Q_PAGE_PROGRAM_QUAD_INP    0x32U
#define W25Q_PAGE_PROGRAM_QUAD_INP_4B 0x34U
#define W25Q_SECTOR_ERASE             0x20U
#define W25Q_SECTOR_ERASE_4B          0x21U
#define W25Q_32KB_BLOCK_ERASE         0x52U
#define W25Q_64KB_BLOCK_ERASE         0xD8U
#define W25Q_64KB_BLOCK_ERASE_4B      0xDCU
#define W25Q_CHIP_ERASE               0xC7U
//#define W25Q_CHIP_ERASE 0x60U
#define W25Q_ERASEPROG_SUSPEND  0x75U
#define W25Q_ERASEPROG_RESUME   0x7AU
#define W25Q_POWERDOWN          0xB9U
#define W25Q_POWERUP            0xABU
#define W25Q_DEVID              0xABU
#define W25Q_FULLID             0x90U
#define W25Q_FULLID_DUAL_IO     0x92U
#define W25Q_FULLID_QUAD_IO     0x94U
#define W25Q_READ_UID           0x4BU
#define W25Q_READ_JEDEC_ID      0x9FU
#define W25Q_READ_SFDP          0x5AU
#define W25Q_ERASE_SECURITY_REG 0x44U
#define W25Q_PROG_SECURITY_REG  0x42U
#define W25Q_READ_SECURITY_REG  0x48U
#define W25Q_IND_BLOCK_LOCK     0x36U
#define W25Q_IND_BLOCK_UNLOCK   0x39U
#define W25Q_READ_BLOCK_LOCK    0x3DU
#define W25Q_GLOBAL_LOCK        0x7EU
#define W25Q_GLOBAL_UNLOCK      0x98U
#define W25Q_ENABLE_RST         0x66U
#define W25Q_RESET              0x99U

enum w25q_err {
  W25Q_ERR_OK,
  W25Q_ERR_BUSY,
  W25Q_ERR_PARAM,
  W25Q_ERR_CHIP,
  W25Q_ERR_SPI,
  W25Q_ERR_CHIP_IGNORE,
  W25Q_ERR_INVALID_ID,
};

struct w25q_status_reg {
  uint8_t BUSY : 1;
  uint8_t WEL : 1;
  uint8_t BP : 4;
  uint8_t TB : 1;
  uint8_t SRP : 1;

  uint8_t SRL : 1;
  uint8_t QE : 1;
  uint8_t RES0 : 1;
  uint8_t LB : 3;
  uint8_t CMP : 1;
  uint8_t SUS : 1;

  uint8_t ADS : 1;
  uint8_t ADP : 1;
  uint8_t WPS : 1;
  uint8_t RES1 : 2;
  uint8_t DRV : 2;
  uint8_t HR : 1;
};

struct w25q_device {
  void *hospi;
  struct w25q_status_reg status;
};

enum w25q_err w25q_init(struct w25q_device *device);
enum w25q_err w25q_update_status_struct(struct w25q_device *device);
enum w25q_err w25q_is_busy(struct w25q_device *device);
enum w25q_err w25q_read_data(struct w25q_device *device, uint8_t *buf,
                             uint32_t len, uint8_t page_offset,
                             uint32_t page_num);

void w25q_delay(uint32_t ms);
enum w25q_err w25q_read_id(struct w25q_device *device, uint8_t *buf);
enum w25q_err w25q_write_enable(struct w25q_device *device, bool enable);
enum w25q_err w25q_enter_4_byte_mode(struct w25q_device *device, bool enable);
enum w25q_err w25q_read_status_reg(struct w25q_device *device,
                                   uint8_t *reg_data, uint8_t reg_num);
enum w25q_err w25q_write_status_reg(struct w25q_device *device,
                                    uint8_t reg_data, uint8_t reg_num);
enum w25q_err w25q_read_raw(struct w25q_device *device, uint8_t *buf,
                            uint32_t len, uint32_t addr);
enum w25q_err w25q_write_raw(struct w25q_device *device, uint8_t *buf,
                             uint32_t len, uint32_t addr);
enum w25q_err w25q_erase_sector(struct w25q_device *device,
                                uint32_t sector_addr);
enum w25q_err w25q_erase_block(struct w25q_device *device, uint32_t block_addr,
                               uint8_t size);
enum w25q_err w25q_erase_chip(struct w25q_device *device);
enum w25q_err w25q_reset(struct w25q_device *device);

#endif /* __W25Q_H__ */