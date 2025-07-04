#include "w25q.h"
#include "port_config.h"
#include "globals.h"

void w25q_delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

enum w25q_err w25q_read_id(struct w25q_device *device, uint8_t *buf) {
  OSPI_RegularCmdTypeDef cmd = {0};

  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;

  cmd.Instruction = W25Q_DEVID;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

  cmd.Address = 0x00U;
  cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
  cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
  cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;

  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;

  cmd.DataMode = HAL_OSPI_DATA_1_LINE;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
  cmd.NbData = 1;

  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);

  if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  if (HAL_OSPI_Receive(&flash_spi, buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

  return W25Q_ERR_OK;
}

enum w25q_err w25q_write_enable(struct w25q_device *device, bool enable) {
  OSPI_RegularCmdTypeDef cmd = {0};

  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;

  cmd.Instruction = enable ? W25Q_WRITE_ENABLE : W25Q_WRITE_DISABLE;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

  cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;

  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;

  cmd.DataMode = HAL_OSPI_DATA_NONE;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
  cmd.NbData = 0;

  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);

  if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

  w25q_delay(1);

  return W25Q_ERR_OK;
}

enum w25q_err w25q_enter_4_byte_mode(struct w25q_device *device, bool enable) {
  while (w25q_is_busy(device) == W25Q_ERR_BUSY)
    ;

  OSPI_RegularCmdTypeDef cmd = {0};

  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;

  cmd.Instruction = enable ? W25Q_ENABLE_4B_MODE : W25Q_DISABLE_4B_MODE;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

  cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;

  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;

  cmd.DataMode = HAL_OSPI_DATA_NONE;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
  cmd.NbData = 0;

  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);

  if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

  w25q_delay(1);

  return w25q_update_status_struct(device);
}

enum w25q_err w25q_read_status_reg(struct w25q_device *device,
                                   uint8_t *reg_data, uint8_t reg_num) {
  OSPI_RegularCmdTypeDef cmd = {0};

  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;

  if (reg_num == 1) {
    cmd.Instruction = W25Q_READ_SR1;
  } else if (reg_num == 2) {
    cmd.Instruction = W25Q_READ_SR2;
  } else if (reg_num == 3) {
    cmd.Instruction = W25Q_READ_SR3;
  } else {
    return W25Q_ERR_PARAM;
  }

  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

  cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;

  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;

  cmd.DataMode = HAL_OSPI_DATA_1_LINE;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
  cmd.NbData = 1;

  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);

  if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  // cli_transmit(&cli, "KEBAB 2\r\n");
  if (HAL_OSPI_Receive(&flash_spi, reg_data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

  return W25Q_ERR_OK;
}

enum w25q_err w25q_write_status_reg(struct w25q_device *device,
                                    uint8_t reg_data, uint8_t reg_num) {
  while (w25q_is_busy(device) == W25Q_ERR_BUSY)
    ;

  enum w25q_err err = w25q_write_enable(device, 1);
  if (err != W25Q_ERR_OK) {
    return err;
  }

  OSPI_RegularCmdTypeDef cmd = {0};

  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;

  if (reg_num == 1) {
    cmd.Instruction = W25Q_WRITE_SR1;
  } else if (reg_num == 2) {
    cmd.Instruction = W25Q_WRITE_SR2;
  } else if (reg_num == 3) {
    cmd.Instruction = W25Q_WRITE_SR3;
  } else {
    return W25Q_ERR_PARAM;
  }

  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

  cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;

  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;

  cmd.DataMode = HAL_OSPI_DATA_1_LINE;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
  cmd.NbData = 1;

  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);

  if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  if (HAL_OSPI_Transmit(&flash_spi, &reg_data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

  return W25Q_ERR_OK;
}

// enum w25q_err w25q_read_raw(struct w25q_device *device, uint8_t *buf,
//                             uint32_t len, uint32_t addr) {
//   if (len > 256 || len == 0) {
//     return W25Q_ERR_PARAM;
//   }
//
//   while (w25q_is_busy(device) == W25Q_ERR_BUSY);
//
//   OSPI_RegularCmdTypeDef cmd = {0};
//
//   cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
//
// #if W25Q_MEM_FLASH_SIZE > 128U
//
//   cmd.Instruction = W25Q_READ_DATA_4B;
//   cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
//
// #else
//
//   cmd.Instruction = W25Q_READ_DATA;
//   cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
//
// #endif
//
//   cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
//   cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
//   cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
//
//   cmd.Address = addr;
//   cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
//   cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
//
//   cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
//
//   cmd.DataMode = HAL_OSPI_DATA_1_LINE;
//   cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
//   cmd.NbData = len;
//
//   cmd.DummyCycles = 0;
//   cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
//   cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
//
//   if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
//       HAL_OK) {
//     return W25Q_ERR_SPI;
//   }
//
//   if (HAL_OSPI_Receive(&flash_spi, buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
//       HAL_OK) {
//     return W25Q_ERR_SPI;
//   }
//
//   return W25Q_ERR_OK;
//
//
// }

enum w25q_err w25q_read_raw(struct w25q_device *device, uint8_t *buf,
                            uint32_t len, uint32_t addr) {
  if (len > 256 || len == 0) {
    return W25Q_ERR_PARAM;
  }

  while (w25q_is_busy(device) == W25Q_ERR_BUSY)
    ;

  OSPI_RegularCmdTypeDef cmd = {0};

  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;

#if W25Q_MEM_FLASH_SIZE > 128U

  cmd.Instruction = W25Q_FAST_READ_QUAD_IO_4B;
  cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;

#else

  cmd.Instruction = W25Q_FAST_READ_QUAD_IO;
  cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;

#endif

  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

  cmd.Address = addr;
  cmd.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
  cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;

  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;

  cmd.DataMode = HAL_OSPI_DATA_4_LINES;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
  cmd.NbData = len;

  cmd.DummyCycles = 6;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);

  if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  if (HAL_OSPI_Receive(&flash_spi, buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

    HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

  return W25Q_ERR_OK;
}

// enum w25q_err w25q_write_raw(struct w25q_device *device, uint8_t *buf,
//                              uint32_t len, uint32_t addr) {
//   if (len > 256 || len == 0) {
//     return W25Q_ERR_PARAM;
//   }
//
//   while (w25q_is_busy(device) == W25Q_ERR_BUSY);
//
//   enum w25q_err err = w25q_write_enable(device, 1);
//   if (err != W25Q_ERR_OK) {
//     return err;
//   }
//
//   OSPI_RegularCmdTypeDef cmd = {0};
//
//   cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
//
// #if W25Q_MEM_FLASH_SIZE > 128U
//
//   cmd.Instruction = W25Q_PAGE_PROGRAM_4B;
//   cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
//
// #else
//
//   cmd.Instruction = W25Q_PAGE_PROGRAM;
//   cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
//
// #endif
//
//   cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
//   cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
//   cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
//
//   cmd.Address = addr;
//   cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
//   cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
//
//   cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
//
//   cmd.DataMode = HAL_OSPI_DATA_1_LINE;
//   cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
//   cmd.NbData = len;
//
//   cmd.DummyCycles = 0;
//   cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
//   cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
//
//   if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
//       HAL_OK) {
//     return W25Q_ERR_SPI;
//   }
//
//   if (HAL_OSPI_Transmit(&flash_spi, buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
//       HAL_OK) {
//     return W25Q_ERR_SPI;
//   }
//
//   return W25Q_ERR_OK;
// }

enum w25q_err w25q_write_raw(struct w25q_device *device, uint8_t *buf,
                             uint32_t len, uint32_t addr) {
  if (len > 256 || len == 0) {
    return W25Q_ERR_PARAM;
  }

  while (w25q_is_busy(device) == W25Q_ERR_BUSY)
    ;

  enum w25q_err err = w25q_write_enable(device, 1);
  if (err != W25Q_ERR_OK) {
    return err;
  }

  OSPI_RegularCmdTypeDef cmd = {0};

  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;

#if W25Q_MEM_FLASH_SIZE > 128U

  cmd.Instruction = W25Q_PAGE_PROGRAM_QUAD_INP_4B;
  cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;

#else

  cmd.Instruction = W25Q_PAGE_PROGRAM_QUAD_INP;
  cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;

#endif

  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

  cmd.Address = addr;
  cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
  cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;

  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;

  cmd.DataMode = HAL_OSPI_DATA_4_LINES;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
  cmd.NbData = len;

  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);

  if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  if (HAL_OSPI_Transmit(&flash_spi, buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

  return W25Q_ERR_OK;
}

enum w25q_err w25q_erase_sector(struct w25q_device *device,
                                uint32_t sector_addr) {
  if (sector_addr >= W25Q_SECTOR_COUNT) {
    return W25Q_ERR_PARAM;
  }

  while (w25q_is_busy(device) == W25Q_ERR_BUSY)
    ;

  enum w25q_err err = w25q_write_enable(device, 1);
  if (err != W25Q_ERR_OK) {
    return err;
  }

  OSPI_RegularCmdTypeDef cmd = {0};

  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;

#if W25Q_MEM_FLASH_SIZE > 128U

  cmd.Instruction = W25Q_SECTOR_ERASE_4B;
  cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;

#else

  cmd.Instruction = W25Q_SECTOR_ERASE;
  cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;

#endif

  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

  cmd.Address = sector_addr * W25Q_MEM_SECTOR_SIZE << 10;
  cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
  cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;

  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;

  cmd.DataMode = HAL_OSPI_DATA_NONE;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;

  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);

  if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

  while (w25q_is_busy(device) == W25Q_ERR_BUSY)
    ;

  return W25Q_ERR_OK;
}

enum w25q_err w25q_erase_block(struct w25q_device *device, uint32_t block_addr,
                               uint8_t size) {
  if ((size != 32) && (size != 64)) {
    return W25Q_ERR_PARAM;
  }

  if ((size == 64 && block_addr >= W25Q_BLOCK_COUNT) ||
      (size == 32 && block_addr >= W25Q_BLOCK_COUNT * 2)) {
    return W25Q_ERR_PARAM;
  }

  while (w25q_is_busy(device) == W25Q_ERR_BUSY)
    ;

  uint32_t raw_addr = block_addr * W25Q_MEM_SECTOR_SIZE << 14;

  if (size == 32) {
    raw_addr >>= 1;
  }

  enum w25q_err err = w25q_write_enable(device, 1);
  if (err != W25Q_ERR_OK) {
    return err;
  }

  OSPI_RegularCmdTypeDef cmd = {0};

  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;

  if (size == 32) {
    cmd.Instruction = W25Q_32KB_BLOCK_ERASE;

#if W25Q_MEM_FLASH_SIZE > 128U

    cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;

#else

    cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;

#endif

  } else if (size == 64) {

#if W25Q_MEM_FLASH_SIZE > 128U

    cmd.Instruction = W25Q_64KB_BLOCK_ERASE_4B;
    cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;

#else

    cmd.Instruction = W25Q_64KB_BLOCK_ERASE;
    cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;

#endif
  }

  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

  cmd.Address = raw_addr;
  cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
  cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;

  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;

  cmd.DataMode = HAL_OSPI_DATA_NONE;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;

  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);

  if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

  while (w25q_is_busy(device) == W25Q_ERR_BUSY)
    ;

  return W25Q_ERR_OK;
}

enum w25q_err w25q_erase_chip(struct w25q_device *device) {
  while (w25q_is_busy(device) == W25Q_ERR_BUSY)
    ;

  enum w25q_err err = w25q_write_enable(device, 1);
  if (err != W25Q_ERR_OK) {
    return err;
  }

  OSPI_RegularCmdTypeDef cmd = {0};

  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;

  cmd.Instruction = W25Q_CHIP_ERASE;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

  cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
  cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;

  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;

  cmd.DataMode = HAL_OSPI_DATA_NONE;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;

  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);

  if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

  while (w25q_is_busy(device) == W25Q_ERR_BUSY)
    ;


  return W25Q_ERR_OK;
}

enum w25q_err w25q_reset(struct w25q_device *device) {
  enum w25q_err err;

  err = w25q_update_status_struct(device);
  if (err != W25Q_ERR_OK) {
    return err;
  }

  if (device->status.BUSY || device->status.SUS) {
    return W25Q_ERR_CHIP;
  }

  while (w25q_is_busy(device) == W25Q_ERR_BUSY)
    ;

  // if (device->status.SUS) {
  //   w25q_write_resume();
  // }

  OSPI_RegularCmdTypeDef cmd = {0};

  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;

  cmd.Instruction = W25Q_ENABLE_RST;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

  cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
  cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;

  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;

  cmd.DataMode = HAL_OSPI_DATA_NONE;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;

  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_RESET);

  if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  w25q_delay(1);

  cmd.Instruction = W25Q_RESET;

  if (HAL_OSPI_Command(&flash_spi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return W25Q_ERR_SPI;
  }

  HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_SET);

  w25q_delay(5);

  return err;
}