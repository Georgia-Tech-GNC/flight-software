#include "flash_chip.h"
#include "main.h"

w25q_read_id(struct quadspi_flash_chip_t *device, uint8_t *buf) {
    OSPI_RegularCmdTypeDef cmd = {
        .OperationType = HAL_OSPI_OPTYPE_COMMON_CFG, // good
        // FlashId not needed ,

        .Instruction = , // TODO
        .InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE,
        .InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS,
        .InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE,

        .Address = 0x00U,
        .AddressMode = HAL_OSPI_ADDRESS_1_LINE,
        .AddressSize = HAL_OSPI_ADDRESS_24_BITS,
        .AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE,

        .AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
        // .AlternateBytesSize, .AlternateBytesDtrMode not needed

        .DataMode = HAL_OSPI_DATA_1_LINE,
        .NbData = 1,
        .DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE,

        .DummyCycles = 0,
        .DQSMode = HAL_OSPI_DQS_DISABLE,
        .SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD
    };

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

void init_w25q_flash_chip(quadspi_flash_chip_t *flash_chip) {

}

