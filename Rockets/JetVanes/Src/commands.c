#include "commands.h"

#include "jet_vanes.h"
#include "util.h"
#include "rtos_globals.h"
#include "state_tx.h"
#include "state_flash.h"
#include "state_estimation.h"

static uint8_t command_idle_to_ground(void);
static uint8_t command_flash_sd_card(void);

uint8_t process_command(RocketCommandID command_id) {
    switch (command_id) {
        case ROCKET_IDLE_TO_GROUND_COMMAND_ID:
            return command_idle_to_ground();
        case ROCKET_FLASH_SD_CARD_COMMAND_ID:
            return command_flash_sd_card();
        default:
            return RET_FAILURE;
    }
}

static uint8_t command_idle_to_ground(void) {
    HALAL_state_estimation_start();
    xTaskNotify(g_state_tx_task_handle, BEGIN_STATE_TX_NOTIFICATION_BIT, eSetBits);

    return RET_SUCCESS;
}

static uint8_t command_flash_sd_card(void) {
    xTaskNotify(g_state_flash_task_handle, FLASH_FS_NOTIFICATION_BIT, eSetBits);

    return RET_SUCCESS;
}

