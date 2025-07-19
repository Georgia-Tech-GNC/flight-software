#include "halal.h"
#include "log.h"
#include "util.h"
#include "debug.h"
#include "radio.h"
#include "state_estimation.h"
#include "storage.h"
#include "ff_gen_drv.h"

#ifdef HALAL_STORAGE_MODULE_ENABLED
Diskio_drvTypeDef diskio_driver = {
    .disk_initialize = HALAL_storage_initialize,
    .disk_status = HALAL_storage_status,
    .disk_read = HALAL_storage_read,
    .disk_write = HALAL_storage_write,
    .disk_ioctl = HALAL_storage_ioctl
};
#endif

static uint8_t HALAL_module_init(uint8_t (*init_function)(), const char *module_name);

uint8_t HALAL_init(void) {
#ifdef HALAL_DEBUG_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_debug_init, "debug")) return RET_FAILURE;
#endif

#ifdef HALAL_STORAGE_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_storage_init, "storage")) return RET_FAILURE;
    if (FATFS_LinkDriver(&diskio_driver, "") == 0) {
        log_printf(LOG_INFO, "FATFS driver linked");
    } else {
        log_printf(LOG_ERROR, "Error linking FATFS driver");
    }
#endif

#ifdef HALAL_RADIO_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_radio_init, "radio")) return RET_FAILURE;
#endif

#ifdef HALAL_STATE_ESTIMATION_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_state_estimation_init, "state estimation")) return RET_FAILURE;
#endif

#ifdef HALAL_ADC_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_adc_init, "ADC")) return RET_FAILURE;
#endif

    return RET_SUCCESS;
}

static uint8_t HALAL_module_init(uint8_t (*init_function)(), const char *module_name) {
    if (init_function()) {
        log_printf(LOG_INFO, "HALAL %s module initialized", module_name);
        return RET_SUCCESS;
    } else {
        log_printf(LOG_ERROR, "Error initializing HALAL %s module", module_name);
        return RET_FAILURE;
    }
}

/* HALAL Weak function definitions */
void __attribute__((weak)) HALAL_adc_convert_callback(uint32_t channel_uuid, uint16_t adc_value) {}

