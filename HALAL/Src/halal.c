#include "halal.h"
#include "log.h"
#include "util.h"
#include "debug.h"
#include "radio.h"
#include "state_estimation.h"
#include "timebase.h"
#include "rcc.h"
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
static uint8_t HALAL_ll_init(void);


uint8_t HALAL_init(void) {
    if (!HALAL_ll_init()) return RET_FAILURE;

#ifdef HALAL_DEBUG_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_debug_init, "debug")) return RET_FAILURE;
#endif

    uint8_t status = RET_SUCCESS;

#ifdef HALAL_STORAGE_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_storage_init, "storage")) status = RET_FAILURE;
    if (FATFS_LinkDriver(&diskio_driver, "") == 0) {
        log_printf(LOG_INFO, "FATFS driver linked");
    } else {
        log_printf(LOG_ERROR, "Error linking FATFS driver");
    }
#endif

#ifdef HALAL_RADIO_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_radio_init, "radio")) status = RET_FAILURE;
#endif

#ifdef HALAL_STATE_ESTIMATION_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_state_estimation_init, "state estimation")) status = RET_FAILURE;
#endif

#ifdef HALAL_ADC_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_adc_init, "ADC")) status = RET_FAILURE;
#endif

#ifdef HALAL_FLASH_CHIP_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_flash_init, "flash chip")) status = RET_FAILURE;
#endif

#ifdef HALAL_BAROMETER_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_barometer_init, "barometer")) status = RET_FAILURE;
#endif

#ifdef HALAL_MAGNETOMETER_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_magnetometer_initialize, "magnetometer")) status = RET_FAILURE;
#endif

    return status;
}

static uint8_t HALAL_module_init(uint8_t (*init_function)(), const char *module_name) {
    log_printf(LOG_INFO, "Initializing HALAL %s module...", module_name);
    if (init_function()) {
        log_printf(LOG_INFO, "HALAL %s module initialized", module_name);
        return RET_SUCCESS;
    } else {
        log_printf(LOG_ERROR, "Error initializing HALAL %s module", module_name);
        return RET_FAILURE;
    }
}

static uint8_t HALAL_ll_init(void) {
    if (HAL_Init() != HAL_OK) return RET_FAILURE;
    if (!HALAL_rcc_init()) return RET_FAILURE;

    return RET_SUCCESS;
}

/* HALAL Weak function definitions */
void __attribute__((weak)) HALAL_adc_convert_callback(uint32_t channel_uuid, uint32_t adc_value, BaseType_t *xHigherPriorityTaskWoken) {
    UNUSED(channel_uuid);
    UNUSED(adc_value);
    UNUSED(xHigherPriorityTaskWoken);
}

void __attribute__((weak)) HALAL_state_estimation_callback(uint8_t *state_estimation_bytes, size_t size, BaseType_t *xHigherPriorityTaskWoken) {
    UNUSED(state_estimation_bytes);
    UNUSED(size);
    UNUSED(xHigherPriorityTaskWoken);
}

void __attribute__((weak)) HALAL_radio_callback(uint8_t *radio_bytes, size_t size, BaseType_t *xHigherPriorityTaskWoken) {
    UNUSED(radio_bytes);
    UNUSED(size);
    UNUSED(xHigherPriorityTaskWoken);
}
