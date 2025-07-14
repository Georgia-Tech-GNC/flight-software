#include "halal.h"
#include "log.h"
#include "util.h"
#include "debug.h"
#include "radio.h"
#include "state_estimation.h"

static uint8_t HALAL_module_init(uint8_t (*init_function)(), const char *module_name);

uint8_t HALAL_init(void) {
#ifdef HALAL_DEBUG_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_debug_init, "debug")) return RET_FAILURE;
#endif

#ifdef HALAL_RADIO_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_radio_init, "radio")) return RET_FAILURE;
#endif

#ifdef HALAL_STATE_ESTIMATION_MODULE_ENABLED
    if (!HALAL_module_init(HALAL_state_estimation_init, "state estimation")) return RET_FAILURE;
#endif

    return RET_SUCCESS;
}

static uint8_t HALAL_module_init(uint8_t (*init_function)(), const char *module_name) {
    if (init_function()) {
        log_printf(LOG_INFO, "HALAL %s module initialized", module_name);
    } else {
        log_printf(LOG_ERROR, "HALAL %s module not initialized", module_name);
    }
}