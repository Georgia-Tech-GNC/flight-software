uint32_t await_notification(uint32_t mask, TickType_t timeout) {
    uint32_t notification_value = 0;
    while ((notification_value & mask) == 0) {
        xTaskNotifyWait(0, mask, &notification_value, timeout);
    }

    return notification_value;
}

uint8_t memcpy_state_bytes(uint8_t *rocket_state, size_t buffer_size, size_t *bytes_copied) {
    if (!assert(sizeof(RocketState) <= buffer_size, "Rocket state size less than buffer size")) {
        return 0;
    }

    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        memcpy(rocket_state, &g_current_state, sizeof(RocketState));
        xSemaphoreGive(g_state_mutex_handle);

        return 1;
    }

    return 0;
}

uint8_t memcpy_state(RocketState *rocket_state) {
    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        memcpy(rocket_state, &g_current_state, sizeof(RocketState));
        xSemaphoreGive(g_state_mutex_handle);

        return 1;
    }

    return 0;
}

uint8_t assert(uint8_t val, const char *assert_name) {
    if (!val) {
        log_printf("Assert failed: '%s'", assert_name);
    }

    return val;
}