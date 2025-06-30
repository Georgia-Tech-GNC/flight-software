uint32_t await_notification(uint32_t mask, TickType_t timeout) {
    uint32_t notification_value = 0;
    while ((notification_value & mask) == 0) {
        xTaskNotifyWait(0, mask, &notification_value, timeout);
    }

    return notification_value;
}

uint8_t memcpy_state_bytes(uint8_t *rocket_state) {
    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        /* Memcpy out so we can give back the mutex as fast as possible */
        memcpy(rocket_state, &g_current_state, sizeof(RocketState));
        xSemaphoreGive(g_state_mutex_handle);

        return 1;
    }

    return 0;
}

uint8_t memcpy_state(RocketState *rocket_state) {
    if (xSemaphoreTake(g_state_mutex_handle, portMAX_DELAY) == pdTRUE) {
        /* Memcpy out so we can give back the mutex as fast as possible */
        memcpy(rocket_state, &g_current_state, sizeof(RocketState));
        xSemaphoreGive(g_state_mutex_handle);

        return 1;
    }

    return 0;
}