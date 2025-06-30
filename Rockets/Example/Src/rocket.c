#include "rocket.h"

/**
 * This file should contain the definition for one freeRTOS task that will act as the "controller" for the rocket*
 * It will control the overall flow and state transitions of the launch, as well as interface with the hardware 
 * through the other provided tasks in the Tasks folder
 */
void run_rocket_task(void *args) {
    vTaskDelay(portMAX_DELAY);
}