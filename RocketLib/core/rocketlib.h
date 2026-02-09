#pragma once
/**
 * @file rocketlib.h
 * @brief Handles primary code execution from main.c and includes the mandatory 
 * initialization methods implemented by each rocket.
*/

#include "utils.h"

/** Entry point for all user code 
 * 
 * This method is called by main.c once the HAL has been successfully initialized.
 * It handles the initialization and execution of all rocket-related processes.
 * This method is not intended to ever halt.
 * 
 * Specifically, this method calls a set of methods (listed below), which should 
 * be defined by he currently selected rocket. All initialization regarding 
 * device drivers and FreeRTOS task creation should be handled in these methods.
 * 
 * Assuming that the rocket is successfully initialized, the FreeRTOS task scheduler
 * will then start. The scheduler will run until the program is physically reset.
 * 
 * Initialization order:
 * 1. initialize_debug() -> This method should initialize the bare minimum required 
 *      to log debug messages. After this method returns, it is assumed that the from
 *      that point onwards, calls to debug(const char *restrict format, ...) are 
 *      assumed to be valid. Note that this method is weakly defined as a no-op in
 *      rocketlib.c and should be overriden by each rocket to support the appropriate 
 *      debug interface
 * 
 * 2. initialize_rocket() -> This is the general-purpose initialization method for
 *      all other initialization processes. This method should set up any required
 *      drivers, initialize desired variables, and declare all FreeRTOS tasks.
 *      Following the successful completion of this method, the FreeRTOS scheduler
 *      will start and control of program execution will not be yielded back to the 
 *      user except through tasks.
 * 
 * Note that if either initialization method return anything other then STATUS_OK,
 * code execution will immediately halt and no further actions will be taken.
*/
void run_rocket_os();


/** General purpose debug command.
 * 
 * The inputs to this method are identical to printf
 * 
 * The exact behavior of this method depends on the active rocket. 
 * By default, a weak implementation of this method is provided in rocketlib.c 
 * as a no-op. Each rocket should implement its own stronger version of this
 * method. The supporting method initialize_debug() will be run prior to any
 * calls to this method, and can be used to set up any neccessary peripherals
 * required to handle debug output.
 * 
 * @param format The format string
 * @param ... variable arguments for the format specification
 */
void debug(const char *restrict format, ...);

