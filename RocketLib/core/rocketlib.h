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


/** The very first method run by the rocket OS
 * 
 * Immediately following this call (assuming it is successful), it will be assumed 
 * that any calls to debug(const char *restrict format, ...) are valid. 
 * As such, this function should do the minimum amount of work required to initialize
 * the debug output. Additional initialization steps should go in the initialize_rocket()
 * method.
 * 
 * Note that it is also guaranteed that this method will be called prior to the execution 
 * of any user code.
 * 
 * @return Returns STATUS_OK if the debug interface has been initialized, and any other 
 *         status otherwise. Note that returning anything other than STATUS_OK will cause 
 *         the rocket to halt.
 */
status_t initialize_debug();


/** General purpose rocket initialization function
 * 
 * This method will be called immediately after initialize_debug(), and should be used
 * for all other initialization tasks, including the process of setting up user drivers
 * and initialization FreeRTOS tasks. Following the execution of this method, assuming
 * it returns succesfully, the FreeRTOS scheduler will start and control of program 
 * execution will not be yielded back to the user except through tasks scheduled in this 
 * method.
 * 
 * @return Returns STATUS_OK if initialization was successful, and any other status 
 *         otherwise. Note that returning anything other than STATUS_OK will cause the 
 *         rocket to halt (and no tasks will be scheduled).
 */
status_t initialize_rocket();


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

