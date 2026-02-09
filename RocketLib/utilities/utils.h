#pragma once
/**
 * @file utils.h
 * @brief General-purpose utilities that are platform and rocket agnostic
*/

/** General purpose function status */
typedef enum Status {
    
    STATUS_OK = 0,      //!< Function completed successfully
    STATUS_FAILED = 1,  //!< Function failed/terminated early
} status_t;

/** No-op that disables compiler warnings regarding parameter x being unused
 * 
 * This macro can be used as a placeholder to indicate that function parameters 
 * are intentionally unused.
 */
#define UNUSED(x) ((void)(x))

