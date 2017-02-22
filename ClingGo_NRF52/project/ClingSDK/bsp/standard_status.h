/*****************************************************************************
 * @file standard_status.h
 * @brief Common status codes for RHT projects
 *
 * @details This file defines standard status codes returned by functions.
 *          If a module and/or driver requires a status code that is not 
 *          represented here, it may define its own custom status code in the 
 *          following manner:
 *
 * @code 
 *          #define STATUSCODE_MODULE_CODE ((EN_STATUSCODE)(STATUSCODE_CUSTOM + x))
 * @endcode
 *
 *          Where:
 *              @li MODULE is the name of the module
 *              @li NAME is a descriptive name for the status
 *              @li x is a value from 0 to (255 - STATUSCODE_CUSTOM)
 *
 * @warning The values associated with these status codes are subject to change!
 *          Any code expecting status codes should compare against these defines,
 *          and <i>never</i> against literal constants.
 * 
 * Copyright &copy; 2011 RHT as unpublished work.  All rights reserved.
 *
 * @author  Greg Lindsey
 * @date 03/08/2011
 * @version 1.3
 *
 * @note Do not include this file directly; it is included by standards.h.
 ******************************************************************************/
#ifndef __STANDARD_STATUS_H__
#define __STANDARD_STATUS_H__
/** @addtogroup STANDARDS */
/** @{ */

typedef enum
{
    STATUSCODE_SUCCESS = 0x00,                     /**< Success */
    STATUSCODE_FAILURE,                            /**< Failure, no details given */
    STATUSCODE_IGNORE,                             /**< Ignore command */
    STATUSCODE_CONTINUE,                           /**< Message parsing continues */
    STATUSCODE_NOT_IMPLEMENTED,                    /**< Feature not implemented */
    STATUSCODE_ADDRESS_OUT_OF_RANGE,               /**< Invalid address for this function */
    STATUSCODE_VALUE_OUT_OF_RANGE,                 /**< Invalid value for this function */
    STATUSCODE_NOT_INITIALIZED,                    /**< Subsystem not initialized */
    STATUSCODE_NULL_POINTER,                       /**< Pointer provided is null */
    STATUSCODE_INVALID_CRC,                        /**< CRC check failed */
    STATUSCODE_INVALID_FUNCTION,                   /**< Function is not valid for this implementation */
    STATUSCODE_DEVICE_DISABLED,                    /**< Simplex Channel Disabled */
    STATUSCODE_OUT_OF_RESOURCES,                   /**< No more resources of requested type available */
    STATUSCODE_UNEXPECTED_INTERRUPT,               /**< Unexpected Interrupt */ 
    STATUSCODE_BUFFER_TOO_SMALL,                   /**< Buffer too small to accept data */
    STATUSCODE_CALLBACK_EXIT,			                 /**< The callback function requested an exit  */
    STATUSCODE_TIMEOUT,                            /**< The operation timed out */
    STATUSCODE_OVERFLOW,                           /**< Buffer overflow (for a ring buffer) */
	  
    STATUSCODE_CUSTOM = 0xE0
	
} EN_STATUSCODE;

/*
    Pointer to a function that returns an ENUM_STATUSCODE
*/
typedef EN_STATUSCODE (*INITFUNCPTR) (void);               /*  Pointer to STATUSCODE(void) function */

/** @} */
#endif /* __STANDARD_STATUS_H__ */

