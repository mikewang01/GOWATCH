/***************************************************************************//**
 * @file Watchdog.h
 * @brief Driver Interface for an on-chip watchdog
 *
 ******************************************************************************/

#ifndef _WATCHDOG_H_
#define _WATCHDOG_H_

#include "standards.h"

/***************************************************************************//**
 * @brief Initializes watchdog interface
 * @details This function initializes the on-chip watchdog.  It does NOT
 *          enable the watchdog; this is done with Watchdog_Enable.
 *
 * @return Returns a status code.
 * @retval STATUSCODE_SUCCESS Initialization succeeded
 * @retval STATUSCODE_FAILURE Initialization failed
 ******************************************************************************/
extern EN_STATUSCODE Watchdog_Init(void);

/***************************************************************************//**
 * @brief Feeds the watchdog timer
 * @details This function should be called to feed the watchdog when needed.
 *
 ******************************************************************************/
extern void Watchdog_Feed(void);

/***************************************************************************//**
 * @brief Enables the watchdog timer
 * @details This function enables the watchdog timer.
 *
 * @return Returns a status code.
 * @retval STATUSCODE_SUCCESS Watchdog enabled.
 * @retval STATUSCODE_FAILURE Watchdog could not be enabled.
 ******************************************************************************/
extern EN_STATUSCODE Watchdog_Enable(void);

/***************************************************************************//**
 * @brief Disables the watchdog timer.
 * @details This function disables resetting of the CPU via the watchdog.
 *          Depending on the architecture it may suspend the watchdog timer,
 *          or it may just stop the watchdog from tripping a reset condition.
 *
 * @return Returns a status code.
 * @retval STATUSCODE__SUCCESS Watchdog diabled.
 * @retval STATUSCODE__FAILURE Watchdog could not be disabled.
 ******************************************************************************/
extern EN_STATUSCODE Watchdog_Disable(void);

#endif  /* _WATCHDOG_H_ */
/** @} */
