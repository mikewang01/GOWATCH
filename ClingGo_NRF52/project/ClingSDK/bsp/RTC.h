/***************************************************************************//**
 * @file RTC.h
 * @brief Driver Interface for a real-time clock.
 *
 ******************************************************************************/

#ifndef _RTC_H_
#define _RTC_H_

#include "standards.h"
#include "btle_api.h"
#define MAX_RTC_CNT             0x00FFFFFF                                  /**< Maximum value of the RTC counter. */


#define EPOCH_HOUR_SECOND 3600 //seconds 
#define EPOCH_DAY_SECOND 86400 //seconds 
#define EPOCH_WEEK_SECOND 604800 //seconds 
#define EPOCH_MONTH_SECOND 2629743 //  (30.44 days)  2629743 seconds 
#define EPOCH_YEAR_SECOND 31556926 // (365.24 days)   31556926 seconds 

#define TIMEZONE_DIFF_UNIT_IN_SECONDS 900 // Time zone differenece in the unit of 15 minutes, equivalent of 900 seconds

#define DOW_MON 0
#define DOW_TUE 1
#define DOW_WED 2
#define DOW_THU 3
#define DOW_FRI 4
#define DOW_SAT 5
#define DOW_SUN 6

typedef struct tagSYSTIME_CTX
{
  I16U year;
  I8U month;
  I8U day;
  I8U dow;
  I8U hour;
  I8U minute;
  I8U second;
} SYSTIME_CTX;

/***************************************************************************//**
 * @brief Initializes RTC Interface
 * @details This function initializes the real-time clock interface.
 *
 * @note
 * Before this interface can be used, this function must be called
 *
 * @return Returns a Status Code
 * @retval STATUSCODE__SUCCESS Function succeeded
 * @retval STATUSCODE__FAILURE Function failed
 ******************************************************************************/
EN_STATUSCODE RTC_Init(void);
void RTC_timer_handler(void);

EN_STATUSCODE RTC_Start( void );
void RTC_start_operation_clk(void);
void RTC_stop_operation_clk( void );
void RTC_system_shutdown_timer(void);

// Get desired clock from UTC time
void RTC_get_local_clock(I32U epoch_start, SYSTIME_CTX *local);
void RTC_get_delta_clock_forward(SYSTIME_CTX *delta, I8U offset);
void RTC_get_delta_clock_backward(SYSTIME_CTX *delta, I8U offset);

void RTC_get_regular_time(I32U epoch, SYSTIME_CTX *t);
I32U RTC_get_epoch_day_start(I32U past_days);
BOOLEAN RTC_timer_process(void);

#endif /* _RTC_H_ */

/** @} */
