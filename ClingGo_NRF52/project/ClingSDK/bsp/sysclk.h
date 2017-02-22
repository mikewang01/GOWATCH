/***************************************************************************//**
 * @file SystemClock.h
 * @brief Header for implementing a system clock.
 * 
 *****************************************************************************/

#include "standards.h"

#ifndef __SYSTEMCLOCK_H__
#define __SYSTEMCLOCK_H__

#define SYSTEMCLOCK_TICKS_PER_SECOND  1024
#define SYSTEMCLOCK_TA0CCR0 0xFFFF // It should be 0xF...F format
#define SYSTEMCLOCK_BITS_SHIFT 16
#define MC__CLEAR 0xFFCF

#define SYSCLK_INTERVAL_10MS         APP_TIMER_TICKS(10, APP_TIMER_PRESCALER) /**< SYSTEM clock interval (ticks): 327 / 32.768K = 10 ms. */
#define SYSCLK_INTERVAL_20MS    	    APP_TIMER_TICKS(20, APP_TIMER_PRESCALER) /**< SYSTEM clock interval (ticks): 327 / 32.768K = 10 ms. */
#define SYSCLK_INTERVAL_100MS    	    APP_TIMER_TICKS(100, APP_TIMER_PRESCALER) /**< SYSTEM clock interval (ticks): 327 / 32.768K = 10 ms. */
#define SYSCLK_INTERVAL_1000MS				APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define SYSCLK_INTERVAL_2000MS				APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)
#define SYSCLK_INTERVAL_4000MS				APP_TIMER_TICKS(4000, APP_TIMER_PRESCALER)
#define SYSCLK_INTERVAL_6000MS				APP_TIMER_TICKS(6000, APP_TIMER_PRESCALER)
#define SYSCLK_INTERVAL_10000MS				APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER)

#define OPERATION_CLK_EXPIRATION          2000  // 20 ms system clock expires in 5 seconds

I32U SYSCLK_GetFineTime(void);
OS_TIMER SYSCLK_get_timer(I32U period_in_ms, TimerCallbackFunction_t callback);
extern void SYSCLK_restart_timer(OS_TIMER timer, I32U period_in_ms);
#endif /* __SYSTEMCLOCK_H__ */

/** @} */
