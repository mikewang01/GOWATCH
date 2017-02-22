/***************************************************************************//**
 * File: sysclk.c
 *
 * Description: System clock manager
 *
 *
 * Created on Feb 08, 2014
 *
 * 
 *****************************************************************************/

#include "standards.h"
#include "main.h"

#include "osal.h"
#include "rtc_hal.h"
#include "btle_api.h"


OS_TIMER SYSCLK_get_timer(I32U period_in_ms, TimerCallbackFunction_t callback)
{
        OS_TIMER timer=NULL;
        I32U status;
        if(timer == NULL){
          timer = OS_TIMER_CREATE("sysclk_timer",
                OS_MS_2_TICKS(period_in_ms), pdTRUE, (void*)0, callback);
        }
        status = OS_TIMER_START(timer, 0);
        if(status != OS_TIMER_SUCCESS){
                Y_SPRINTF("[SYSCLK_TIMER] SYSCLK Timer Started Failed ");
                OS_TIMER_DELETE(timer,0);
                timer = NULL;
        }
        return timer;
}

void SYSCLK_restart_timer(OS_TIMER timer, I32U period_in_ms)
{
        OS_TIMER_CHANGE_PERIOD(timer, OS_MS_2_TICKS(period_in_ms), 0);
        OS_TIMER_RESET(timer, 0);
}

/* Uses the 16-bit TimerA0 counter and the lower 16 bits of the system
 * clock to give a "fine-grained" time. */
I32U SYSCLK_GetFineTime(void)
{
	double tick_diff;
	I32U tick_now;
	double system_clock_in_ms;

	// Get the elapsed time
	tick_now =  OS_TICKS_2_MS(OS_GET_TICK_COUNT());
	tick_diff = tick_now - cling.time.tick_count;
	system_clock_in_ms = cling.time.system_clock_in_sec*1000;
	system_clock_in_ms += tick_diff;

	N_SPRINTF("[SYSCLK]: sec: %d, n: %d, o:%d", cling.time.system_clock_in_sec, tick_now, cling.time.tick_count);
	// 32758 ticks per second, about 32 tick per ms.
	return (I32U)system_clock_in_ms;
}



/** @} */
