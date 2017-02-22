/***************************************************************************//**
 * @file RTC_MSP430.c
 * @brief Driver implementation for the Dialog real-time clock.
 *
 *
 ******************************************************************************/

#include "standards.h"
#include "main.h"
#include "rtc_hal.h"
#include "btle_api.h"
static  CLASS(HalRtc)*p_rtc_instance = NULL;
const I8U DAYS_IN_EACH_MONTH[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

extern void CLING_task_rtc_notify();

void RTC_timer_handler(void)
{
        static int second_count = 0;
        second_count++;
        if(second_count%5 == 0){
                CLING_task_rtc_notify();
                second_count = 0;
        }
}

BOOLEAN RTC_timer_process(void)
{
        I32U tick_now;
        I32U tick_diff;
        I32U tick_in_s;

        if (cling.time.tick_count) {
                tick_now = OS_TICKS_2_MS(OS_GET_TICK_COUNT());
                if (tick_now < cling.time.tick_count)
                        cling.time.tick_count = 0;
                tick_diff = tick_now - cling.time.tick_count;
                tick_in_s = tick_diff / 1000;
                tick_diff = tick_in_s * 1000;
                cling.time.tick_count += tick_diff;
        }
        else {
                cling.time.tick_count = OS_TICKS_2_MS(OS_GET_TICK_COUNT());
                tick_in_s = cling.time.tick_count / 1000;
        }

        // update battery measuring timer
        cling.batt.level_update_timebase += tick_in_s;
        BATT_update_charging_time(tick_in_s);

        // update radio duty cycling
        cling.time.system_clock_in_sec += tick_in_s;
        cling.time.time_since_1970 += tick_in_s;
        if (cling.batt.state_switching_duration < 128)
                cling.batt.state_switching_duration += tick_in_s;
        if (cling.batt.shut_down_time < BATTERY_SYSTEM_UNAUTH_POWER_DOWN) {
                cling.batt.shut_down_time += tick_in_s;
        }

        // Indicates a second-based RTC interrupt
        RTC_get_local_clock(cling.time.time_since_1970, &cling.time.local);

        // see if a second is hit
#ifdef _ENABLE_UART_
        if (tick_in_s) {
                N_SPRINTF("[RTC] second is hit, %d, %d, %d", tick_in_s, cling.time.time_since_1970, cling.time.local.second);
        }
#endif
        // Check if we have minute passed by, or
        if (cling.time.local.minute != cling.time.local_minute) {
                cling.time.local_minute_updated = TRUE;
                cling.time.local_minute = cling.time.local.minute;
                if (
                (cling.user_data.idle_time_in_minutes > 0) &&
                        (cling.time.local.hour >= cling.user_data.idle_time_start) &&
                        (cling.time.local.hour < cling.user_data.idle_time_end)
                        )
                        {
                        cling.user_data.idle_minutes_countdown--;
                }
                Y_SPRINTF("[RTC] min updated (%d)", cling.activity.day.walking);
        }

        if (cling.time.local.day != cling.time.local_day) {
                cling.time.local_day_updated = TRUE;
                cling.time.local_day = cling.time.local.day;

                // Reset reminder
                cling.reminder.state = REMINDER_STATE_CHECK_NEXT_REMINDER;

                Y_SPRINTF("[RTC] local day updated");
        }

        // Testing, assuming user sleeps around 22:00 at night
        if (cling.time.local_hour != cling.time.local.hour) {
                if (cling.time.local.hour == 12) {
                        cling.time.local_noon_updated = TRUE;
                }
                cling.time.local_hour = cling.time.local.hour;
                cling.time.local_hour_updated = TRUE;
        }

        // Event for minute-based tasks, such as user idle alert, data logging, reminder, etc.,
        if (cling.time.local_minute_updated || cling.time.local_day_updated || cling.time.local_noon_updated)
        {
                return TRUE;
        } else {
                return FALSE;
        }
}

// RTC is set to Calendar mode
EN_STATUSCODE RTC_Init()
{
        p_rtc_instance = HalRtc_get_instance();
        if (p_rtc_instance == NULL) {
                return STATUSCODE_FAILURE;
        }
        if(p_rtc_instance->isr_reg(p_rtc_instance, RTC_timer_handler) != 0){
                return STATUSCODE_FAILURE;
        }

        // Start RTC timer
        return RTC_Start();
}

EN_STATUSCODE RTC_Start(void)
{
        p_rtc_instance->start(p_rtc_instance);
        return STATUSCODE_SUCCESS;
}

//
// We might not need RTC stop as RTC runs all the time
//
void RTC_stop_operation_clk(void)
{
}

void RTC_start_operation_clk()
{
}

void RTC_system_shutdown_timer()
{
}

void RTC_get_delta_clock_forward(SYSTIME_CTX *delta, I8U offset)
{
        I32U epoch = cling.time.time_since_1970 + offset * EPOCH_DAY_SECOND;
        I16S time_diff_in_minute = cling.time.time_zone;
        time_diff_in_minute *= TIMEZONE_DIFF_UNIT_IN_SECONDS;
        epoch += time_diff_in_minute;

        RTC_get_regular_time(epoch, delta);
}

void RTC_get_delta_clock_backward(SYSTIME_CTX *delta, I8U offset)
{
        I32U epoch = cling.time.time_since_1970 - offset * EPOCH_DAY_SECOND;
        I16S time_diff_in_minute = cling.time.time_zone;
        time_diff_in_minute *= TIMEZONE_DIFF_UNIT_IN_SECONDS;
        epoch += time_diff_in_minute;

        RTC_get_regular_time(epoch, delta);
}

void RTC_get_local_clock(I32U epoch_start, SYSTIME_CTX *local)
{
        I32U epoch;
        I32S time_diff_in_minute = cling.time.time_zone;

        time_diff_in_minute *= TIMEZONE_DIFF_UNIT_IN_SECONDS;
        epoch = epoch_start + time_diff_in_minute;

        RTC_get_regular_time(epoch, local);

        N_SPRINTF("RTC: get local: %d, %d", epoch_start, local->second);
}

I8U const month_leap_in_days[12] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
I8U const month_normal_in_days[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

void RTC_get_regular_time(I32U epoch, SYSTIME_CTX *t)
{
        I8U i;
        I32U s;
        I32U tick = epoch;
        I32U dayno = tick / EPOCH_DAY_SECOND;
        I32U dayclock = tick - dayno * EPOCH_DAY_SECOND;

        t->year = 1970;
        t->month = 1;
        t->day = 1;
        t->hour = dayclock / 3600;
        t->minute = (dayclock - t->hour * 3600) / 60;
        t->second = dayclock % 60;
        t->dow = (dayno + 3) % 7;

        // Give a life span of 200 years
        s = 31556926;
        for (i = 0; i < 200; i++) {
                // First figure out the year
                if ((t->year & 0x03) == 0) {
                        s = 366;
                }
                else {
                        s = 365;
                }
                if (dayno < s) {
                        break;
                }
                t->year++;
                dayno -= s;
        }

        // get the month
        if ((t->year & 0x03) == 0) {
                for (i = 0; i < 12; i++) {
                        if (dayno < month_leap_in_days[i]) {
                                break;
                        }
                        t->month++;
                        dayno -= month_leap_in_days[i];
                }
        }
        else {
                for (i = 0; i < 12; i++) {
                        if (dayno < month_normal_in_days[i]) {
                                break;
                        }
                        t->month++;
                        dayno -= month_normal_in_days[i];
                }
        }

        // get the day
        t->day += dayno;

}

I32U RTC_get_epoch_day_start(I32U past_days)
{
        I32U current_epoch = cling.time.time_since_1970;
        I32U offset = cling.time.local.second;

        offset += cling.time.local.minute * 60;
        offset += cling.time.local.hour * 3600;

        offset += past_days * EPOCH_DAY_SECOND;

        current_epoch -= offset;

        return current_epoch;
}
/* @} */
