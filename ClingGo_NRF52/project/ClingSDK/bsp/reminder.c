/***************************************************************************/
/**
 * File: reminder.c
 * 
 * Description: user reminder processing
 *
 * Created on May 15, 2014
 *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>


#include "main.h"

#define REMINDER_ON_TIME_IN_MS 200
#define REMINDER_OFF_TIME_IN_MS 400

#define REMINDER_SECOND_REMINDER_LATENCY 2000

#define REMINDER_VIBRATION_REPEAT_TIME 2
#define SECOND_REMINDER_TIME 8

static OS_TIMER reminder_timer;

void REMINDER_setup(I8U *msg, BOOLEAN b_daily)
{
	I32U data[32];
	I8U *pdata = (I8U*)data;
	I8U *pmsg;
	I8U i, a, b, c, day;
	BOOLEAN b_init_alarm = FALSE;
	
	// Get current day
	day = 1 << cling.time.local.dow;

	FLASH_erase_App(SYSTEM_REMINDER_SPACE_START);
	
	BASE_delay_msec(50); // Latency before refreshing reminder space. (Erasure latency: 50 ms)
	if (b_daily) {
		// 3 bytes per one alarm clock
		cling.reminder.total = (msg[0] / 3);
	} else {
		// 2 bytes per one alarm clock
		cling.reminder.total = (msg[0] >> 1);
	}
	if (cling.reminder.total > 32)
		cling.reminder.total = 32;

	cling.reminder.ui_hh = 0xff;
	cling.reminder.ui_mm = 0xff;
	
	pmsg = msg + 1;
	if (cling.reminder.total > 0) {
		
		memset(pdata, 0, 128);
		
		// Copy all the alarm clock, and set it to everyday
		for (i = 0; i < cling.reminder.total; i++) {
			if (b_daily) 
				c = *pmsg++;
			else
				c = 0x7f;
			//*pdata++ = *pmsg++;
			//*pdata++ = *pmsg++;
			a = *pmsg++;
			b = *pmsg++;
			*pdata++ = c;
			*pdata++ = a;
			*pdata++ = b;

			if (c & day) {
				if (!b_init_alarm) {
					b_init_alarm = TRUE;
					cling.reminder.ui_hh = a;
					cling.reminder.ui_mm = b;
				}
			}
			N_SPRINTF("[CP] reminder setup(%d, %d), %d:%d(%02x)", i, cling.reminder.total, a, b, c);
		}
		pdata = (I8U *)data;
		FLASH_Write_App(SYSTEM_REMINDER_SPACE_START, pdata, 128); // Maximum 32 entries
	}
	cling.reminder.state = REMINDER_STATE_CHECK_NEXT_REMINDER;
}

void REMINDER_alarm_init()
{
        REMINDER_set_next_reminder();
}

void REMINDER_set_next_reminder()
{
	I8U i, hour, minute, week, day;
	BOOLEAN b_found = FALSE;
	BOOLEAN b_first_alarm = FALSE;
	I32U buffer[32];
	I8U *data = (I8U *)buffer;
	
	cling.reminder.ui_hh = 0xff;
	cling.reminder.ui_mm = 0xff;
	
	if (!LINK_is_authorized()) {
		cling.reminder.b_valid = FALSE;
		cling.reminder.hour = 0;
		cling.reminder.minute = 0;
		N_SPRINTF("[REMINDER] NOT authorized, set invalid hh/mm");
		return;
	}

	if (cling.user_data.b_reminder_off_weekends) 
	{
		// Check if today is Saturday or Sunday
		if (cling.time.local.dow > 4) {
			cling.reminder.b_valid = FALSE;
			cling.reminder.hour = 0;
			cling.reminder.minute = 0;
			N_SPRINTF("[REMINDER] Weekend OFF mode, set invalid hh/mm");
			return;
		}
	}
	
	if (cling.reminder.total) {
		FLASH_Read_App(SYSTEM_REMINDER_SPACE_START, (I8U *)buffer, 128);
		// Get current day
		day = 1 << cling.time.local.dow;
		N_SPRINTF("[REMINDER] today is: 0x%02x, %d", day, cling.time.local.dow);

		for (i = 0; i < cling.reminder.total; i++) {
			week = *data++;
			hour = *data++;
			minute = *data++;
			
			N_SPRINTF("[REMINDER] current time: %d:%d, reminder: %d:%d, %02x",
				cling.time.local.hour, cling.time.local.minute, hour, minute, week);

			if (hour >= 24) break;
			if (minute >= 60) break;
			
			if (week & day) {
				
				if (!b_first_alarm) {
					b_first_alarm = TRUE;
					cling.reminder.ui_hh = hour;
					cling.reminder.ui_mm = minute;
					N_SPRINTF("[REMINDER] first reminder is set: %d:%d", hour, minute);

				}
				if (hour == cling.time.local.hour) {
					if (minute > cling.time.local.minute) {
						b_found = TRUE;				
						break;
					}
				} else if (hour > cling.time.local.hour) {
					b_found = TRUE;
					break;
				}
			}
		}
	}

	if (b_found) {
		cling.reminder.b_valid = TRUE;
		cling.reminder.hour = hour;
		cling.reminder.minute = minute;
		
		N_SPRINTF("[REMINDER] found reminder: %d:%d", cling.reminder.hour, cling.reminder.minute);
	} else {
		cling.reminder.b_valid = FALSE;
		cling.reminder.hour = 0;
		cling.reminder.minute = 0;

		N_SPRINTF("[REMINDER] No new reminder is found");
	}
	
	if (!b_first_alarm) {
		N_SPRINTF("[REMINDER] NOT reminder found for today, set invalid hh/mm");
  }

	N_SPRINTF("[REMINDER] Set next reminder - done");
}

BOOLEAN REMINDER_check_reminder()
{
	if (!cling.reminder.b_valid)
		return FALSE;
	
	if ((cling.time.local.hour >= cling.reminder.hour) && (cling.time.local.minute >= cling.reminder.minute))
	{
		return TRUE;
	} else {
		return FALSE;
	}
}

void REMINDER_fire_up_alert()
{
        cling.reminder.state = REMINDER_STATE_ON;
        // Reset vibration times
        cling.reminder.vibrate_time = 0;
        cling.reminder.second_vibrate_time = 0;
        cling.reminder.ui_alarm_on = TRUE;
        cling.ui.notif_type = NOTIFICATION_TYPE_REMINDER;
//                              UI_turn_on_display(UI_STATE_NOTIFICATIONS, 1000);
}


extern void ALERT_reminder_timer_callback(OS_TIMER tmr);
extern void SYSCLK_restart_timer(OS_TIMER timer, I32U period_in_ms);
extern OS_TIMER SYSCLK_get_timer(I32U period_in_ms, TimerCallbackFunction_t callback);

int REMINDER_state_machine()
{
        I32U t_curr   __attribute__((unused)) = CLK_get_system_time();
        BOOLEAN b_notify_task = FALSE;

	if (OTA_if_enabled())
		return FALSE;

	switch (cling.reminder.state) {
		case REMINDER_STATE_IDLE:
		{
                        if (cling.notific.state == NOTIFIC_STATE_IDLE) {

					GPIO_vibrator_set(FALSE);
                        }
			break;
		}
		case REMINDER_STATE_ON:
		{
			N_SPRINTF("[REMINDER] vibrator is ON, %d", t_curr);
			GPIO_vibrator_set(TRUE);
			cling.reminder.state = REMINDER_STATE_OFF;

			if (!reminder_timer) {
			        // Make sure alert timer module is created!
			        reminder_timer = SYSCLK_get_timer(REMINDER_ON_TIME_IN_MS, ALERT_reminder_timer_callback);
	                        if(!reminder_timer){
	                                N_SPRINTF("Reminder: Timer Started Failed ");
	                                cling.reminder.state = REMINDER_STATE_IDLE;
	                                break;
	                        }
			} else {
			        SYSCLK_restart_timer(reminder_timer, REMINDER_ON_TIME_IN_MS);
			}
			break;
		}
		case REMINDER_STATE_OFF:
		{
                        N_SPRINTF("[REMINDER] vibrator is OFF, %d", t_curr);
                        GPIO_vibrator_set(FALSE);
                        cling.reminder.state = REMINDER_STATE_REPEAT;
                        cling.reminder.vibrate_time ++;
			SYSCLK_restart_timer(reminder_timer, REMINDER_OFF_TIME_IN_MS);
			break;
		}
		case REMINDER_STATE_REPEAT:
		{
                        if (cling.reminder.vibrate_time >= REMINDER_VIBRATION_REPEAT_TIME) {
                                cling.reminder.state = REMINDER_STATE_SECOND_REMINDER;
                                cling.reminder.second_vibrate_time ++;
                                N_SPRINTF("[REMINDER] go second reminder, %d, %d", cling.reminder.second_vibrate_time, cling.reminder.vibrate_time);
                                SYSCLK_restart_timer(reminder_timer, REMINDER_SECOND_REMINDER_LATENCY);
                        } else {
                                cling.reminder.state = REMINDER_STATE_ON;
                                N_SPRINTF("[REMINDER] vibrator repeat, %d, %d", cling.reminder.vibrate_time, t_curr);
                                b_notify_task = TRUE;
                        }
			break;
		}
		case REMINDER_STATE_SECOND_REMINDER:
		{
                        if (cling.reminder.second_vibrate_time >= SECOND_REMINDER_TIME) {
                                cling.reminder.state = REMINDER_STATE_CHECK_NEXT_REMINDER;
                                N_SPRINTF("[REMINDER] Go check next: %d", cling.reminder.second_vibrate_time);
                        } else {
                                N_SPRINTF("[REMINDER] second reminder on");
                                cling.reminder.state = REMINDER_STATE_ON;
                                // Reset vibration times
                                cling.reminder.vibrate_time = 0;
                        }
                        b_notify_task = TRUE;
			break;
		}
		case REMINDER_STATE_CHECK_NEXT_REMINDER:
                default:
		{
                        GPIO_vibrator_set(FALSE);
			REMINDER_set_next_reminder();
			cling.reminder.state = REMINDER_STATE_IDLE;
			N_SPRINTF("[REMINDER] STATE: CHECK NEXT");
                        b_notify_task = TRUE;
			break;
		}
	}
	return b_notify_task;
	if (b_notify_task) {
                OS_TASK_NOTIFY(OS_GET_CURRENT_TASK(), OS_TASK_ALERT_REMINDER_BITS, OS_NOTIFY_SET_BITS);
	}
}

I8U REMINDER_get_time_at_index(I8U index)
{
	I8U new_idx, hh, mm, i, week, day, total;
	I32U i32_buf[32];
	I8U *data;
	I8U alarm[64];
	I8U *palarm;
		
	// In case that no reminder is configure, go return
	cling.reminder.ui_hh = 0xff;
	cling.reminder.ui_mm = 0xff;
	if (cling.reminder.total == 0) {
		N_SPRINTF("[REMINDER] no reminder due to ZERO total");
		return 0;
	}
	
	// First we get Alarms for the day
	FLASH_Read_App(SYSTEM_REMINDER_SPACE_START, (I8U *)i32_buf, 128);
	
	total = 0;
	data = (I8U *)i32_buf;
	palarm = alarm;

	// Get correct day for alarm checkup
	day = 1 << cling.time.local.dow;

	for (i = 0; i < cling.reminder.total; i++) {
		week = *data++;
		hh = *data++;
		mm = *data++;
		if (week & day) {
			*palarm++ = hh;
			*palarm++ = mm;
			total++;
		}
	}
	
	if (total == 0) {
		N_SPRINTF("[REMINDER] No reminder found for today, set invalid hh/mm");
		return 0;
	}

	palarm = alarm;
	new_idx = index;
	N_SPRINTF("[REMINDER] maximum entry: %d, %d", total, new_idx);

	if (new_idx >= total) {
		N_SPRINTF("[REMINDER] reset entry: %d, %d", total, new_idx);
		new_idx = 0;
	}
	hh = alarm[new_idx<<1];
	mm = alarm[(new_idx<<1)+1];

	if (hh>=24 || mm>=60) {
		N_SPRINTF("[REMINDER] invalid alarm set up from flash, set invalid hh/mm");
		return 0;
	}

	cling.reminder.ui_hh = hh;
	cling.reminder.ui_mm = mm;
	
	N_SPRINTF("[REMINDER] ui display: %d:%d", hh, mm);
		
	return new_idx;
}

