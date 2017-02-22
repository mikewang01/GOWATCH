/***************************************************************************/
/**
 * File: notification.c
 * 
 * Description: notification processing
 *
 * Created on May 15, 2014
 *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>


#include "main.h"

//#define _NOTIFIC_TESTING_

#define NOTIFIC_ON_TIME_IN_MS 100
#define NOTIFIC_OFF_TIME_IN_MS 400
#define NOTIFIC_VIBRATION_REPEAT_TIME 3
#define NOTIFIC_VIBRATION_SHORT_TIME  2
#define NOTIFIC_VIBRATION_SINGLE_TIME 1
#define NOTIFIC_MULTI_REMINDER_LATENCY 2000

#define NOTIFIC_MULTI_REMINDER_TIME 15


static OS_TIMER notific_timer;

void NOTIFIC_stop_notifying()
{
	Y_SPRINTF("[NOTIFIC] Stop notifying");
	cling.notific.state = NOTIFIC_STATE_IDLE;
//	GPIO_vibrator_set(FALSE);
}

#define NOTIFIC_MULTI_REMINDER_INCOMING_CALL 3
#define NOTIFIC_MULTI_REMINDER_OTHERS        1
#define NOTIFIC_MULTI_REMINDER_IDLE_ALERT    3
#define NOTIFIC_MULTI_REMINDER_HR            2

void NOTIFIC_start_notifying(I8U cat_id)
{		
#ifndef _CLING_PC_SIMULATION_
#ifdef _ENABLE_ANCS_
	// Do not notify user if unit is not worn
	if (!TOUCH_is_skin_touched())
		return;
	
	// Do not notify user if unit is in sleep state
	if (SLEEP_is_sleep_state(SLP_STAT_SOUND) || SLEEP_is_sleep_state(SLP_STAT_LIGHT))
		return;

	// Reset vibration times
	cling.notific.vibrate_time = 0;
	// The maximum vibration time
	if (cat_id == BLE_ANCS_CATEGORY_ID_INCOMING_CALL)
		cling.notific.second_reminder_max = NOTIFIC_MULTI_REMINDER_INCOMING_CALL;
	else
		cling.notific.second_reminder_max = NOTIFIC_MULTI_REMINDER_OTHERS;
	cling.notific.cat_id = cat_id;
	cling.notific.first_reminder_max = NOTIFIC_VIBRATION_SHORT_TIME;
	cling.notific.state = NOTIFIC_STATE_SETUP_VIBRATION;
	Y_SPRINTF("[NOTIFIC] start notification: %d", cat_id);
	cling.ui.notif_type = NOTIFICATION_TYPE_MESSAGE;
	// Also, turn on screen
	UI_turn_on_display(UI_STATE_NOTIFICATIONS, 3000);
#endif	
#endif
}

void NOTIFIC_start_idle_alert()
{
	cling.notific.vibrate_time = 0;
	cling.notific.first_reminder_max = NOTIFIC_VIBRATION_SHORT_TIME;
	cling.notific.second_reminder_max = NOTIFIC_MULTI_REMINDER_IDLE_ALERT;
	cling.notific.state = NOTIFIC_STATE_SETUP_VIBRATION;
#ifdef _UI_ENABLED_
	cling.ui.notif_type = NOTIFICATION_TYPE_IDLE_ALERT;
	UI_turn_on_display(UI_STATE_NOTIFICATIONS, 3000);
#endif
	Y_SPRINTF("NOTIFIC - IDLE ALERT @ %d:%d", cling.time.local.hour, cling.time.local.minute);
}

void NOTIFIC_start_HR_alert()
{
#ifdef _HEART_RATE_ENABLED_
	cling.notific.vibrate_time = 0;
	cling.notific.first_reminder_max = NOTIFIC_VIBRATION_REPEAT_TIME;
	cling.notific.second_reminder_max = NOTIFIC_MULTI_REMINDER_HR;
	cling.notific.state = NOTIFIC_STATE_SETUP_VIBRATION;
	cling.ui.notif_type = NOTIFICATION_TYPE_HR;
	UI_turn_on_display(UI_STATE_NOTIFICATIONS, 3000);
	Y_SPRINTF("NOTIFIC - HEART RATE ALERT @ %d:%d", cling.time.local.hour, cling.time.local.minute);
#endif
}

extern void ALERT_notific_timer_callback(OS_TIMER tmr);
extern void SYSCLK_restart_timer(OS_TIMER timer, I32U period_in_ms);
extern OS_TIMER SYSCLK_get_timer(I32U period_in_ms, TimerCallbackFunction_t callback);

int NOTIFIC_state_machine()
{
	I32U t_curr   __attribute__((unused)) = CLK_get_system_time();
	BOOLEAN b_run_state_machine = TRUE;
	
	if (OTA_if_enabled()) {
		if (cling.notific.state != NOTIFIC_STATE_IDLE) {
			cling.notific.state = NOTIFIC_STATE_IDLE;
			GPIO_vibrator_set(FALSE);
		}
		return FALSE;
	} 
	
	switch (cling.notific.state) {
		case NOTIFIC_STATE_IDLE:
		{
		        b_run_state_machine = FALSE;
			break;
		}
		case NOTIFIC_STATE_SETUP_VIBRATION:
		{
			cling.notific.state = NOTIFIC_STATE_ON;
			cling.notific.second_reminder_time = 0;
			Y_SPRINTF("[NOTIFIC] second notif max: %d,", cling.notific.second_reminder_max);
			break;
		}
		case NOTIFIC_STATE_ON:
		{
			Y_SPRINTF("[NOTIFIC] vibrator is ON, %d", t_curr);
			GPIO_vibrator_set(TRUE);

			cling.notific.state = NOTIFIC_STATE_OFF;

                        if (!notific_timer) {
                                // Make sure alert timer module is created!
                                notific_timer = SYSCLK_get_timer(NOTIFIC_ON_TIME_IN_MS, ALERT_notific_timer_callback);
                                if (!notific_timer) {
                                        Y_SPRINTF("Reminder: Timer Started Failed ");
                                        cling.notific.state = NOTIFIC_STATE_IDLE;
                                        break;
                                }
                        } else {
                                SYSCLK_restart_timer(notific_timer, NOTIFIC_ON_TIME_IN_MS);
                        }
                        b_run_state_machine = FALSE;
			break;
		}
		case NOTIFIC_STATE_OFF:
		{
                        Y_SPRINTF("[NOTIFIC] vibrator is OFF, %d", t_curr);
                        GPIO_vibrator_set(FALSE);
                        cling.notific.state = NOTIFIC_STATE_REPEAT;
                        b_run_state_machine = FALSE;
                        SYSCLK_restart_timer(notific_timer, NOTIFIC_OFF_TIME_IN_MS);
                        cling.notific.vibrate_time ++;
			break;
		}
		case NOTIFIC_STATE_REPEAT:
		{
                        if (cling.notific.vibrate_time >= cling.notific.first_reminder_max) {
                                cling.notific.state = NOTIFIC_STATE_SECOND_REMINDER;
                                cling.notific.second_reminder_time ++;
                                b_run_state_machine = FALSE;
                                SYSCLK_restart_timer(notific_timer, NOTIFIC_MULTI_REMINDER_LATENCY);
                        } else {
                                cling.notific.state = NOTIFIC_STATE_ON;
                        }
                        Y_SPRINTF("[NOTIFIC] vibrator repeat, %d, %d", cling.notific.vibrate_time, t_curr);
			break;
		}
		case NOTIFIC_STATE_SECOND_REMINDER:
		{
                        if (cling.notific.second_reminder_time >= cling.notific.second_reminder_max) {
                                cling.notific.state = NOTIFIC_STATE_IDLE;
                                Y_SPRINTF("[NOTIFIC] Notify second reminder over, idle - %d", cling.notific.second_reminder_time);
                                GPIO_vibrator_set(FALSE);
                        } else {
                                cling.notific.state = NOTIFIC_STATE_ON;
                                // Reset vibration times
                                cling.notific.vibrate_time = 0;
                                Y_SPRINTF("[NOTIFIC] Notify second reminder - %d", cling.notific.second_reminder_time);
                        }
			break;
		}
		case NOTIFIC_STATE_STOPPED_EXTERNAL:
		{
			cling.notific.state = NOTIFIC_STATE_IDLE;
			GPIO_vibrator_set(FALSE);
			break;
		}
		default:
		{
			Y_SPRINTF("[NOTIFIC] Notify wrong state - %d", cling.notific.state);
			cling.notific.state = NOTIFIC_STATE_IDLE;
			GPIO_vibrator_set(FALSE);
			break;
		}
	}
	return b_run_state_machine;
	if (b_run_state_machine) {
                OS_TASK_NOTIFY(OS_GET_CURRENT_TASK(), OS_TASK_ALERT_NOTIFIC_BITS, OS_NOTIFY_SET_BITS);
	}
}
#ifdef _ENABLE_ANCS_
void NOTIFIC_smart_phone_notify(I8U* data)
{
#ifndef _CLING_PC_SIMULATION_	
	I8U mode = data[0];
	I8U id = data[1];
	I8U title_len, msg_len;
	
	title_len = data[2];
	msg_len = data[3];
	
	if ((title_len + msg_len) > 120)
		return;
	
	data[4+title_len+msg_len] = 0;
	
	if (mode == NOTIFIC_SMART_PHONE_NEW) {
		 if(cling.ancs.message_total >= 16) {
				
			 FLASH_erase_App(SYSTEM_NOTIFICATION_SPACE_START);
			 
			 // Latency before writing notification message (Erasure latency: 50 ms)
			 BASE_delay_msec(50);
		 
			 Y_SPRINTF("[ANCS] message is full, go erase the message space");

			 cling.ancs.message_total = 0;
		 }
		 
		cling.ancs.message_total++;		
		
		Y_SPRINTF("\n[NOTIFIC] *** SMART PHONE @ %d, %d, %d, %d, %s\n", mode, id, title_len, msg_len, data+4);
	 
		// title len, message len, ....
		ANCS_nflash_store_one_message(data+2);
		
		// Inform NOTIFIC state machine to notify user
		NOTIFIC_start_notifying(id);
		
	} else if (mode == NOTIFIC_SMART_PHONE_DELETE) {
		NOTIFIC_stop_notifying();
	} else if (mode == NOTIFIC_SMART_PHONE_STOP) {
		NOTIFIC_stop_notifying();
	}
#endif
}

I8U NOTIFIC_get_message_total(void)
{

#ifdef _NOTIFIC_TESTING_
	return 5;
#endif
	if (cling.ancs.message_total > 16)
		cling.ancs.message_total = 16;
	
	return cling.ancs.message_total;
}
#endif

I8U NOTIFIC_get_app_name(I8U index, char *app_name)
{
	I8U  title_len;
#ifdef _ENABLE_ANCS_		
	I32U addr_in;
	I32U tmpBuf[32];
	
	I8U  *pdata = (I8U *)tmpBuf;
	
	if (cling.ancs.message_total == 0) {
		title_len = sprintf(app_name, "No message!");
		return title_len;
	}
	
	
	if (index > (cling.ancs.message_total-1)) {
		title_len = sprintf(app_name, "No message!");
		return title_len;
	}
	
	// Get the latest notification first.
	index = cling.ancs.message_total - 1 - index;

	// Get message title
	addr_in=((index*256)+SYSTEM_NOTIFICATION_SPACE_START);

	FLASH_Read_App(addr_in, pdata, 128);	
	
	title_len =  pdata[0];
	
	if (title_len >= ANCS_SUPPORT_MAX_TITLE_LEN)
		title_len = ANCS_SUPPORT_MAX_TITLE_LEN;
	
	// Get the incoming message
	memcpy(app_name,pdata+2,title_len);	
	
	app_name[title_len] = 0;		

	Y_SPRINTF("[NOTIFIC] get ancs title len :%d",title_len );
	
	Y_SPRINTF("[NOTIFIC] get ancs title string :%s",app_name );
	
	return title_len;
#else
	
	title_len = sprintf(app_name, "No message!");
	return title_len;
#endif
}

I8U NOTIFIC_get_app_message_detail(I8U index, char *string)
{
	I8U  title_len;
#ifdef _ENABLE_ANCS_	
	I8U  mes_len;	
	I8U  mes_offset;
	I32U addr_in;
	I32U tmpBuf_1[32];
	I32U tmpBuf_2[32];
	I8U  *pdata_1 = (I8U *)tmpBuf_1;
  I8U  *pdata_2 = (I8U *)tmpBuf_2;

	if (cling.ancs.message_total == 0) {
		title_len = sprintf(string, "No message!");
		return title_len;
	}
	
#ifdef _NOTIFIC_TESTING_
	if (index == 0)
		mes_len = sprintf(string, "aaaa2388qwerjqfoiajfkqerfaoiasrqwerqwerasdfasfdjfiq23rjfdoieroaaaa");
	else if (index == 1)
		mes_len = sprintf(string, "bbbb2388qwerjqfoiajfkqerfaoiasrqwerqwerasdfasfdjfiq23rjfdoierbbbb");
	else if (index == 2)
		mes_len = sprintf(string, "cccc2388qwerjqfoiajfkqerfaoiasrqwerqwerasdfasfdjfiq23rjfdoiercccc");
	else if (index == 3)
		mes_len = sprintf(string, "dddd2388qwerjqfoiajfkqerfaoiasrqwerqwerasdfasfdjfiq23rjfdoierdddd");
	else if (index == 4)
		mes_len = sprintf(string, "eeee2388qwerjqfoiajfkqerfaoiasrqwerqwerasdfasfdjfiq23rjfdoiereeee");
	else 
		mes_len = sprintf(string, "n/a");
	
	return mes_len;
#endif
	
	if (index > (cling.ancs.message_total-1)) {
		title_len = sprintf(string, "No message!");
		return title_len;
	}
	
	// Get the latest notification first.
	index = cling.ancs.message_total - 1 - index;

	// First, read the first 128 bytes
	addr_in=((index*256)+SYSTEM_NOTIFICATION_SPACE_START);

	FLASH_Read_App(addr_in, pdata_1, 128);	

  title_len = pdata_1[0];
  mes_len   = pdata_1[1];	
	
	mes_offset = 1+1+title_len;
	
	//Currently only support 128 byte	
	if(mes_len >= 127)
	 mes_len=127;
	
	// if the overall length of message and title is less than 128 bytes
	if (mes_len <= (128-mes_offset)) {
		memcpy(string, pdata_1+mes_offset, mes_len);
		string[mes_len] = 0;
	} else {
		addr_in+=128;
		FLASH_Read_App(addr_in, pdata_2, 128);	
		
		// if the overall length of message and title is greater than 128 bytes
		memcpy(string, pdata_1+mes_offset, 128-mes_offset);
		memcpy(string+(128-mes_offset), pdata_2, mes_len-(128-mes_offset));
		string[mes_len] = 0;
	}
	
	return mes_len;
#else

	title_len = sprintf(string, "No message!");
	return title_len;
#endif
}

//if message = "Incoming Call" .......
I8U NOTIFIC_get_callerID(char *string)
{
	I8U  title_len = 0;
#ifdef _ENABLE_ANCS_		
	// ....
	I32U addr_in;
	I32U tmpBuf[32];
	
	I8U  *pdata = (I8U *)tmpBuf;

	addr_in=((cling.ancs.message_total-1)*256+SYSTEM_NOTIFICATION_SPACE_START);
	
	FLASH_Read_App(addr_in, pdata, 128);	

	title_len= pdata[0];
	
	if (title_len >= ANCS_SUPPORT_MAX_TITLE_LEN)
		title_len = ANCS_SUPPORT_MAX_TITLE_LEN;

	memcpy(string, pdata+2, title_len);		
	string[title_len] = 0;

	Y_SPRINTF("[NOTIFIC] get notf callerID string is :%s",string );
	
	return title_len;	
#else
	
	title_len = sprintf(string, "No CallerID!");
	return title_len;
#endif	
}

BOOLEAN NOTIFIC_is_user_viewing_message()
{
#ifdef _UI_ENABLED_
	// 1. see if device is receiving a notification message
	if (cling.ui.state == UI_STATE_NOTIFICATIONS)
		return TRUE;
	
	// 1. see if device is in a active display state
	if (UI_is_idle()) {
		return FALSE;
	}
	// 3. see if UI frame is displaying notification message
	if ((cling.ui.frame_index >= UI_DISPLAY_SMART) && (cling.ui.frame_index <= UI_DISPLAY_SMART_END))
	{
		return TRUE;
	}
#endif
	return FALSE;
}
