/***************************************************************************/
/**
 * File: user.c
 * 
 * Description: USER related data processing functions
 *
 * Created on Feb 21, 2014
 *
 ******************************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "pedo.h"

static BOOLEAN _is_idle_alert_allowed()
{	
	if (OTA_if_enabled()) {
		return FALSE;
	}
#if 0
  if (cling.sleep.state==SLP_STAT_LIGHT || cling.sleep.state==SLP_STAT_SOUND) {
		return FALSE;
  }
#endif
	return TRUE;
}

void USER_data_init()
{
	USER_DATA *u = &cling.user_data;

	memset(&u->profile, 0, sizeof(USER_PROFILE_CTX));

	// Initialize stride length and weight for distance calculation
	u->profile.stride_in_cm = 75; // In center meters
	u->profile.weight_in_kg = 70; // in KG
	u->profile.height_in_cm = 170; // in center meters

	// default distance
	u->profile.metric_distance = FALSE;

	// user dynamic data initialization
	u->calories_factor = 36; // Calorie factor normalized by 1000

	// User pedometer state filtering thresholds
	u->ppg_day_interval = 900000; // 15 minutes
	u->ppg_night_interval = 1800000; // 30 minutes
	u->skin_temp_day_interval = 600000; // 10 minutes
	u->skin_temp_night_interval = 600000; // 10 minutes
#ifdef _CLING_PC_SIMULATION_
	// gesture recognition
	u->b_screen_wrist_flip = FALSE;
	u->b_screen_press_hold_1 = FALSE;
	u->b_screen_press_hold_3 = FALSE;
	u->b_screen_tapping = FALSE;

	u->b_navigation_tapping = FALSE;
	u->b_navigation_wrist_shake = FALSE;
#else
	// gesture recognition
	u->b_screen_wrist_flip = FALSE;
	u->b_screen_press_hold_1 = TRUE;
	u->b_screen_press_hold_3 = FALSE;
	u->b_screen_tapping = FALSE;

	u->b_navigation_tapping = FALSE;
	u->b_navigation_wrist_shake = FALSE;
#endif
	// Reset alert time
	u->idle_time_in_minutes = 0;
	u->idle_time_start = 0;
	u->idle_time_end = 0;
	u->idle_state = IDLE_ALERT_STATE_IDLE;
	
	// Screen ON time
	u->screen_on_general = 5;
	u->screen_on_heart_rate = 25;
	
	// Sleep detection sensitivity mode
//cling.sleep.m_sensitive_mode = SLEEP_SENSITIVE_HIGH;
//cling.sleep.m_sensitive_mode = SLEEP_SENSITIVE_LOW;
	
	// Reminder off during weekends
	u->b_reminder_off_weekends = FALSE;
}

void USER_store_device_param(I8U *data)
{
	I8U *pdata = data+1; // Leave the first byte for the length
	I8U setting_length = 0;
	USER_DATA *u = &cling.user_data;

	*pdata++ = (u->ppg_day_interval>>24)&0xff; // day interval
	*pdata++ = (u->ppg_day_interval>>16)&0xff; // day interval
	*pdata++ = (u->ppg_day_interval>>8)&0xff; // day interval
	*pdata++ = (u->ppg_day_interval&0xff); // day interval
	setting_length += 4;
	
	*pdata++ = (u->ppg_night_interval>>24)&0xff; // night interval
	*pdata++ = (u->ppg_night_interval>>16)&0xff; // night interval
	*pdata++ = (u->ppg_night_interval>>8)&0xff; // night interval
	*pdata++ = (u->ppg_night_interval&0xff); // night interval
	setting_length += 4;

	*pdata++ = (u->skin_temp_day_interval>>24)&0xff; // skin temp day interval
	*pdata++ = (u->skin_temp_day_interval>>16)&0xff; // skin temp day interval
	*pdata++ = (u->skin_temp_day_interval>>8)&0xff; // skin temp day interval
	*pdata++ = (u->skin_temp_day_interval&0xff); // skin temp day interval
	setting_length += 4;
	
	*pdata++ = (u->skin_temp_night_interval>>24)&0xff; // skin temp night interval
	*pdata++ = (u->skin_temp_night_interval>>16)&0xff; // skin temp night interval
	*pdata++ = (u->skin_temp_night_interval>>8)&0xff; // skin temp night interval
	*pdata++ = (u->skin_temp_night_interval&0xff); // skin temp night interval
	setting_length += 4;

	// gesture recognition
	*pdata++ = u->b_screen_wrist_flip;
	*pdata++ = u->b_screen_press_hold_1;
	*pdata++ = u->b_screen_press_hold_3;
	*pdata++ = u->b_screen_tapping;
	setting_length += 4;
	
	*pdata++ = u->b_navigation_tapping;
	*pdata++ = u->b_navigation_wrist_shake;
	setting_length += 2;

	*pdata++ = u->idle_time_in_minutes;
	*pdata++ = u->idle_time_start; // Idle alert - start time
	*pdata++ = u->idle_time_end; // Idle alert - end time
	setting_length += 3;
	
	*pdata++ = u->screen_on_general;
	*pdata++ = u->screen_on_heart_rate;
	setting_length += 2;
	
//	*pdata++ = cling.sleep.m_sensitive_mode;
	*pdata++ = 0;
	setting_length ++;
	
	*pdata++ = u->b_reminder_off_weekends;
	setting_length ++;
	
	*pdata++ = u->m_pedo_sensitivity;
	setting_length ++;
	
	// Finally, we put in the length
	data[0] = setting_length;
	
	N_SPRINTF("[USER] critical store device param: %d", setting_length);
}

void USER_setup_profile(I8U *data)
{
	USER_DATA *u = &cling.user_data;
	I16U height_in_cm, weight_in_kg, stride_in_cm;
	I8U *pdata = data;

	height_in_cm = *pdata++; // height
	height_in_cm |= (*pdata++)<<8; // 
	
	weight_in_kg = *pdata++; // weight
	weight_in_kg |= (*pdata++)<<8; // 
	
	stride_in_cm = *pdata++; // stride
	stride_in_cm |= (*pdata++)<<8; // 
	
	Y_SPRINTF("[USER] USER_setup_profile %d, %d, %d", height_in_cm, weight_in_kg, stride_in_cm);
	
	// Initialize stride length and weight for distance calculation
	u->profile.stride_in_cm = stride_in_cm; // In center meters
	u->profile.weight_in_kg = weight_in_kg; // in KG
	u->profile.height_in_cm = height_in_cm; // in center meters

}

static I8U _idle_time_setup(USER_DATA *u, I8U *pdata)
{
	u->idle_time_in_minutes = *pdata++;
	u->idle_time_start = *pdata++; // Idle alert - start time
	u->idle_time_end = *pdata++; // Idle alert - end time
	
	// Do not reset idle alert state machine during data sync
	// u->idle_state = IDLE_ALERT_STATE_IDLE;

	Y_SPRINTF("\n\n idle alert: %d, %d, %d", u->idle_time_in_minutes, u->idle_time_start, u->idle_time_end);

	return 3;
}

static I8U _screen_on_setup(USER_DATA *u, I8U *pdata)
{
	
	u->screen_on_general = *pdata++;
	u->screen_on_heart_rate = *pdata++;

	Y_SPRINTF("\n screen on: %d,%d\n", u->screen_on_general, u->screen_on_heart_rate);

	return 2;
}

static I8U _sleep_sensitive_setup(USER_DATA *u, I8U *pdata)
{
	cling.sleep.m_sensitive_mode = (SLEEP_SENSITIVE_MODE)(*pdata++);
	Y_SPRINTF("\n sleep sensitivity level: %d", cling.sleep.m_sensitive_mode);

	return 1;
}

static I8U _reminder_weekends_setup(USER_DATA *u, I8U *pdata)
{
	u->b_reminder_off_weekends = *pdata++;

	Y_SPRINTF("\n reminder off: %d\n", u->b_reminder_off_weekends);

	return 1;
}
	
static I8U _pedo_sensitivity_setup(USER_DATA *u, I8U *pdata)
{
	u->m_pedo_sensitivity = *pdata++;
	
	if (u->m_pedo_sensitivity > PEDO_SENSITIVITY_LOW) {
		u->m_pedo_sensitivity = PEDO_SENSITIVITY_HIGH;
	}
	Y_SPRINTF("\n Pedo Sensitivity: %d\n", u->m_pedo_sensitivity);

	return 1;
}

static I8U _voc_frequency_setup(USER_DATA *u, I8U *pdata)
{

	return 1;
}

static I8U _etha_sensitivity_setup(USER_DATA *u, I8U *pdata)
{

	return 1;
}

static I8U _ppg_interval_set(USER_DATA *u, I8U *pdata)
{
	I32U value;

	value = *pdata++; // day interval
	value <<= 8; // 
	value |= *pdata++; // 
	value <<= 8; // 
	value |= *pdata++; // 
	value <<= 8; // 
	value |= *pdata++; // 
	// 5 minutes is the minimum
#if 0 // Not used
	if (value < 300000) {
		Y_SPRINTF("[USER] illegal setting(day-hr): %d", value);
		//return;
	}
	u->ppg_day_interval = value;
#endif

	value = *pdata++; // night interval
	value <<= 8; // 
	value |= *pdata++; // 
	value <<= 8; // 
	value |= *pdata++; // 
	value <<= 8; // 
	value |= *pdata++; // 
	// 5 minutes is the minimum
#if 0 // not used
	if (value < 300000) {
		Y_SPRINTF("[USER] illegal setting(night-hr): %d", value);
		//return;
	}
	u->ppg_night_interval = value;
#endif

	return 8;
}

static I8U _skin_temp_interval_set(USER_DATA *u, I8U *pdata)
{
	I32U value;
	
	
	value = *pdata++; // night interval
	value <<= 8; // 
	value |= *pdata++; // 
	value <<= 8; // 
	value |= *pdata++; // 
	value <<= 8; // 
	value |= *pdata++; // 
	// 5 minutes is the minimum
#if 0 // Not used
	if (value < 300000) {
		Y_SPRINTF("[USER] illegal setting(day-temp): %d", value);

		//return;
	}
	u->skin_temp_day_interval = value;
#endif

	value = *pdata++; // night interval
	value <<= 8; // 
	value |= *pdata++; // 
	value <<= 8; // 
	value |= *pdata++; // 
	value <<= 8; // 
	value |= *pdata++; // 
	// 5 minutes is the minimum
#if 0 // Not used
	if (value < 300000) {
				Y_SPRINTF("[USER] illegal setting(night-temp): %d", value);

		//return;
	}
	u->skin_temp_night_interval = value;
	u->skin_temp_day_interval = 5000;
#endif
	return 8;
}

static I8U _gesture_UI_interval_set(USER_DATA *u, I8U *pdata)
{
	// gesture recognition
	u->b_screen_wrist_flip = *pdata++;
	u->b_screen_press_hold_1 = *pdata++;
	u->b_screen_press_hold_3 = *pdata++;
	u->b_screen_tapping = *pdata++;
	u->b_screen_tapping = FALSE; // TURN off screen tapping by default
	
	u->b_navigation_tapping = *pdata++;
	u->b_navigation_wrist_shake = *pdata++;
	
	// No longer support navigation setting
	u->b_navigation_tapping = FALSE;
	u->b_navigation_wrist_shake = FALSE;
	
	return 6;
}

void USER_setup_device(I8U *data, I8U setting_length)
{
	USER_DATA *u = &cling.user_data;
	I8U len=0;

	// Setting length
	//
	// Type: 1 B
	// PPG day interval: 4 B
	// PPG night interval: 4 B
	// SKIN TEMP day interval: 4 B
	// SKIN TEMP night interval: 4 B
	// Screen activation: 4 B
	// Navigation: 2 B
	//
	// New added:
	//
	// Idle alert: 3 B
	// Screen on : 2 B
	// Sleep sensitive mode : 1B
	// Reminder off weekends : 1B
	
	if (len < setting_length) {
		len += _ppg_interval_set(u, data+len);
	}
	
	if (len < setting_length) {
		len += _skin_temp_interval_set(u, data+len);
	}
	
	if (len < setting_length) {
		len += _gesture_UI_interval_set(u, data+len);
	}

	Y_SPRINTF("[USER]: Ppg_day_inter = %d, ppg_night = %d,%d,%d,%d,%d,%d,%d,%d,%d",
		u->ppg_day_interval,
		u->ppg_night_interval,
		u->skin_temp_day_interval,
		u->skin_temp_night_interval,
		u->b_screen_wrist_flip,
		u->b_screen_press_hold_1,
		u->b_screen_press_hold_3,
		u->b_screen_tapping,
		u->b_navigation_tapping,
		u->b_navigation_wrist_shake);
		
	if (len < setting_length) {
		len += _idle_time_setup(u, data+len);
	}
		
	if (len < setting_length) {
		len += _screen_on_setup(u, data+len);
	}
	
	if (len < setting_length) {
		len += _sleep_sensitive_setup(u, data+len);
	}
	
	if (len < setting_length) {
		len += _reminder_weekends_setup(u, data+len);
	}
	
	if (len < setting_length) {
		len += _pedo_sensitivity_setup(u, data+len);
	}
	
	if (len < setting_length) {
		len += _voc_frequency_setup(u, data+len);
	}
	
	if (len < setting_length) {
		len += _etha_sensitivity_setup(u,data+len);
	}
}

extern void ALERT_task_idle_notify(void);

void USER_idle_state_machine_update()
{
	USER_DATA *u = &cling.user_data;
	
	switch (u->idle_state) {
		case IDLE_ALERT_STATE_IDLE:
			if (u->idle_time_in_minutes > 0) {
				if (cling.time.local.hour>=cling.user_data.idle_time_start) {
					if (cling.time.local.hour < cling.user_data.idle_time_end) {
						u->idle_minutes_countdown = u->idle_time_in_minutes;
						u->idle_step_countdown = 64;
						u->idle_state = IDLE_ALERT_STATE_COUNT_DOWN;
						Y_SPRINTF("[USER] reset idle alert: %d, %d", u->idle_minutes_countdown, u->idle_step_countdown);
					}
				}
			}
			break;

		case IDLE_ALERT_STATE_COUNT_DOWN:
			if (u->idle_minutes_countdown ==0) {
				if (u->idle_time_in_minutes == 0) {
					u->idle_state = IDLE_ALERT_STATE_IDLE;
				} else if (cling.time.local.hour < cling.user_data.idle_time_start) {
					u->idle_state = IDLE_ALERT_STATE_IDLE;
				} else if (cling.time.local.hour >= cling.user_data.idle_time_end) {
					u->idle_state = IDLE_ALERT_STATE_IDLE;
				} else {
					if (_is_idle_alert_allowed()) {
			                        u->idle_state = IDLE_ALERT_STATE_IDLE;
			                        ALERT_task_idle_notify();
						Y_SPRINTF("[USER] start idle alert");
					}
				}
			}
			break;

		default:
			u->idle_state = IDLE_ALERT_STATE_IDLE;
			break;
  }
}

