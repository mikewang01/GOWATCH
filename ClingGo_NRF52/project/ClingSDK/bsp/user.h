//
//  File: user.h
//  
//  Description: USER data process header
//
//  Created on Feb 6, 2014
//
#ifndef __USER_HEADER__
#define __USER_HEADER__

enum {
	IDLE_ALERT_STATE_IDLE,
	IDLE_ALERT_STATE_COUNT_DOWN,
};
			
typedef struct tagUSER_PROFILE_CTX {
#if 0
	I8U name[32]; // Limit user name length to 32 bytes
	I8U name_len;
#endif
	I16U weight_in_kg;
	I16U height_in_cm;
  I16U stride_in_cm;
	BOOLEAN metric_distance;
} USER_PROFILE_CTX;

typedef struct tagUSER_DATA {
	
	// System measurement setting
	I32U ppg_day_interval;
	I32U ppg_night_interval;
	I32U skin_temp_day_interval;
	I32U skin_temp_night_interval;
	
	// User profile
	USER_PROFILE_CTX profile;
	
	// device length
	I8U setting_len;
	// User dynamic data 
	I16U calories_factor;
	
	// gesture recognition
	BOOLEAN b_screen_wrist_flip;
	BOOLEAN b_screen_press_hold_1;
	BOOLEAN b_screen_press_hold_3;
	BOOLEAN b_screen_tapping;
	
	BOOLEAN b_navigation_tapping;
	BOOLEAN b_navigation_wrist_shake;
	
	// Idle alert
	I8U idle_state;
	I8U idle_time_in_minutes;
	I8U idle_minutes_countdown;
	I8U idle_step_countdown;
	I8U idle_time_start;
	I8U idle_time_end;
	
	// Screen ON time
	I8U screen_on_general;
	I8U screen_on_heart_rate;
	
	// Reminder OFF during weekends
	BOOLEAN b_reminder_off_weekends;
	
	// Pedometer sensitivity mode
	I8U m_pedo_sensitivity;

} USER_DATA;

void USER_data_init(void);
void USER_profile_update(void);
void USER_setup_device(I8U *data, I8U setting_length);
void USER_idle_state_machine_update(void);
void USER_store_device_param(I8U *data);
void USER_setup_profile(I8U *data);

#endif  // __USER_HEADER__
