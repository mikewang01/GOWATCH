

#ifndef __SLEEP_H__
#define __SLEEP_H__

#include "standards.h"

#define SLP_CORR_WINDOW_LONG                                          8
#define SLP_CORR_WINDOW_SHORT                                         2

// accelerometer sample correlation value threshold           
#define AWAKE_CORR_THRESHOLD_LOW                                 140000
#define AWAKE_CORR_THRESHOLD_MEDIUM                              120000
#define AWAKE_CORR_THRESHOLD_HIGH                                100000

#define SLEEP_CORR_THRESHOLD_LOW                                 240000
#define SLEEP_CORR_THRESHOLD_MEDIUM                              220000
#define SLEEP_CORR_THRESHOLD_HIGH                                200000

// awake->light and light->sound entering threshold (in mins)
#define AWAKE_TO_LIGHT_THRESHOLD_LOW                                 14
#define AWAKE_TO_LIGHT_THRESHOLD_MEDIUM                              12
#define AWAKE_TO_LIGHT_THRESHOLD_HIGH                                10

#define LIGHT_TO_SOUND_THRESHOLD_LOW                                 14
#define LIGHT_TO_SOUND_THRESHOLD_MEDIUM                              12
#define LIGHT_TO_SOUND_THRESHOLD_HIGH                                10

// activity detected per mins threshold                             
#define AWAKE_ACTIVITY_PER_MIN_THRESHOLD_LOW                          5
#define AWAKE_ACTIVITY_PER_MIN_THRESHOLD_MEDIUM                       3
#define AWAKE_ACTIVITY_PER_MIN_THRESHOLD_HIGH                         1

#define SLEEP_ACTIVITY_PER_MIN_THRESHOLD_LOW                         12
#define SLEEP_ACTIVITY_PER_MIN_THRESHOLD_MEDIUM                      10
#define SLEEP_ACTIVITY_PER_MIN_THRESHOLD_HIGH                         8

// wake up from sleep threshold                                     
#define WAKE_UP_ACTIVITY_PER_MIN_THRESHOLD_LOW                      100
#define WAKE_UP_ACTIVITY_PER_MIN_THRESHOLD_MEDIUM                    90
#define WAKE_UP_ACTIVITY_PER_MIN_THRESHOLD_HIGH                      80

// successive stationary minutes threshold
#define SUCCESSIVE_STATIONARY_MINS                                   10

#define SLP_MINS_LEN1                                                 2
#define SLP_MINS_LEN2                                                 4
#define SLP_MINS_LEN3                                                 8
#define SLP_MINS_LEN4                                                16

typedef enum {
	SLP_STAT_IDLE,
	SLP_STAT_AWAKE,
	SLP_STAT_LIGHT,
	SLP_STAT_SOUND,
	SLP_STAT_REM
} SLEEP_STATUSCODE;

typedef enum {
	SLEEP_SENSITIVE_LOW, 
	SLEEP_SENSITIVE_MEDIUM, 
	SLEEP_SENSITIVE_HIGH 
} SLEEP_SENSITIVE_MODE;

typedef struct tagSLEEP_CTX {
	I32U slp_measue_timer;

	SLEEP_STATUSCODE 	state;
	ACC_AXIS  m_pre_sample;

	I16S m_activity_per_min;
	I8S  m_mins_cnt;
	I32U m_activity_status;
	I8U  m_stationary_mins;

  // variables defined for valid monitoring sleep status switching cases
  // 1. from awake state switching to sleep state
	// 2. from sleep state switching to awake state
	I8S  m_successive_same_state_mins;
	BOOLEAN b_entered_sleep_stage;
	SLEEP_STATUSCODE 	pre_state;
	
	// variables defined for monitoring whether device has been correctly worn on the 
	// wrist in the entering sleep state stage
	I8S  m_successive_no_skin_touch_mins;
	BOOLEAN b_valid_worn_in_entering_sleep_state;

  // variable defined for detecting waking up from sleep state
	BOOLEAN b_sudden_wake_from_sleep;

	SLEEP_SENSITIVE_MODE m_sensitive_mode;
	I8U  step_status;
	BOOLEAN b_step_flag;

  I8U  m_successive_stationary_mins;
	I32U activity_status_per_min;
	I32U sound_sleep_timestamp;
} SLEEP_CTX;

void SLEEP_init(void);
void SLEEP_algorithms_proc(ACC_AXIS *xyz);
void SLEEP_minute_proc(void);
void SLEEP_wake_up_by_force(BOOLEAN b_motion);
BOOLEAN SLEEP_is_sleep_state(SLEEP_STATUSCODE s);

#ifdef USING_VIRTUAL_ACTIVITY_SIM

void SLEEP_activity_minute_sim(int activity_per_min, int steps, int bWearing);
#endif
#endif

