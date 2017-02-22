
#include "main.h"

BOOLEAN SLEEP_is_sleep_state(SLEEP_STATUSCODE s)
{
	SLEEP_CTX *slp = &cling.sleep;
	return (s==slp->state);
}

static void _update_activity_status_register(I8U stat)
{
	SLEEP_CTX *slp = &cling.sleep;

	slp->m_activity_status <<= 1;
	slp->m_activity_status |= stat;
}

static I32U _calc_sample_corr(ACC_AXIS s1, ACC_AXIS s2)
{
	return (s1.x - s2.x)*(s1.x - s2.x) + \
		     (s1.y - s2.y)*(s1.y - s2.y) + \
	       (s1.z - s2.z)*(s1.z - s2.z);
}

void SLEEP_wake_up_by_force(BOOLEAN b_motion)
{
	SLEEP_CTX *slp = &cling.sleep;
	slp->b_sudden_wake_from_sleep = TRUE;
	slp->b_step_flag = b_motion;
}

static void _calc_activity_per_minute(ACC_AXIS d)
{
	I32S t_curr1, t_diff1;
	I32S epoch_period;

	I32U corr_val;

	SLEEP_CTX *slp = &cling.sleep;

	I32U threshold;

	if (SLEEP_is_sleep_state(SLP_STAT_AWAKE)) {
		switch (slp->m_sensitive_mode) {
			case SLEEP_SENSITIVE_LOW    : threshold = AWAKE_CORR_THRESHOLD_LOW;     break;
			case SLEEP_SENSITIVE_MEDIUM : threshold = AWAKE_CORR_THRESHOLD_MEDIUM;  break;
			case SLEEP_SENSITIVE_HIGH   : threshold = AWAKE_CORR_THRESHOLD_HIGH;    break;
			default :                     threshold = AWAKE_CORR_THRESHOLD_MEDIUM;  break;
		}
	}
	else if (SLEEP_is_sleep_state(SLP_STAT_LIGHT) || SLEEP_is_sleep_state(SLP_STAT_SOUND)) {
		switch (slp->m_sensitive_mode) {
			case SLEEP_SENSITIVE_LOW    : threshold = SLEEP_CORR_THRESHOLD_LOW;     break;
			case SLEEP_SENSITIVE_MEDIUM : threshold = SLEEP_CORR_THRESHOLD_MEDIUM;  break;
			case SLEEP_SENSITIVE_HIGH   : threshold = SLEEP_CORR_THRESHOLD_HIGH;    break;
			default :                     threshold = SLEEP_CORR_THRESHOLD_MEDIUM;  break;
		}
	} else {
		threshold = AWAKE_CORR_THRESHOLD_MEDIUM;
	}

	if (!cling.lps.b_low_power_mode)
	{
		t_curr1 = CLK_get_system_time();
		t_diff1 = t_curr1 - slp->slp_measue_timer;

		if ( SLEEP_is_sleep_state(SLP_STAT_AWAKE) ) {
			epoch_period = SLP_CORR_WINDOW_SHORT;
		} else {
			epoch_period = SLP_CORR_WINDOW_LONG;
		}

		if ( t_diff1 > ( epoch_period * 20 ) ) {
			slp->slp_measue_timer = t_curr1;
			corr_val = _calc_sample_corr(d, slp->m_pre_sample );

			if ( corr_val > threshold ) {
				slp->m_activity_per_min++;
			}

			slp->m_pre_sample.x = d.x;
			slp->m_pre_sample.y = d.y;
			slp->m_pre_sample.z = d.z;
		}
	}
}

static void _update_stationary_mins()
{
	SLEEP_CTX *slp = &cling.sleep;
	I8U  i;

	slp->m_stationary_mins = 0;
	for (i=0; i<16; i++) {
		if (((slp->m_activity_status)&(1<<i))==0) {
			slp->m_stationary_mins++;
	  }
  }
}

//static BOOLEAN _device_is_worn()
//{
//	SLEEP_CTX *slp = &cling.sleep;
//
//	if (slp->b_valid_worn_in_entering_sleep_state) {
//		return TRUE;
//	} else {
//		// if touch pads say it is skin touched, trust it!
//		return FALSE;
//	}
//}

static BOOLEAN _sleep_monitor_allowed()
{
	SLEEP_CTX *slp = &cling.sleep;

	if (BATT_charging_det_for_sleep())
		return FALSE;

	if (slp->b_valid_worn_in_entering_sleep_state)
		return TRUE;

	return FALSE;
}

static void _sleep_main_state()
{
	SLEEP_CTX *slp = &cling.sleep;

	BOOLEAN b_enter_light_sleep    = FALSE;
	BOOLEAN b_wake_from_sleep      = FALSE;
	BOOLEAN b_enter_sound_sleep    = FALSE;

	I32S threshold_entering_light;
	I32S threshold_entering_sound;

	// put the device into awake mode when its in successive stationary minutes more than

	switch (slp->m_sensitive_mode) {
		case SLEEP_SENSITIVE_LOW    :
			threshold_entering_light = AWAKE_TO_LIGHT_THRESHOLD_LOW;
		  threshold_entering_sound = LIGHT_TO_SOUND_THRESHOLD_LOW;
		  break;

		case SLEEP_SENSITIVE_MEDIUM :
			threshold_entering_light = AWAKE_TO_LIGHT_THRESHOLD_MEDIUM;
		  threshold_entering_sound = LIGHT_TO_SOUND_THRESHOLD_MEDIUM;
		  break;

		case SLEEP_SENSITIVE_HIGH   :
			threshold_entering_light = AWAKE_TO_LIGHT_THRESHOLD_HIGH;
		  threshold_entering_sound = LIGHT_TO_SOUND_THRESHOLD_HIGH;
		  break;

		default :
			threshold_entering_light = AWAKE_TO_LIGHT_THRESHOLD_MEDIUM;
		  threshold_entering_sound = LIGHT_TO_SOUND_THRESHOLD_MEDIUM;
		  break;
	}

	// If user is currently in awake state
	if (slp->state==SLP_STAT_AWAKE) {

		if (
				 ((slp->b_entered_sleep_stage==TRUE) &&
					(slp->m_mins_cnt>=3) &&
					(slp->m_activity_status&0x7)==0x0) ||         // when wakeup in midnight in sudden activity (not walk/run),
																												// fall asleep again in 3 stationary minutes.

				 ((slp->b_entered_sleep_stage==FALSE) &&
					(slp->m_mins_cnt>=threshold_entering_light) &&
					((slp->m_activity_status&0x3f)==0x0) &&        // when in solid entering sleep stage, must have 6 stationary minutes.
					(slp->m_stationary_mins>=13) &&               // in the past 16 minutes, must have 13 statioanry minutes above.
					(slp->m_successive_stationary_mins<=SUCCESSIVE_STATIONARY_MINS))      // if device has been put on desk, omit sleep in this case.
			 )
		{
			b_enter_light_sleep = TRUE;
		}
	}

	// Do not wake up from sleep until 5 minutes in a state
	if ( (slp->b_sudden_wake_from_sleep) &&
		   ( (slp->state==SLP_STAT_SOUND) || slp->state==SLP_STAT_LIGHT) )   {
    b_wake_from_sleep = TRUE;
	}

	if ( slp->m_mins_cnt>=threshold_entering_sound &&
		  (slp->m_activity_status&0xfff)==0x0) {
			 if (slp->state==SLP_STAT_LIGHT)
    b_enter_sound_sleep = TRUE;
	}

  switch (slp->state) {
		case SLP_STAT_IDLE:
			SLEEP_init();
		  break;
		case SLP_STAT_AWAKE:
			if (_sleep_monitor_allowed()) {
				if (b_enter_light_sleep) {
					slp->b_sudden_wake_from_sleep = FALSE;
					slp->m_mins_cnt = 0;
					slp->m_activity_status = 0;
					slp->state = SLP_STAT_LIGHT;
				}
			}
			else {
				slp->b_sudden_wake_from_sleep = FALSE;
				slp->m_mins_cnt = 0;
				slp->m_activity_status = 0;
			}
			break;

		case SLP_STAT_LIGHT:
			if (b_wake_from_sleep) {
				slp->m_mins_cnt = 0;
				slp->m_activity_status = 0;

				slp->state = SLP_STAT_AWAKE;
			}
			else if (b_enter_sound_sleep) {
				slp->m_mins_cnt = 0;
				slp->m_activity_status = 0;
				slp->state = SLP_STAT_SOUND;
			}
			break;

		case SLP_STAT_SOUND:
			if ( slp->b_sudden_wake_from_sleep || (slp->m_activity_status&0x1)!=0 ) {
				slp->m_mins_cnt = 0;
				slp->m_activity_status = 0;
				slp->state = SLP_STAT_LIGHT;
			}
			break;

		case SLP_STAT_REM:
		  SLEEP_init();
			break;

		default:
		  SLEEP_init();
			break;
	}
}

static void _wristband_not_wearing_detection(SLEEP_CTX *slp)
{
	I32U activityState, activeMinutes;
	int i;

	if (slp->m_successive_stationary_mins < SUCCESSIVE_STATIONARY_MINS) {
		return;
	}

	// Check the latest 24 minutes
	activityState = slp->activity_status_per_min & 0x0ffffff;
	activeMinutes = 0;

	// Counting the amount of movement minutes
	for (i = 0; i < 32; i++) {
		if (activityState & 0x01) {
			activeMinutes ++;
		}
		activityState >>= 1;
	}

	// If user has very active 8 minutes in past 24 minutes, and the latest 10 minutes is completely stationary
	// we think this is when user puts down the wristband
	if (activeMinutes >= 8) {
		// Reset non charging steps & activity seconds
		cling.batt.non_charging_accumulated_steps = 0;
		cling.batt.non_charging_accumulated_active_sec = 0;
	}
}

#define TOUCH_is_skin_touched() true

void SLEEP_minute_proc()
{
	SLEEP_CTX *slp = &cling.sleep;
	I8U  threshold = 0;
	I8U  wakeup_threshold = 0;
	I32U time_diff;
	I8U  step_status_tmp;
	I8U  i, step_cnt;

	if ( SLEEP_is_sleep_state(SLP_STAT_LIGHT) || SLEEP_is_sleep_state(SLP_STAT_SOUND) ) {    // check whether the device has been put on the desk when getting up..

		// Get out of sleep mode if device has been off the wrist for more than 120 minutes (1 hour)
    	if (slp->m_successive_no_skin_touch_mins > 120) {
			slp->b_sudden_wake_from_sleep = TRUE;
		}

		// As user is currently in sleep state, reset idle alert
		cling.user_data.idle_state = IDLE_ALERT_STATE_IDLE;

		// As we detect sleep state, adjust step detection threshold to low sensitivity
		PEDO_set_step_detection_sensitivity(FALSE);
	}

	// Check unnormal sleep state (Special when skin touch sensor doesn't work)
	// If a device stays in deep sleep for over 90 minutes, this is unusual, wake it up from sleep, and
  // put it in a charging recovery state
	if (SLEEP_is_sleep_state(SLP_STAT_SOUND)) {

		time_diff = cling.time.system_clock_in_sec - slp->sound_sleep_timestamp;

		if (time_diff > 7200) {
			slp->b_sudden_wake_from_sleep = TRUE;
			cling.batt.non_charging_accumulated_steps = 0;
			cling.batt.non_charging_accumulated_active_sec = 0;
		}
	} else {
		slp->sound_sleep_timestamp = cling.time.system_clock_in_sec;
	}

	// monitoring sleep stage switching.
	if (
	     ( (slp->state==SLP_STAT_LIGHT || slp->state==SLP_STAT_SOUND) && (slp->pre_state==SLP_STAT_LIGHT || slp->pre_state==SLP_STAT_SOUND) ) ||
	     (  slp->state==SLP_STAT_AWAKE && slp->pre_state==SLP_STAT_AWAKE )
     )
	{
		slp->m_successive_same_state_mins++;
	}
	else
	{
		slp->m_successive_same_state_mins = 0;
	}

	if ( (slp->state==SLP_STAT_LIGHT || slp->state==SLP_STAT_SOUND) && slp->m_successive_same_state_mins>=20 ) {
		slp->b_entered_sleep_stage = TRUE;
	}
	else if ( slp->state==SLP_STAT_AWAKE && slp->m_successive_same_state_mins>=10 ) {
		slp->b_entered_sleep_stage = FALSE;
	}

	slp->pre_state = slp->state;

	// monitoring whether device has been correctly worn
	if (TOUCH_is_skin_touched()) {
		slp->m_successive_no_skin_touch_mins = 0;
		slp->b_valid_worn_in_entering_sleep_state = TRUE;
	} else {
		slp->m_successive_no_skin_touch_mins++;
	}

	// detect walking/running step status
	if ( slp->b_entered_sleep_stage ) {         // only check when having really entered sleep state.
		slp->step_status <<= 1;      // shift/update step status in this epoch.


		if (slp->b_step_flag == TRUE) {        // one minute..
			slp->step_status |= 1;    // walking/running status in this epoch
			slp->b_step_flag = FALSE;
		}

		step_status_tmp = slp->step_status & 0x1F;      // two waling/running minutes in 3 minutes
		step_cnt = 0;
		for (i=0; i<5; i++) {
			if ( step_status_tmp & ((0x01)<<i) ) {
				step_cnt++;
			}
		}

		if (step_cnt>=3)  {
			slp->b_entered_sleep_stage = FALSE;
		}

		if ( (slp->step_status & 0x03)!=0 &&  (slp->m_successive_no_skin_touch_mins>0) ) {
			slp->b_entered_sleep_stage = FALSE;
		}
	}

	if (
		   (slp->m_successive_no_skin_touch_mins>5  &&  SLEEP_is_sleep_state(SLP_STAT_AWAKE)) ||
       (slp->m_successive_no_skin_touch_mins>90 && (SLEEP_is_sleep_state(SLP_STAT_LIGHT)  || SLEEP_is_sleep_state(SLP_STAT_SOUND)))
		)
  {
		slp->b_valid_worn_in_entering_sleep_state = FALSE;
	}

	if ( SLEEP_is_sleep_state(SLP_STAT_AWAKE) ) {
		switch (slp->m_sensitive_mode) {
			case SLEEP_SENSITIVE_LOW    : threshold = AWAKE_ACTIVITY_PER_MIN_THRESHOLD_LOW;     break;
			case SLEEP_SENSITIVE_MEDIUM : threshold = AWAKE_ACTIVITY_PER_MIN_THRESHOLD_MEDIUM;  break;
			case SLEEP_SENSITIVE_HIGH   : threshold = AWAKE_ACTIVITY_PER_MIN_THRESHOLD_HIGH;    break;
			default                     : threshold = AWAKE_ACTIVITY_PER_MIN_THRESHOLD_MEDIUM;  break;
		}
	}
	else if ( SLEEP_is_sleep_state(SLP_STAT_LIGHT) || SLEEP_is_sleep_state(SLP_STAT_SOUND) ) {
		switch (slp->m_sensitive_mode) {
			case SLEEP_SENSITIVE_LOW    : threshold = SLEEP_ACTIVITY_PER_MIN_THRESHOLD_LOW;     break;
			case SLEEP_SENSITIVE_MEDIUM : threshold = SLEEP_ACTIVITY_PER_MIN_THRESHOLD_MEDIUM;  break;
			case SLEEP_SENSITIVE_HIGH   : threshold = SLEEP_ACTIVITY_PER_MIN_THRESHOLD_HIGH;    break;
			default                     : threshold = SLEEP_ACTIVITY_PER_MIN_THRESHOLD_MEDIUM;  break;
		}
	}

	switch (slp->m_sensitive_mode) {
    case SLEEP_SENSITIVE_LOW      : wakeup_threshold = WAKE_UP_ACTIVITY_PER_MIN_THRESHOLD_LOW;     break;
		case SLEEP_SENSITIVE_MEDIUM 	: wakeup_threshold = WAKE_UP_ACTIVITY_PER_MIN_THRESHOLD_MEDIUM;  break;
		case SLEEP_SENSITIVE_HIGH     : wakeup_threshold = WAKE_UP_ACTIVITY_PER_MIN_THRESHOLD_HIGH;    break;
    default                       : wakeup_threshold = WAKE_UP_ACTIVITY_PER_MIN_THRESHOLD_MEDIUM;  break;
	}

  if (slp->m_mins_cnt<31)
  	slp->m_mins_cnt++;

  if ( (slp->m_activity_per_min>=wakeup_threshold) &&
		   (SLEEP_is_sleep_state(SLP_STAT_SOUND) || SLEEP_is_sleep_state(SLP_STAT_LIGHT)) )
	{
    slp->b_sudden_wake_from_sleep = TRUE;
	}

  if (slp->m_activity_per_min>=threshold)
  	_update_activity_status_register(1);
  else
  	_update_activity_status_register(0);

	N_SPRINTF("[SLEEP] activity per min:  %d", slp->m_activity_per_min);

	// Update no charging parameters
	if (slp->m_activity_per_min >= AWAKE_ACTIVITY_PER_MIN_THRESHOLD_LOW) {
		BATT_exit_charging_state(60);
	}

	// count successive  stationary minutes
	slp->activity_status_per_min <<= 1;
	if (slp->m_activity_per_min == 0) {
		slp->m_successive_stationary_mins++;
	} else {
		slp->m_successive_stationary_mins = 0;

		// Recording the minutes that user has dramatic movement
		if (slp->m_activity_per_min >= WAKE_UP_ACTIVITY_PER_MIN_THRESHOLD_HIGH)
			slp->activity_status_per_min |= 1;
	}

  slp->m_activity_per_min = 0;

	//
	// Detect if wristband is put on desk -
	//
	// If yes, then, exit from sleep first
	//
	// Also, start monitoring steps and active seconds untile user starts to using the band again.
	//
	_wristband_not_wearing_detection(slp);

	// count stationary minutes.
	_update_stationary_mins();
}

void SLEEP_init()
{
	SLEEP_CTX *slp = &cling.sleep;

	slp->slp_measue_timer = CLK_get_system_time();

	slp->state = SLP_STAT_AWAKE;
	slp->m_pre_sample.x = 0;
	slp->m_pre_sample.y = 0;
	slp->m_pre_sample.z = 0;

	slp->m_activity_per_min = 0;
        slp->m_activity_status  = 0;
        slp->m_mins_cnt         = 0;
	slp->m_stationary_mins  = 0;

	slp->m_successive_same_state_mins = 0;
	slp->b_entered_sleep_stage = FALSE;
	slp->pre_state = SLP_STAT_AWAKE;

	slp->b_sudden_wake_from_sleep = FALSE;

//slp->m_sensitive_mode = SLEEP_SENSITIVE_MEDIUM;
	slp->m_sensitive_mode = SLEEP_SENSITIVE_LOW;

	slp->m_successive_no_skin_touch_mins = 0;
	slp->b_valid_worn_in_entering_sleep_state = TRUE;
	slp->step_status = 0x00;
	slp->b_step_flag = FALSE;

	slp->m_successive_stationary_mins = 0;
}

void SLEEP_algorithms_proc(ACC_AXIS *xyz)
{
	ACC_AXIS current_sample;

	current_sample.x = xyz->x;
	current_sample.y = xyz->y;
	current_sample.z = xyz->z;

	// calc activity status per minute..
	_calc_activity_per_minute(current_sample);

	// sleep monitering main state machine.
	_sleep_main_state();
}

#ifdef _SLEEP_SIMULATION_


void SLEEP_activity_minute_sim(int activity_per_min, int steps, int skin_touch)
{
	SLEEP_CTX *slp = &cling.sleep;

	slp->m_activity_per_min = activity_per_min;

	if (skin_touch > 0) {
		cling.touch.b_skin_touch = skin_touch;
	}
	else {
		cling.touch.b_skin_touch = skin_touch;
	}

	if (steps > 4)
		SLEEP_wake_up_by_force(TRUE);

	//
	_sleep_main_state();
}
#endif

