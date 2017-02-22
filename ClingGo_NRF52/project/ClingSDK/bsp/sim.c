/***************************************************************************/
/**
 * File: sim.c
 * 
 * Description: virtual device simulator
 *
 * Created on May 15, 2014
 *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>


#include "main.h"
//#include "virtual_activity.h"

#ifdef USING_VIRTUAL_ACTIVITY_SIM

const I8U weather_sim_array[5][5] =
{
	{5, 22, 0, 16, 24},
	{5, 23, 1, 15, 22},
	{5, 24, 2, 18, 27},
	{5, 25, 3, 21, 30},
	{5, 21, 0, 24, 29},
};

const I32U tracking_sim_array[9][5] =
{
	// format:
	// yyyymmdd, sleep, steps, distance, calories
	{0x07de050c, 443,  9076, 6882, 2972},
	{0x07de050d, 472, 11672, 7062, 2382},
	{0x07de050e, 463, 10345, 6546, 2084},
	{0x07de050f, 500, 14720, 8752, 3672},
	//{         0,   0,     0,    0,    0},
	{0x07de0510, 519, 11345, 6982, 2367},
	//{         0,   0,     0,    0,    0},
		{0x07de0511, 453, 12785, 7182, 3276},
	//{         0,   0,     0,    0,    0},
		{0x07de0512, 497,  9745, 6072, 2074},
	//{         0,   0,     0,    0,    0},
		{0x07de0513, 383,  7645, 5232, 1837},
	//{         0,   0,     0,    0,    0},
		{0x07de0514, 418,  7655, 4982, 1263},
//	{         0,   0,     0,    0,    0},
};

static void _tracking_init()
{
	I8U i;
	I32U offset = 0;
	
	// Erase this space first
	FLASH_erase_App(SYSTEM_DAYSTAT_SPACE_START);

  // Add latency before storing messages (Erasure latency: 50 ms)		
  BASE_delay_msec(50); 
	
	for (i = 0; i < 9; i++) {
		FLASH_Write_App(SYSTEM_DAYSTAT_SPACE_START+offset, (I8U *)tracking_sim_array[i], TRACKING_DAY_STAT_SIZE);
		offset += 64; // Reserve 64 bytes space
	}
}

static void _weather_init()
{
	WEATHER_CTX *w = cling.weather;
	
	memcpy(&w[0], weather_sim_array[0], sizeof(WEATHER_CTX));
	memcpy(&w[1], weather_sim_array[1], sizeof(WEATHER_CTX));
	memcpy(&w[2], weather_sim_array[2], sizeof(WEATHER_CTX));
	memcpy(&w[3], weather_sim_array[3], sizeof(WEATHER_CTX));
	memcpy(&w[4], weather_sim_array[4], sizeof(WEATHER_CTX));
}

void SIM_init()
{
	// We should add system restoration for critical info
	CLING_TIME_CTX *t = &cling.time;
	TRACKING_CTX *a = &cling.activity;

	// Restoring the time zone info
	// Shanghai: UTC +8 hours (units in 15 minutes) -> 8*(60/15) = 32 minutes
	t->time_zone = 32;
	t->time_since_1970 = 1400666400;  // 2014, 5, 21, 18:00
	cling.time.local.year = 2015;
	cling.time.local.month = 10;
	cling.time.local.day = 29;
	cling.time.local.hour = 18;
	cling.time.local.minute = 0;
	cling.time.local.second = 0;
	t->time_since_1970 += 365 * 24 * 60 * 60;  // year
	t->time_since_1970 += (31 + 30 + 31 + 31 + 30 + 8) * 24 * 60 * 60; // days
	t->time_since_1970 -= 18 * 60 * 60;
	RTC_get_local_clock(t->time_since_1970, &cling.time.local);

	N_SPRINTF("[SIM] virtual device gets initialized");
	
	_weather_init();
	
	_tracking_init();
	
	// Make sure the minute file has correct offset
	a->tracking_flash_offset = TRACKING_get_daily_total(&a->day);

	// Get current local minute
	cling.time.local_day = cling.time.local.day;
	cling.time.local_minute = cling.time.local.minute;

	// Update stored total
	a->sleep_by_noon = TRACKING_get_sleep_by_noon(FALSE);
	a->sleep_stored_by_noon = TRACKING_get_sleep_by_noon(TRUE);
	a->day_stored.walking = a->day.walking;
	a->day_stored.running = a->day.running;
	a->day_stored.distance = a->day.distance;
	a->day_stored.calories = a->day.calories;

	// Minute file critical timing info
	cling.system.reset_count = 0;
#if 0
	// Enable OTA
	cling.ota.percent = 0;
	OTA_set_state(TRUE);
#endif

#ifdef _CLING_PC_SIMULATION_
	// initialize random seed
	srand(time(NULL));
#endif

}

#endif

#ifdef _ACTIVITY_SIM_BASED_ON_EPOCH_
static MINUTE_TRACKING_CTX _activity_sim_with_type(SIM_ACTIVITY_TYPE type)
{
	MINUTE_TRACKING_CTX a;
	
	switch (type) {
		case SIM_ACTIVITY_WALKING:
		{
			a.walking = 120;
			a.running = 0;
			a.distance = 90;
			a.sleep_state = SLP_STAT_AWAKE;
			a.calories = 7;
			a.skin_temperature = 320;
			a.heart_rate = 95;
			a.skin_touch_pads = 4;
			a.activity_count = 80;
			break;
		}
		case SIM_ACTIVITY_REST:
		{
			a.walking = 0;
			a.running = 0;
			a.distance = 0;
			a.sleep_state = SLP_STAT_AWAKE;
			a.calories = 1;
			a.skin_temperature = 315;
			a.heart_rate = 74;
			a.skin_touch_pads = 4;
			a.activity_count = 10;
			break;
		}
		case SIM_ACTIVITY_RUNNING:
		{
			a.walking = 0;
			a.running = 180;
			a.distance = 254;
			a.sleep_state = SLP_STAT_AWAKE;
			a.calories = 13;
			a.skin_temperature = 330;
			a.heart_rate = 142;
			a.skin_touch_pads = 4;
			a.activity_count = 100;
			break;
		}
		case SIM_ACTIVITY_SLEEP:
		{
			a.walking = 0;
			a.running = 0;
			a.distance = 0;
			a.sleep_state = SLP_STAT_SOUND;
			a.calories = 1;
			a.skin_temperature = 306;
			a.heart_rate = 52;
			a.skin_touch_pads = 4;
			a.activity_count = 0;
			break;
		}
	}
	return a;
	
}

static MINUTE_TRACKING_CTX _get_activity_with_epoch()
{
	SYSTIME_CTX local;
	I32U i; // Time offset in minute)
	
	RTC_get_local_clock(cling.time.time_since_1970, &local);

	i = local.hour*60+local.minute;
	
	if (i < 35) {
		return _activity_sim_with_type(SIM_ACTIVITY_WALKING);
	} else if (i < 100) {
		return _activity_sim_with_type(SIM_ACTIVITY_REST);
	} else if (i < 115) {
		return _activity_sim_with_type(SIM_ACTIVITY_RUNNING);
	} else if (i < 295) {
		return _activity_sim_with_type(SIM_ACTIVITY_REST);
	} else if (i < 790) {
		return _activity_sim_with_type(SIM_ACTIVITY_SLEEP);
	} else if (i < 820) {
		return _activity_sim_with_type(SIM_ACTIVITY_REST);
	} else if (i < 840) {
		return _activity_sim_with_type(SIM_ACTIVITY_WALKING);
	} else if (i < 1020) {
		return _activity_sim_with_type(SIM_ACTIVITY_REST);
	} else if (i < 1060) {
		return _activity_sim_with_type(SIM_ACTIVITY_WALKING);
	} else if (i < 1180) {
		return _activity_sim_with_type(SIM_ACTIVITY_REST);
	} else if (i < 1190) {
		return _activity_sim_with_type(SIM_ACTIVITY_WALKING);
	} else {
		return _activity_sim_with_type(SIM_ACTIVITY_REST);
	}
}

I16S SIM_get_current_activity(I8U type)
{
	MINUTE_TRACKING_CTX minute_activity;

	minute_activity = _get_activity_with_epoch();
	
	switch (type) {
		case TRACKING_SKIN_TEMPERATURE:
			return minute_activity.skin_temperature;
		case TRACKING_HEART_RATE:
			return minute_activity.heart_rate;
		case TRACKING_SKIN_TOUCH:
			return minute_activity.skin_touch_pads;
		case TRACKING_WALK:
			return minute_activity.walking;
		case TRACKING_RUN:
			return minute_activity.running;
		case TRACKING_DISTANCE:
			return minute_activity.distance;
		case TRACKING_CALORIES:
			return minute_activity.calories;
		case TRACKING_SLEEP:
			return minute_activity.sleep_state;
		case TRACKING_ACTIVITY:
			return minute_activity.activity_count;
	}

	return 0;
}
#endif

void SIM_setup_idle_alert()
{
	cling.user_data.idle_time_in_minutes = 60;
	cling.user_data.idle_time_start = 8; // Idle alert - start time
	cling.user_data.idle_time_end = 20; // Idle alert - end time
}

void SIM_setup_reminder()
{
	I8U total = 64;
	I8U hour[32] = { 
		 0,  1,  2,  2,  3,  4,  5,  6,
		 6,  6,  7,  8,  8,  9, 10, 11,
		12, 13, 14, 15, 16, 17, 18, 19,
		20, 20, 20, 21, 21, 21, 22, 23
	};
	I8U minute[32] = {
		2, 4, 12, 56, 29, 2, 6, 27,
		42, 58, 2, 13, 29, 17, 29, 39,
		21, 7, 12, 33, 45, 6, 9, 23,
		28, 29, 39, 29, 35, 58, 17, 12
	};

	I8U msg[128];
	I8U i, j;

	msg[0] = total;
	j = 1;
	for (i = 0; i < (msg[0]>>1); i++) {
		msg[j++] = hour[i];
		msg[j++] = minute[i];
	}

	REMINDER_setup(msg, FALSE);
}

#ifdef _SLEEP_SIMULATION_
int LIS3DH_x = 0;
int LIS3DH_y = 0;
int LIS3DH_z = 0;
int LIS3DH_is_awake;

#define MAXIMUM_MINUTE_ENTRIES 1440

double acc_sim_table[] = {
	   0, 0.001, 1, 30,  // 0
	   60, 0.002, 1, 30,  // 1
	   120, 0.003, 1, 30,  // 2
	   170, 1.001, 1, 30,
	   171, 0.003, 0, 28,
	   180, 0.004, 0, 28,  // 3
	   240, 0.002, 0, 28,  // 4
	   300, 0.002, 1, 30,  // 5
	   301, 1.850, 1, 30,
	   302, 0.002, 0, 28,
	   360, 0.003, 0, 28,  // 6
	   420, 0.008, 1, 30,  // 7
	   480, 0.009, 1, 30,  // 8
	   540, 0.010, 0, 28,  // 9
	   600, 1.010, 0, 28,  // 10
	   660, 1.020, 1, 30,  // 11
	   720, 1.030, 1, 30,  // 12
	   780, 1.040, 0, 28,  // 13
	   840, 1.050, 0, 28,  // 14
	   900, 0.004, 1, 30,  // 15
	   960, 0.004, 1, 30,  // 16
	   1020, 0.004, 0, 28,  // 17
	   1080, 0.004, 0, 28,  // 18
	   1140, 1.500, 1, 30,  // 19
	   1200, 1.500, 1, 30,  // 20
	   1260, 1.500, 0, 28,  // 21
	   1320, 1.500, 0, 28,  // 22
	   1380, 1.500, 1, 30, // 23
	   1440, 1.500, 1, 31  // 24
};

#define G_ACCELEROMETER (2048*4)

void SIM_get_accelerometer(I16S *x, I16S *y, I16S *z)
{
	double curr_minute = cling.time.local.hour * 60 + cling.time.local.minute;
	int i;
	double std;
	int integerStd, random_acc_x, random_acc_y, random_acc_z;

	for (i = 0; i <= MAXIMUM_MINUTE_ENTRIES; i++) {

		if (curr_minute < acc_sim_table[i * 4]) {
			std = acc_sim_table[(i - 1) * 4 + 1];
			//cling.touch.b_skin_touch_type = acc_sim_table[(i - 1) * 4 + 2];
			cling.touch.b_skin_touch = 1;
			cling.therm.current_temperature = acc_sim_table[(i - 1) * 4 + 3]*10;
			break;
		}
	}
	std *= (G_ACCELEROMETER<<1);
	integerStd = (int)std;

	random_acc_x = (rand() % integerStd) - (integerStd >> 1);
	random_acc_y = (rand() % integerStd) - (integerStd >> 1);
	random_acc_z = (rand() % integerStd) - (integerStd >> 1);

	LIS3DH_x += random_acc_x;
	LIS3DH_y += random_acc_y;
	LIS3DH_z += random_acc_z;

	if (LIS3DH_x > (G_ACCELEROMETER<<2))
		LIS3DH_x -= G_ACCELEROMETER*5;
	else if (LIS3DH_x < -(G_ACCELEROMETER << 2))
		LIS3DH_x += G_ACCELEROMETER * 5;
	if (LIS3DH_y > (G_ACCELEROMETER << 2))
		LIS3DH_y -= G_ACCELEROMETER * 5;
	else if (LIS3DH_y < -(G_ACCELEROMETER << 2))
		LIS3DH_y += G_ACCELEROMETER * 5;
	if (LIS3DH_z > (G_ACCELEROMETER << 2))
		LIS3DH_z -= G_ACCELEROMETER * 5;
	else if (LIS3DH_z < -(G_ACCELEROMETER << 2))
		LIS3DH_z += G_ACCELEROMETER * 5;

	*x = LIS3DH_x;
	*y = LIS3DH_y;
	*z = LIS3DH_z;
}
#endif
