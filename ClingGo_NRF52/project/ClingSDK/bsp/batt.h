/***************************************************************************//**
 * File batt.h
 * 
 * Description: Power measurement and management header
 *
 *
 ******************************************************************************/

#ifndef __BATT_HEADER__
#define __BATT_HEADER__

#define BATTERY_MEASURE_INTERVAL 600   // 600 seconds (10 MINUTES)

#define BATTERY_LOW_PERCENTAGE 5

#define SLEEP_ACTIVE_SECONDS_THRESHOLD 60 // 60 seconds
#define SLEEP_STEP_ACCUMULATED_THRESHOLD 10 // 10 STEPS

#define BATTERY_SYSTEM_UNAUTH_POWER_DOWN 300

#define BATTERY_MAXIMUM_CHARGING_TIME 5400 // 5400 Seconds -> 1.5 hours
#define BATTERY_75_PERC_FULL_CHARGE 3600 // 3600 Seconds -> 1.0 hour
#define BATTERY_MINIMUM_CHARGING_TIME 2700 // 2700 Seconds -> 45 minutes
#define BATTERY_CHARGING_SPEED 38000 // 38 Seconds one percent (charging speed)
enum {
	CHARGER_REMOVED,
	CHARGER_FULL,
	CHARGER_IN_CHARGING,
};

enum {
	CHARGER_ADC_IDLE,
	CHARGER_ADC_MEASURING,
	CHARGER_ADC_ACQUIRED,
};

typedef struct tagBATT_CTX {
	I32U charging_timebase;			// Charging percentage update -- 1 percent every 100 seconds, complete in 7200 seconds
	I16U charging_overall_time;
	I16U volts_reading;
	I16U level_update_timebase;		// Battery percentage update timer -- 15 seconds
	I16U shut_down_time;
	I8U charging_state;
	I8U adc_state;
	I8U state_switching_duration;
	I8U toggling_number;
	I8U battery_measured_perc;
	BOOLEAN b_initial_measuring;
	BOOLEAN b_no_batt_restored;
	
	// charging detection
	I8U non_charging_accumulated_active_sec;
	I8U non_charging_accumulated_steps;
	
	// Charging status
	BOOLEAN charging_connected;
	
} BATT_CTX;

I8U BATT_get_level(void);
BOOLEAN BATT_is_charging(void);
void BATT_init(void);
void BATT_monitor_state_machine(void);
void BATT_start_first_measure(void);
BOOLEAN BATT_is_low_battery(void);
void BATT_charging_update_steps(I8U steps);
void BATT_exit_charging_state(I8U sec);
void BATT_update_charging_time(I8U tick_in_s);
BOOLEAN BATT_charging_det_for_sleep(void);
BOOLEAN BATT_device_unauthorized_shut_down(void);
void BATT_charging_init(void);
void BATT_charging_deinit(void);
void BATT_interrupt_process(BOOLEAN b_init);
void BATT_monitor_process(void);
#endif // __BATT_HEADER__
