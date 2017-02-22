//
//#include "main.h"
//
//#define SKIN_TEMPERATURE_MEASURING_PERIOD_FOREGROUND  2
//#define SKIN_TEMPERATURE_MEASURING_PERIOD_BACKGROUND  900
//
//static void _calc_temperature()
//{
//	THERMISTOR_CTX *t = &cling.therm;
//	I32S volt_diff;
//	I32S resistance, resistance_t;
//	I16S i, temperature;
//
////#define _HUNDRED_THOUSAND_OHM_
//
//#ifdef _HUNDRED_THOUSAND_OHM_
//
//#define    FIRST_REST    127080                // resistance at 20.0 Celsius Degreee
//#define    FIRST_TEMP    200                   // represent 20.0 Celsius Degreee
//#define    TAB_LENGTH    299
//#define    NOMINAL_RESISTANCE      100000
//#define    TEMPERATURE_STEP        5
//
//	const I8U resistance_inc[TAB_LENGTH] = {
//    122, 123, 122, 121, 120, 120, 120, 118, 118, 118, 116, 116, 116, 115, 114, 114,
//    113, 112, 112, 111, 110, 110, 110, 109, 108, 108, 107, 106, 106, 106, 104, 105,
//    103, 104, 102, 102, 102, 101, 100, 100,  99,  99,  99,  97,  98,  96,  97,  95,
//     96,  94,  95,  93,  94,  92,  93,  92,  91,  91,  90,  90,  89,  89,  89,  88,
//     87,  87,  87,  86,  86,  85,  85,  84,  84,  84,  83,  83,  82,  82,  81,  81,
//     80,  81,  79,  79,  79,  79,  78,  78,  77,  77,  76,  76,  76,  75,  75,  74,
//     75,  73,  74,  73,  72,  72,  72,  72,  71,  71,  70,  70,  70,  69,  69,  69,
//     68,  68,  67,  68,  66,  67,  66,  66,  66,  65,  65,  64,  64,  64,  64,  63,
//     63,  63,  62,  62,  61,  62,  61,  61,  60,  60,  60,  59,  60,  58,  59,  58,
//     58,  58,  58,  57,  57,  56,  57,  56,  55,  56,  55,  55,  54,  55,  54,  54,
//     53,  54,  53,  52,  53,  52,  52,  52,  51,  51,  51,  51,  51,  50,  50,  49,
//     50,  49,  49,  49,  48,  49,  48,  47,  48,  47,  47,  47,  47,  46,  46,  46,
//     46,  45,  46,  45,  44,  45,  44,  45,  44,  43,  44,  43,  43,  43,  43,  42,
//     42,  43,  41,  42,  42,  41,  41,  41,  40,  41,  40,  40,  40,  40,  39,  39,
//     40,  38,  39,  39,  38,  38,  38,  38,  38,  37,  37,  37,  37,  37,  37,  36,
//     36,  36,  36,  36,  35,  36,  35,  35,  35,  34,  35,  34,  35,  34,  33,  34,
//     34,  33,  33,  33,  33,  33,  33,  32,  32,  33,  32,  31,  32,  32,  31,  31,
//     31,  31,  31,  31,  30,  31,  30,  30,  30,  30,  29,  30,  29,  30,  29,  29,
//     29,  28,  29,  28,  29,  28,  28,  28,  28,  27,  28};
//
//#else
//
//#define    FIRST_REST    17926                 // resistance at 10.0 Celsius Degreee
//#define    FIRST_TEMP    100                   // represent 10.0 Celsius Degreee
//#define    TAB_LENGTH    399
//#define    NOMINAL_RESISTANCE      10000
//#define    TEMPERATURE_STEP        1
//
//	const I8U resistance_inc[TAB_LENGTH] = {
//    54, 74, 74, 74, 74, 73, 73, 72, 72, 72, 52, 71, 71, 71, 70, 69,
//    70, 69, 69, 68, 49, 68, 67, 68, 66, 67, 66, 66, 66, 65, 49, 64,
//    65, 64, 64, 63, 63, 63, 63, 62, 47, 61, 62, 61, 61, 60, 61, 60,
//    60, 59, 45, 59, 59, 58, 58, 58, 58, 57, 57, 57, 45, 56, 56, 56,
//    55, 56, 55, 54, 55, 54, 42, 54, 53, 54, 53, 52, 53, 52, 52, 52,
//    41, 52, 51, 51, 50, 51, 50, 50, 50, 49, 39, 49, 49, 49, 48, 48,
//    48, 48, 48, 47, 38, 47, 47, 47, 46, 46, 46, 45, 46, 45, 37, 45,
//    44, 45, 44, 44, 44, 43, 44, 43, 36, 43, 43, 42, 42, 42, 42, 42,
//    42, 41, 35, 41, 41, 41, 40, 41, 40, 40, 39, 40, 35, 39, 39, 39,
//    39, 39, 38, 38, 38, 38, 34, 38, 37, 37, 37, 37, 37, 36, 37, 36,
//    32, 36, 36, 36, 35, 35, 36, 34, 35, 35, 33, 34, 35, 34, 34, 33,
//    34, 33, 34, 33, 32, 33, 33, 32, 33, 32, 32, 32, 32, 32, 30, 32,
//    31, 31, 31, 31, 31, 31, 31, 30, 30, 30, 30, 30, 30, 30, 29, 30,
//    29, 29, 28, 29, 29, 28, 29, 28, 29, 28, 28, 28, 28, 27, 28, 28,
//    27, 27, 27, 27, 27, 27, 27, 27, 26, 26, 27, 26, 26, 26, 25, 26,
//    26, 25, 25, 26, 25, 25, 25, 25, 24, 25, 25, 24, 25, 24, 24, 24,
//    24, 24, 23, 24, 25, 23, 23, 24, 23, 23, 23, 23, 22, 23, 23, 23,
//    22, 22, 22, 23, 22, 21, 22, 22, 23, 22, 21, 22, 21, 21, 21, 21,
//    21, 21, 22, 21, 20, 21, 20, 20, 21, 20, 20, 20, 22, 20, 20, 20,
//    19, 20, 19, 19, 20, 19, 22, 19, 19, 19, 19, 19, 18, 19, 19, 18,
//    21, 18, 18, 19, 18, 18, 18, 17, 18, 18, 20, 18, 17, 18, 17, 18,
//    17, 17, 17, 17, 20, 16, 17, 17, 17, 16, 17, 16, 17, 16, 19, 17,
//    16, 16, 16, 16, 16, 16, 15, 16, 19, 16, 15, 16, 15, 15, 16, 15,
//    15, 15, 18, 15, 15, 15, 15, 15, 14, 15, 14, 15, 18, 14, 14, 14,
//    15, 14, 14, 14, 14, 14, 17, 14, 14, 14, 13, 14, 14, 13, 14};
//
//#endif    // _HUNDRED_THOUSAND_OHM_
//
//		// calculate resistance of ntc.
//	volt_diff  = ( t->power_volts_reading - t->therm_volts_reading ) * NOMINAL_RESISTANCE;
//	resistance = 0;
//
//  for (resistance=0; (volt_diff>0) ; resistance++) {
//		volt_diff -= t->therm_volts_reading;
//	}
//	N_SPRINTF( "resistance: %d ", resistance);
//
//	// calculate temperature.
//	resistance_t = FIRST_REST;
//	for (i=0; i <(TAB_LENGTH); i++) {
//		if (resistance>=resistance_t) {
//			break;
//		}
//
//		resistance_t -= (resistance_inc[i] * TEMPERATURE_STEP);
//	}
//
//	temperature = FIRST_TEMP + i;
//  N_SPRINTF( "therm: %d  power: %d  temp: %d", t->therm_volts_reading, t->power_volts_reading, temperature);
//
//	t->current_temperature = temperature;
//}
//
//void THERMISTOR_init(void)
//{
//	THERMISTOR_CTX *t = &cling.therm;
//
//	// init thermistor measue time base
//	t->measure_timebase = cling.time.system_clock_in_sec;
//
//	t->state = THERMISTOR_STAT_IDLE;
//	t->therm_volts_reading = 0;
//	t->power_volts_reading = 0;
//
//	t->current_temperature = 310;
//}
//
//BOOLEAN _is_user_viewing_skin_temp()
//{
//	if (UI_is_idle()) {
//		return FALSE;
//	}
//
//	if (cling.ui.frame_index != UI_DISPLAY_VITAL_SKIN_TEMP) {
//		return FALSE;
//	}
//
//	return TRUE;
//}
//
//void THERMISTOR_state_machine()
//{
//#ifndef _CLING_PC_SIMULATION_
//	THERMISTOR_CTX *t = &cling.therm;
//	I32U t_curr, t_diff;
//
//	if (OTA_if_enabled())
//		return;
//
//	t_curr = CLK_get_system_time();
//
//	N_SPRINTF("[THERM] state: %d", t->state);
//
//	switch (t->state) {
//
//		case THERMISTOR_STAT_IDLE:
//		{
//			// Jump to duty off to check if measuring time is up.
//			t->state = THERMISTOR_STAT_DUTY_OFF;
//			N_SPRINTF("[THERM] idle");
//			break;
//		}
//	  case THERMISTOR_STAT_DUTY_ON:
//		{
//			// Start 50ms operation clock
//		  RTC_start_operation_clk();
//			GPIO_therm_power_on();
//			t->measure_timebase = cling.time.system_clock_in_sec;
//			t->state = THERMISTOR_STAT_MEASURING;
//			t->power_on_timebase = t_curr;
//			// Configure ADC
//			GPIO_therm_adc_config();
//			N_SPRINTF("[THERM] duty on");
//			break;
//		}
//		case THERMISTOR_STAT_MEASURING:
//		{
//			// add 15 milli-second delay to have power to settle down.
//			//
//			// The actual measuring takes about 7 ms to finish
//			if (t_curr > (t->power_on_timebase + THERMISTOR_POWER_SETTLE_TIME_MS)) {
//				N_SPRINTF("[THERM] therm measuring at %d ", CLK_get_system_time());
//				cling.therm.therm_volts_reading = nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_6);
//				N_SPRINTF("[THERM] power measuring ");
//				cling.therm.power_volts_reading = nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_2);
//				//t->state = THERMISTOR_STAT_START_THERM_PIN_MEASURING;
//				t->state = THERMISTOR_STAT_DUTY_OFF;
//				GPIO_therm_power_off();
//				t->b_start_adc = TRUE;
//				N_SPRINTF("[THERM] adc measured at(%d): %d,%d", CLK_get_system_time(), cling.therm.therm_volts_reading, cling.therm.power_volts_reading);
//  			_calc_temperature();
//			}
//			N_SPRINTF("[THERM] therm pin turn on adc");
//
//			break;
//		}
//	  case THERMISTOR_STAT_DUTY_OFF:
//		{
//			// If skin touch is positive, go ahead to measure PPG
//			if (!TOUCH_is_skin_touched())
//				break;
//
//	  	t_diff = cling.time.system_clock_in_sec - t->measure_timebase;
//
//	  	if (_is_user_viewing_skin_temp()) {
//					N_SPRINTF("[THERM] duty off view -> %d, %d", t_diff, cling.user_data.skin_temp_day_interval);
//	  		if (t_diff > SKIN_TEMPERATURE_MEASURING_PERIOD_FOREGROUND) {
//	  			t->state = THERMISTOR_STAT_DUTY_ON;
//					N_SPRINTF("[THERM] duty off -> ON, view screen");
//	  		}
//	  	} else {
//				N_SPRINTF("[THERM] duty off normal-> %d, %d", t_diff, cling.user_data.skin_temp_day_interval);
//    		if ( t_diff >  SKIN_TEMPERATURE_MEASURING_PERIOD_BACKGROUND ) {
//					t->state = THERMISTOR_STAT_DUTY_ON;
//					N_SPRINTF("[THERM] duty off -> ON, normal: %d", t_diff);
//	  		}
//	  	}
//	  	break;
//		}
//		default:
//			break;
//	}
//#endif
//}
//
//BOOLEAN THERMISTOR_is_the_state(THERMISTOR_STATES state_to_check)
//{
//	THERMISTOR_CTX *t = &cling.therm;
//
//	if (t->state == state_to_check)
//		return TRUE;
//	else
//		return FALSE;
//}
//
//void THERMISTOR_set_state(THERMISTOR_STATES new_state)
//{
//	cling.therm.state = new_state;
//}
//
//
//BOOLEAN THERMISTOR_switch_to_duty_on_state()
//{
//	return TRUE;
//}
//
//
