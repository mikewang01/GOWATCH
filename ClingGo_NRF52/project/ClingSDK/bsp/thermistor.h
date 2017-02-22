#ifndef __THERMISTOR_H__
#define __THERMISTOR_H__

#include "standards.h"

#define THERMISTOR_POWER_SETTLE_TIME_MS 15
//  
//  #define THERM_REF_VOLT         3.1         // voltage
//  #define THERM_REF_RESITANCE    10000       // 10K ohm
//  #define THERM_ADC_REF_VOLT     1.2         // ADC internal reference voltage
//  
// 
// below Steinhart-Hart equation coefficients a, b, c are
// calculated by 3-point-couple way;
// the three temperature-resistance points are as below
// NTC Model Code:
//   10k  8307 3478 3450 3492 B57540G1103+000
//       30 Celsius, 8281.8 ohm
//       35 Celsius, 6895.4 ohm
//       40 Celsius, 5770.3 ohm
// 
// 
// #define    A_Coef    8.5186e-4f
// #define    B_Coef    2.6042e-4f
// #define    C_Coef    1.3265e-7f
// 


//  #define  TABLE_SIZE    140    // 393
//  #define  BOTTOM_ADC    432    // 232
//  #define  TEMP_OFFSET   360

#define  TABLE_SIZE    155
#define  BOTTOM_ADC    322
#define  TEMP_OFFSET   325


#define TEMPERATURE_AVERAGE_COUNT 8

typedef enum {
	THERMISTOR_STAT_IDLE,                                // <----------------------- 0
	THERMISTOR_STAT_DUTY_ON,                             // <----------------------- 1
	THERMISTOR_STAT_MEASURING,                           // <----------------------- 2
	THERMISTOR_STAT_DUTY_OFF,                            // <----------------------- 3
} THERMISTOR_STATES;

typedef struct tagTHERMISTOR_CTX {
  I32U      measure_timebase;
	I16U      therm_volts_reading;
	I16U      power_volts_reading;

	// Thermistor state machine
	THERMISTOR_STATES state;

	// Thermistor power on timeing
	I32U power_on_timebase;

	// Temperature reading
	I16S current_temperature;

	// Temperature ADC sample
	BOOLEAN b_start_adc;
	
	// NRF51822 ADC module calibration
//I8S   m_gain_error;
//I8S   m_offset_error;
} THERMISTOR_CTX;

void THERMISTOR_init(void);
void THERMISTOR_set_state(THERMISTOR_STATES new_state);
void THERMISTOR_state_machine(void);
BOOLEAN THERMISTOR_is_the_state(THERMISTOR_STATES state_to_check);
BOOLEAN THERMISTOR_switch_to_duty_on_state(void);

BOOLEAN _is_user_viewing_skin_temp(void);

#endif




