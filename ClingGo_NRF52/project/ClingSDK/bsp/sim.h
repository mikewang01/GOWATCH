//
//  File: sim.h
//  
//  Description: the header contains all simulated activity info
//
//  Created on Jan 6, 2014
//
#ifndef __SIM_HEADER__
#define __SIM_HEADER__

typedef enum {
	SIM_ACTIVITY_WALKING,
	SIM_ACTIVITY_REST,
	SIM_ACTIVITY_RUNNING,
	SIM_ACTIVITY_SLEEP,
} SIM_ACTIVITY_TYPE;

//#define _SLEEP_SIMULATION_

//#define USING_VIRTUAL_ACTIVITY_SIM

//#define _ACTIVITY_SIM_BASED_ON_EPOCH_
//#define _REMINDER_SIMULATION_
//#define _LONG_TERM_TRACKING_

void SIM_init(void);
I16S SIM_get_current_activity(I8U type);
void SIM_setup_reminder(void);
void SIM_setup_idle_alert(void);
void SIM_get_accelerometer(I16S *x, I16S *y, I16S *z);

#endif  // __ACTIVITY_HEADER__
