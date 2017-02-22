//
//  File: homekey.h
//
//  Header file for home key processing
//
#ifndef _HOMEKEY_HEADER_
#define _HOMEKEY_HEADER_


#define HOMEKEY_TIME_RATIO_SH 0

#define DOUBLE_CLICK_TIME_MAX	100
#define DOUBLE_CLICK_TIME_MIN	80

#define CLICK_HOLD_TIME_LONG 1500
#define CLICK_HOLD_TIME_SHORT 1000 // 30 -- remove debouncing for now

#define CLICK_HOLD_TIME_SOS  4000

#define HOMEKEY_CLICK_DEBOUNCE		10 // 100 milli-second for debouncing
#define DELAY_SINGLE_CLICK_DECLARATION 200

#define HOMEKEY_ON_HOLDING          1000

typedef struct tagHOMEKEY_CLICK_STAT {
	int t;
} HOMEKEY_CLICK_STAT;

// BUTTON STATUS
enum {
	ON_CLICK,
	OFF_CLICK,
};

typedef enum {
	CLICK_NONE,			// No event
	CLICK_HOLD,			// click and hold
	SINGLE_CLICK,		// click 
	DOUBLE_CLICK,		// double click

} CLICK_SUB_EVENTS;


void HOMEKEY_click_init(void);
BOOLEAN HOMEKEY_check_on_click_event(void);
void HOMEKEY_check_sim(void);
void home_key_long_pressed_detected(void);
#endif
