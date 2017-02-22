/***************************************************************************//**

 * touch.h
 *
 ******************************************************************************/

#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "standards.h"

#define _ENABLE_TOUCH_

#define TOUCH_DEBOUNCING_TIME_PER_MINUTE 40 // 40 seconds for minute activity logging

enum {
	TOUCH_I2C_REG_CONTROL = 0,
	TOUCH_I2C_REG_VERSION,
	TOUCH_I2C_REG_GESTURE,
	TOUCH_I2C_REG_STATUS
};

typedef enum {
	TOUCH_POWER_HIGH_20MS,
	TOUCH_POWER_MIDIAN_200MS,
	TOUCH_POWER_LOW_2000MS,
	TOUCH_POWER_DEEP_SLEEP,
} TOUCH_POWER_MODE;

enum {
	TOUCH_NONE = 0,
	TOUCH_SWIPE_LEFT,
	TOUCH_SWIPE_RIGHT,
	TOUCH_FINGER_MIDDLE,
	TOUCH_FINGER_LEFT,
	TOUCH_FINGER_RIGHT,
	TOUCH_BUTTON_SINGLE,
	TOUCH_DOUBLE_TAP,
	TOUCH_BUTTON_PRESS_HOLD,
	TOUCH_BUTTON_PRESS_SOS,
	TOUCH_MAX
};

typedef enum {
	TOUCH_STATE_IDLE,
	TOUCH_STATE_MODE_SET,
	TOUCH_STATE_MODE_CONFIRMING,
} TOUCH_STATE;

typedef struct tagTOUCH_CTX {
	// Tap activities
	I32U tap_time_base;
	I8U tap_counts;
	BOOLEAN b_double_tap;
	
	// Gesture
	BOOLEAN b_valid_gesture;
	I8U gesture;
	
	// State machine
	TOUCH_STATE state;
		
	// Skin touch update
	BOOLEAN b_skin_touch;
	I8U skin_touch_time_per_minute;
	
} TOUCH_CTX;

void TOUCH_init(void);
I8U TOUCH_get_gesture_panel(void);
void TOUCH_gesture_check(void);
void TOUCH_power_set(TOUCH_POWER_MODE mode);
void TOUCH_power_mode_state_machine(void);
BOOLEAN TOUCH_new_gesture(void);
BOOLEAN TOUCH_is_skin_touched(void);
I8U TOUCH_get_skin_touch_time(void);

#endif // __TOUCH_H__
/** @} */
