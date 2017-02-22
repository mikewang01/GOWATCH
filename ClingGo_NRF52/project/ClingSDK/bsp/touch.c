///***************************************************************************//**
// * File touch.c
// *
// * Description: touch panel driver
// *
// ******************************************************************************/

#include "main.h"

//#ifndef _CLING_PC_SIMULATION_
//
//#include "uicoTouch.h"
//#endif

BOOLEAN TOUCH_new_gesture()
{
	return cling.touch.b_valid_gesture;
}

I8U TOUCH_get_gesture_panel()
{
	TOUCH_CTX *t = &cling.touch;

	if (t->b_double_tap) {
		t->tap_counts = 0;
		t->b_double_tap = FALSE;
		t->b_valid_gesture = FALSE;
		return TOUCH_DOUBLE_TAP;
	} else if (t->b_valid_gesture) {
		t->b_valid_gesture = FALSE;
		return t->gesture;
	}

	return TOUCH_NONE;
}

//void TOUCH_power_set(TOUCH_POWER_MODE mode)
//{
//#ifndef _CLING_PC_SIMULATION_
//#ifdef _ENABLE_TOUCH_
//	// Generate a pulse prior to I2C communication
//
//	if (mode == TOUCH_POWER_DEEP_SLEEP) {
//		UICO_set_power_mode(UICO_POWER_DEEP_SLEEP);
//	} else {
//		UICO_set_power_mode(UICO_POWER_FAST_SCAN);
//	}
//	N_SPRINTF("\n[TOUCH] power mode setting --- %d ---\n", mode);
//#endif
//#endif
//}
//
//void TOUCH_reset_tap_module()
//{
//	TOUCH_CTX *t = &cling.touch;
//
//	t->tap_counts = 0;
//	t->b_double_tap = FALSE;
//}
//
//void _tap_process(TOUCH_CTX *t, I32U t_curr)
//{
//		UI_ANIMATION_CTX *u = &cling.ui;
//
//#if 0
//		if (!LINK_is_authorized())
//			return;
//#endif
//
//		if ((u->frame_index >= UI_DISPLAY_CAROUSEL) && (u->frame_index <= UI_DISPLAY_CAROUSEL_END))
//			return;
//
//		if (cling.user_data.b_navigation_tapping) {
//			t->tap_counts ++;
//			t->tap_time_base = t_curr;
//
//			if (t->tap_counts >= 3) {
//
//				N_SPRINTF("[TOUCH] +++ double tapped, %d, %d", t->tap_counts, t->tap_time_base);
//				// Double tap - let's jump to icon page
//				//
//				t->b_double_tap = TRUE;
//			}
//		}
//}
//
//static BOOLEAN _touch_screen_activation()
//{
//	if (cling.user_data.b_screen_press_hold_1)
//		return TRUE;
//
//	if (cling.user_data.b_screen_press_hold_3)
//		return TRUE;
//
//	return FALSE;
//}
//
//static void _finger_down_processing(TOUCH_CTX *t, I8U op_detail, I32U t_curr)
//{
//	if (UI_is_idle()) {
//
//		if (!_touch_screen_activation())
//			return;
//	}
//
//	// finger down
//	if (op_detail == 0) {
//		t->gesture = TOUCH_FINGER_LEFT;
//		N_SPRINTF("[TOUCH] ------------ FINGER: left --------");
//		N_SPRINTF("Finger: left");
//	} else if (op_detail == 1) {
//		t->gesture = TOUCH_FINGER_MIDDLE;
//		N_SPRINTF("[TOUCH] ------------ FINGER: middle --------");
//		N_SPRINTF("Finger: middle");
//	} else if (op_detail == 2) {
//		t->gesture = TOUCH_FINGER_RIGHT;
//		N_SPRINTF("[TOUCH] ------------ FINGER: right --------");
//		N_SPRINTF("Finger: right");
//	}
//
//	// Make sure OLED display panel is faced up.
////		if (LINK_is_authorized() && (cling.activity.z_mean < 0)) {
//	if (LINK_is_authorized()) {
//
//		N_SPRINTF("[TOUCH] ------------ TURN ON SCREEN --------");
//
//		// Turn on OLED panel
//		if (!OLED_set_panel_on()) {
//			t->b_valid_gesture = TRUE;
//
//			// Tap detection
//			_tap_process(t, t_curr);
//		}
//
//		cling.ui.true_display = TRUE;
//
//		// Update touch event time
//		cling.ui.touch_time_stamp = CLK_get_system_time();
//
//		if (UI_is_idle()) {
//			// UI initial state, a glance of current clock
//			UI_switch_state(UI_STATE_CLOCK_GLANCE, 0);
//			N_SPRINTF("[TOUCH] set UI: clock glance (finger down)");
//		} else {
//			//
//			UI_switch_state(UI_STATE_TOUCH_SENSING, 0);
//			N_SPRINTF("[TOUCH] set UI: touch sensing (finger down)");
//		}
//	}
//}
//
//static void _swipe_processing(TOUCH_CTX *t, I8U op_detail)
//{
//	if (UI_is_idle()) {
//
//		if (!_touch_screen_activation())
//			return;
//	}
//
//	// Swipe
//	if (op_detail == 0) {
//		t->gesture = TOUCH_SWIPE_LEFT;
//		N_SPRINTF("[TOUCH] ------------ SWIPE: left --------");
//		N_SPRINTF("SWIPE: left");
//	} else if (op_detail == 1) {
//		t->gesture = TOUCH_SWIPE_RIGHT;
//		N_SPRINTF("[TOUCH] ------------ SWIPE: right --------");
//		N_SPRINTF("SWIPE: Right");
//	} else {
//		return;
//	}
//
////		if (LINK_is_authorized() && (cling.activity.z_mean < 0)) {
//	if (LINK_is_authorized()) {
//
//		N_SPRINTF("[TOUCH] ------------ TURN ON SCREEN --------");
//
//		// Turn on OLED panel
//		if (!OLED_set_panel_on()) {
//			t->b_valid_gesture = TRUE;
//
//			N_SPRINTF("[TOUCH] ------------ VALID gesture --------");
//
//		}
//
//		cling.ui.true_display = TRUE;
//
//		// touch time update
//		cling.ui.touch_time_stamp = CLK_get_system_time();
//
//		if (UI_is_idle()) {
//			// UI initial state, A glance of current time
//			UI_switch_state(UI_STATE_CLOCK_GLANCE, 0);
//			N_SPRINTF("[TOUCH] set UI: clock glance (swipe)");
//		} else {
//			UI_switch_state(UI_STATE_TOUCH_SENSING, 0);
//			N_SPRINTF("[TOUCH] set UI: touch sensing (swipe)");
//		}
//	}
//}
//
//static void _skin_touch_processing(TOUCH_CTX *t, BOOLEAN b_skin_detected)
//{
//		// skin touch
//		if (b_skin_detected) {
//			t->b_skin_touch = TRUE;
//			N_SPRINTF("[TOUCH] ------------ SKIN: ON --------");
//			N_SPRINTF("SKIN: ON");
//		} else {
//			t->b_skin_touch = FALSE;
//			N_SPRINTF("[TOUCH] ------------ SKIN: OFF --------");
//			N_SPRINTF("SKIN: OFF");
//		}
//}
//
//void TOUCH_gesture_check(void)
//{
//#ifndef _CLING_PC_SIMULATION_
//  I8U int_pin;
//	I8U reg_value;
//	I8U op_code, op_detail;
//	TOUCH_CTX *t = &cling.touch;
//	I32U t_curr = CLK_get_system_time();
//
//	// 1.5 Seconds expiration of double tapping
//	if (t_curr > (t->tap_time_base + 1500)) {
//		if (t->tap_counts) {
//			N_SPRINTF("[TOUCH] +++ clear tap, %d, %d, %d", t->tap_counts, t->tap_time_base, t_curr);
//			t->tap_counts = 0;
//		}
//		t->b_double_tap = FALSE;
//	}
//
//	// Check to see if the touch Interrupt pin is pulled down
//	int_pin = nrf_gpio_pin_read(GPIO_TOUCH_INT);
//
//	if (int_pin)
//		return;
//
//	// get gesture
//	//
//	N_SPRINTF("[TOUCH] Interrupt status (%d): %d", CLK_get_system_time(), int_pin);
//
//	//
//	// Gesture byte definition -
//	//
//	// Bit   7: unused
//	//
//	// Bit   6: new gesture
//	//
//	// Bit 3-5: operation code
//	//
//	//          000: no op
//	//          001: swipe
//	//          010: finger
//	//          101: skin touch + swipe
//	//          110: skin touch + finger
//	//
//	// Bit 0-2: detail
//	//
//	reg_value =	UICO_main();
//
//	// Invalid gesture, go return directly
//	if (reg_value == 0xff) {
//		return;
//	}
//
//	op_code = (reg_value>>3) & 0x07;
//	op_detail = reg_value & 0x07;
//
//	N_SPRINTF("[TOUCH] reg value: %02x, op: %d, detail: %d", reg_value, op_code, op_detail);
//
//	// skin touch processing
//	if (op_code & 0x04) {
//		_skin_touch_processing(t, TRUE);
//	} else {
//		_skin_touch_processing(t, FALSE);
//	}
//
//	// Make sure if we get a different gesture
//	if (!(reg_value & 0x40)) {
//		return;
//	}
//
//	if (op_code & 0x01) {
//
//		// Swipe processing
//		_swipe_processing(t, op_detail);
//
//	}
//
//	if (op_code & 0x02) {
//
//		// finger down processing
//		_finger_down_processing(t, op_detail, t_curr);
//
//	}
//
//#endif
//}
//
//void TOUCH_init(void)
//{
//#ifndef _CLING_PC_SIMULATION_
//	// Initialize skin touch states
//	UICO_init();
//	//uico_touch_ic_floating_calibration_start();
//	Y_SPRINTF("[TOUCH] skin touched: %d", cling.touch.b_skin_touch);
//
//#endif
//}
//
//I8U TOUCH_get_skin_touch_time()
//{
//	I8U touch_time = cling.touch.skin_touch_time_per_minute;
//
//	cling.touch.skin_touch_time_per_minute = 0;
//	return touch_time;
//}
//
//BOOLEAN TOUCH_is_skin_touched(void)
//{
//	if (BATT_is_charging())
//		return FALSE;
//
//	return cling.touch.b_skin_touch;
//}
