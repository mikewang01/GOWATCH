///***************************************************************************//**
// * File: ui.c
// *
// * Description: master UI controller
// *
// ******************************************************************************/
//
#include "main.h"
#include "assets.h"
#include "font.h"

#include "ui_matrix.h"
//#include "sys_power_mgr.h"
//#define RENDERING_MAX_FRAMES     11

static OS_TIMER ui_timer;

extern void UI_rendering_timer_callback(OS_TIMER tmr);
extern void UI_task_notify_rendering();
extern void SYSCLK_restart_timer(OS_TIMER timer, I32U period_in_ms);
extern OS_TIMER SYSCLK_get_timer(I32U period_in_ms, TimerCallbackFunction_t callback);
extern uint16_t oled_get_x_solution(void);
extern uint16_t oled_get_y_solution(void);
extern void UI_task_notify_get_balance(void);
extern void ALERT_task_reminder_notify(void);
extern void ALERT_task_notific_notify(void);
void UI_init()
{
        // Turn on OLED panel
        OLED_set_panel_on();

        // UI initial state
        cling.ui.state = UI_STATE_HOME; //UI_STATE_CLING_START;

        // UI prefer state
        cling.ui.prefer_state = UI_STATE_HOME;
        cling.ui.true_display = TRUE;

        // Start UI timer !
        if (!ui_timer) {
                ui_timer = SYSCLK_get_timer(10, UI_rendering_timer_callback);
                if (!ui_timer) {
                        N_SPRINTF("Reminder: Timer Started Failed ");
                        cling.ui.state = UI_STATE_IDLE;
                }
        }
        else {
                SYSCLK_restart_timer(ui_timer, 10);
        }

        N_SPRINTF("UI task: Initialize UI module: %d", cling.time.time_since_1970);
}

static void _set_animation(I8U mode, I8U dir)
{
        UI_ANIMATION_CTX *u = &cling.ui;

        u->state = UI_STATE_ANIMATING;

        u->animation_index = 0;
        u->direction = dir;
        u->frame_prev_idx = u->frame_index;
        u->animation_mode = mode;
}

//static I8U _get_max_detail_depth()
//{
//	char data[128];
//  I8U max_frame_num;
//
//	NOTIFIC_get_app_message_detail(cling.ui.app_notific_index, data);
//
//	if (data[0] == 0)
//		return 0;
//
//	max_frame_num = FONT_get_string_display_depth(data);
//
//	return max_frame_num;
//}
//
//static void _update_horizontal_app_notific_index(UI_ANIMATION_CTX *u, BOOLEAN b_up)
//{
//	I8U index = u->frame_prev_idx;
//	I8U max_frame_num=0;
//
//	if (index == UI_DISPLAY_SMART_DETAIL_NOTIF) {
//		return;
//	} else if (index == UI_DISPLAY_SMART_APP_NOTIF) {
//#ifdef _ENABLE_ANCS_
//		max_frame_num = NOTIFIC_get_message_total();
//#endif
//		if (max_frame_num == 0) {
//			u->app_notific_index = 0;
//			return;
//		}
//
//		// Update App notific index
//		if (b_up) {
//			u->app_notific_index++;
//
//			if (u->app_notific_index >= max_frame_num) {
//				u->app_notific_index = 0;
//			}
//
//		} else {
//			if (u->app_notific_index == 0) {
//				u->app_notific_index = max_frame_num - 1;
//			} else {
//				u->app_notific_index --;
//			}
//		}
//	} else {
//		u->app_notific_index = 0;
//	}
//}
//
__attribute__((unused)) static void _update_vertical_index(UI_ANIMATION_CTX *u, BOOLEAN b_up)
{
//	I8U index = u->frame_prev_idx;
//	I8U max_frame_num;
//
//	if (index == UI_DISPLAY_HOME) {
//		max_frame_num = FRAME_DEPTH_CLOCK;
//	} else if (index == UI_DISPLAY_TRACKER_STEP) {
//		max_frame_num = FRAME_DEPTH_STEP;
//	} else if (index == UI_DISPLAY_TRACKER_SLEEP) {
//		max_frame_num = FRAME_DEPTH_SLEEP;
//	} else if (index == UI_DISPLAY_TRACKER_DISTANCE) {
//		max_frame_num = FRAME_DEPTH_DISTANCE;
//	} else if (index == UI_DISPLAY_TRACKER_CALORIES) {
//		max_frame_num = FRAME_DEPTH_CALORIES;
//	} else if (index == UI_DISPLAY_SMART_REMINDER) {
//		if (cling.reminder.ui_alarm_on)
//			return;
//		max_frame_num = cling.reminder.total;
//		N_SPRINTF("[UI] max reminder num: %d", max_frame_num);
//	} else if (index == UI_DISPLAY_SMART_WEATHER) {
//		max_frame_num = FRAME_DEPTH_WEATHER;
//	} else {
//		max_frame_num = 0;
//	}
//
//	if (max_frame_num == 0) {
//		u->vertical_index = 0;
//		return;
//	}
//
//	// Update vertical index
//	if (b_up) {
//		u->vertical_index++;
//
//		if (u->vertical_index >= max_frame_num) {
//			u->vertical_index = 0;
//		}
//
//	} else {
//		if (u->vertical_index == 0) {
//			u->vertical_index = max_frame_num - 1;
//		} else {
//			u->vertical_index --;
//		}
//	}
//
//	// Read out reminder index from Flash
//	if (index == UI_DISPLAY_SMART_REMINDER) {
//		cling.ui.vertical_index = REMINDER_get_time_at_index(cling.ui.vertical_index);
//		N_SPRINTF("[UI] new vertical index(Reminder): %d", cling.ui.level_1_index);
//	}
//
//	if (index == UI_DISPLAY_HOME) {
//		cling.ui.clock_orientation ++;
//		if (cling.ui.clock_orientation >= max_frame_num) {
//			cling.ui.clock_orientation = 0;
//		}
//	}
//
//	N_SPRINTF("[UI] new vertical index: %d", cling.ui.level_1_index);
}

static void _operation_mode_switch(I8U gesture)
{
//	UI_ANIMATION_CTX *u = &cling.ui;
//	I8U index = u->frame_prev_idx;
//
//	if (gesture == TOUCH_FINGER_RIGHT){
//
//		if (index == UI_DISPLAY_SMART_INCOMING_CALL) {
//
//			// Stop vibration in incoming call state
//			cling.notific.state = NOTIFIC_STATE_IDLE;
//			GPIO_vibrator_set(FALSE);
//			return;
//		}
//
//		if (index == UI_DISPLAY_STOPWATCH_START) {
//			cling.activity.workout_time_stamp_start = CLK_get_system_time();
//			cling.activity.workout_time_stamp_stop = cling.activity.workout_time_stamp_start;
//			cling.activity.workout_Calories_start = cling.activity.day.calories;
//			cling.activity.workout_Calories_acc = 0;
//			cling.activity.b_workout_active = TRUE;
//			return;
//		}
//
//		if (index == UI_DISPLAY_STOPWATCH_STOP) {
//			cling.activity.workout_time_stamp_stop = CLK_get_system_time();
//			cling.activity.workout_type = WORKOUT_NONE;
//			cling.activity.workout_place = WORKOUT_PLACE_NONE;
//			cling.activity.b_workout_active = FALSE;
//			return;
//		}
//	}
//
//	if (gesture == TOUCH_FINGER_RIGHT) {
//
//		if (index == UI_DISPLAY_WORKOUT_INDOOR) {
//			cling.activity.workout_place = WORKOUT_PLACE_INDOOR;
//			return;
//		}
//
//		if (index == UI_DISPLAY_WORKOUT_OUTDOOR) {
//			cling.activity.workout_place = WORKOUT_PLACE_OUTDOOR;
//			return;
//		}
//
//		if (index == UI_DISPLAY_WORKOUT_WALKING) {
//			cling.activity.workout_type = WORKOUT_WALK;
//			return;
//		}
//		if (index == UI_DISPLAY_WORKOUT_RUNNING) {
//			cling.activity.workout_type = WORKOUT_RUN;
//			return;
//		}
//		if (index == UI_DISPLAY_WORKOUT_ROW) {
//			cling.activity.workout_type = WORKOUT_ROWING;
//			return;
//		}
//		if (index == UI_DISPLAY_WORKOUT_ELLIPTICAL) {
//			cling.activity.workout_type = WORKOUT_ELLIPTICAL;
//			return;
//		}
//		if (index == UI_DISPLAY_WORKOUT_STAIRS) {
//			cling.activity.workout_type = WORKOUT_STAIRS;
//			return;
//		}
//		if (index == UI_DISPLAY_WORKOUT_CYCLING) {
//			cling.activity.workout_type = WORKOUT_CYCLING;
//			return;
//		}
//		if (index == UI_DISPLAY_WORKOUT_PILOXING) {
//			cling.activity.workout_type = WORKOUT_PILOXING;
//			return;
//		}
//		if (index == UI_DISPLAY_WORKOUT_AEROBIC) {
//			cling.activity.workout_type = WORKOUT_AEROBIC;
//			return;
//		}
//		if (index == UI_DISPLAY_WORKOUT_OTHERS) {
//			cling.activity.workout_type = WORKOUT_OTHER;
//			return;
//		}
//	}
}

__attribute__((unused)) static BOOLEAN _ui_vertical_animation(I8U index)
{
//	if ((index >= UI_DISPLAY_TRACKER_STEP) && (index <= UI_DISPLAY_TRACKER_CALORIES)) {
//		return TRUE;
//	}
//
//	if  (index == UI_DISPLAY_SMART_REMINDER) {
//		return TRUE;
//	} else if (index == UI_DISPLAY_SMART_WEATHER) {
//		return TRUE;
//	} else if (index == UI_DISPLAY_SMART_DETAIL_NOTIF) {
//		return TRUE;
//	} else {
//		return FALSE;
//	}
        return FALSE;
}

__attribute__((unused)) static void _update_notific_detail_index(UI_ANIMATION_CTX *u, BOOLEAN b_left)
{
//	I8U max_frame_num;
//
//	if (u->frame_index == UI_DISPLAY_SMART_DETAIL_NOTIF) {
//		max_frame_num = _get_max_detail_depth();
//
//		N_SPRINTF("[UI] max frame num : %d, notific detail index: %d", max_frame_num, u->notif_detail_index);
//
//		// Update vertical Notific detail index
//		if (b_left) {
//			u->notif_detail_index++;
//
//			if (u->notif_detail_index >= max_frame_num) {
//				u->notif_detail_index = 0;
//			}
//
//		} else {
//			if (u->notif_detail_index == 0) {
//				u->notif_detail_index = max_frame_num - 1;
//			} else {
//				u->notif_detail_index --;
//			}
//		}
//
//	}
}

//static void _perform_ui_with_a_swipe(UI_ANIMATION_CTX *u, I8U gesture)
//{
//	const I8U *p_matrix = NULL;
//
//	// Erase "more" icon
//	u->b_detail_page = FALSE;
//
//	// Make sure the UI frame can take swipe gesture
//	if (ui_gesture_constrain[u->frame_index] & UFG_SWIPE_PANNING) {
//		if (gesture == TOUCH_SWIPE_LEFT) {
//			p_matrix = ui_matrix_swipe_left;
//			if (u->frame_index == UI_DISPLAY_SMART_DETAIL_NOTIF)
//				_set_animation(ANIMATION_PANNING_IN, TRANSITION_DIR_LEFT);
//			else
//				_set_animation(ANIMATION_PANNING_OUT, TRANSITION_DIR_LEFT);
//
//			// Update frame index
//			u->frame_index = p_matrix[u->frame_index];
//			u->frame_next_idx = u->frame_index;
//
//			// Get correct horizontal level index - App notific index
//			_update_horizontal_app_notific_index(u, TRUE);
//
//			// Clear vertical index
//			u->vertical_index = 0;
//			// Clear notific detail index
//			u->notif_detail_index = 0;
//			N_SPRINTF("[UI] swipe left: %d, %d, %d", u->frame_prev_idx, u->frame_index, u->frame_next_idx);
//		} else  {
//			p_matrix = ui_matrix_swipe_right;
//			if (u->frame_index == UI_DISPLAY_SMART_DETAIL_NOTIF)
//				_set_animation(ANIMATION_PANNING_IN, TRANSITION_DIR_RIGHT);
//			else
//				_set_animation(ANIMATION_PANNING_OUT, TRANSITION_DIR_RIGHT);
//
//			// Update frame index
//			u->frame_index = p_matrix[u->frame_index];
//			u->frame_next_idx = u->frame_index;
//
//			// Get correct horizontal level index - App notific index
//			_update_horizontal_app_notific_index(u, FALSE);
//
//			// Clear vertical index
//			u->vertical_index = 0;
//			// Clear notific detail index
//			u->notif_detail_index = 0;
//
//			N_SPRINTF("[UI] swipe right: %d, %d, %d", u->frame_prev_idx, u->frame_index, u->frame_next_idx);
//		}
//	} else {
//		// Animation is not needed
//		u->frame_next_idx = u->frame_index;
//
//		// No further process
//		return;
//	}
//}

static void _render_one_icon_24(I8U len, I8U *p_out, const I8U *p_in)
{
        I8U j;
        I8U *p_out_0 = p_out + 64;
        I8U *p_out_1 = p_out_0 + 64;
        I8U *p_out_2 = p_out_1 + 64;

        // Render the left side
        for (j = 0; j < len; j++) {
                *p_out_0++ = (*p_in++);
                *p_out_1++ = (*p_in++);
                *p_out_2++ = (*p_in++);
        }
}
static void _render_screen()
{

//pm_stay_alive();
        OLED_full_scree_show(cling.ui.p_oled_up);

        OLED_set_display(1);
      //  pm_resume_sleep();
}

static BOOLEAN _display_frame_icon(I8U index, BOOLEAN b_render)
{
        UI_ANIMATION_CTX *u = &cling.ui;
        I32U offset = 64;

        N_SPRINTF("[UI] frame icon: %d", index);

        memset(cling.ui.p_oled_up, 0, 256);

        if (index == UI_DISPLAY_TRACKER_STEP) {
                offset = (64 - ICON32_STEP_LEN)>>1;
                _render_one_icon_24(ICON32_STEP_LEN, cling.ui.p_oled_up+offset, asset_content+ICON32_STEP);
                u->frame_cached_index = index;
        }
        else if (index == UI_DISPLAY_TRACKER_DISTANCE) {
                offset = (64-ICON32_DISTANCE_LEN)>>1;
                _render_one_icon_24(ICON32_DISTANCE_LEN, cling.ui.p_oled_up+offset, asset_content+ICON32_DISTANCE); u->frame_cached_index = index;
        }
        else if (index == UI_DISPLAY_TRACKER_CALORIES) {
                offset = (64-ICON32_CALORIES_LEN)>>1;
                _render_one_icon_24(ICON32_CALORIES_LEN, cling.ui.p_oled_up+offset, asset_content+ICON32_CALORIES); u->frame_cached_index = index;
        }
        else if (index == UI_DISPLAY_TRACKER_BALANCE) {
                UI_task_notify_get_balance();
                offset = (64-ICON32_BALANCE_LEN)>>1;
                _render_one_icon_24(ICON32_BALANCE_LEN, cling.ui.p_oled_up+offset, asset_content+ICON32_BALANCE); u->frame_cached_index = index;
        }
        else {
                return FALSE;
        }

        if (b_render) {
                // Finallywe render the frame
                _render_screen();
        }
        return TRUE;
}

static void _perform_ui_with_button_click(UI_ANIMATION_CTX *u)
{
        const I8U *p_matrix = NULL;

        u->b_detail_page = FALSE;

        if (ui_gesture_constrain[u->frame_index] & UFG_BUTTON_SINGLE) {
                p_matrix = ui_matrix_button;

                // Update frame index
                u->frame_index = p_matrix[u->frame_index];

                if (_display_frame_icon(u->frame_index, TRUE)) {
                        // Then panning in data page
                        _set_animation(ANIMATION_PANNING_IN, TRANSITION_DIR_LEFT);
                        SYSCLK_restart_timer(ui_timer, 500);
                }
                else {
                        // For clock page rendering, no need to perform an animation
                        UI_task_notify_rendering();
                }

                // Update next index as well
                u->frame_next_idx = u->frame_index;
                // Reset vertical index
                u->vertical_index = 0;
                // Reset app notific index
                u->app_notific_index = 0;
                // Reset notif detail index
                u->notif_detail_index = 0;
                N_SPRINTF("[UI] button single: %d at %d", u->frame_index, CLK_get_system_time());
        }
}

//static void _perform_ui_with_button_press_hold(UI_ANIMATION_CTX *u)
//{
//	// Erase "more" icon
//	u->b_detail_page = FALSE;
//
//	if (ui_gesture_constrain[u->frame_index] & UFG_BUTTON_HOLD) {
//		_set_animation(ANIMATION_IRIS, TRANSITION_DIR_NONE);
//		u->frame_index = UI_DISPLAY_CAROUSEL_1;
//		u->frame_next_idx = UI_DISPLAY_CAROUSEL_1;
//		// Reset vertical index
//		u->vertical_index = 0;
//		// Reset app notific index
//		u->app_notific_index = 0;
//		// Reset notif detal index
//		u->notif_detail_index = 0;
//		N_SPRINTF("[UI] button press and hold: %d", u->frame_index);
//	}
//}
//
static void _perform_ui_with_a_finger_touch(UI_ANIMATION_CTX *u, I8U gesture)
{
        const I8U *p_matrix  __attribute__((unused)) = NULL;
        I8U prev_index  __attribute__((unused));
        /*
         // Make sure current UI frame supports finger touch
         if (!(ui_gesture_constrain[u->frame_index] & UFG_FINGER_IRIS)) {
         return;
         }

         // Perform UI update with finger down on left/right/middle

         if (gesture == TOUCH_FINGER_MIDDLE) {
         u->b_detail_page = TRUE;
         N_SPRINTF("[UI] finger middle: %d, %d", u->frame_index, u->b_detail_page);

         p_matrix = ui_matrix_finger_middle;

         _set_animation(ANIMATION_IRIS, TRANSITION_DIR_NONE);

         // Update frame index
         prev_index = u->frame_index;
         u->frame_index = p_matrix[u->frame_index];
         u->frame_next_idx = u->frame_index;

         if ((prev_index == UI_DISPLAY_CAROUSEL_2) && (u->frame_index == UI_DISPLAY_SMART_REMINDER))
         {
         N_SPRINTF("[UI] check on alarm clock");
         // Always set to the first alarm clock
         REMINDER_get_time_at_index(0);
         }

         } else if (gesture == TOUCH_FINGER_LEFT) {

         u->b_detail_page = TRUE;

         N_SPRINTF("[UI] finger left: %d, %d", u->frame_index, u->b_detail_page);

         p_matrix = ui_matrix_finger_left;

         _set_animation(ANIMATION_IRIS, TRANSITION_DIR_NONE);
         // Update frame index
         prev_index = u->frame_index;
         u->frame_index = p_matrix[u->frame_index];
         u->frame_next_idx = u->frame_index;

         // If landing page changes to clock, do not show detail page
         if (u->frame_index == UI_DISPLAY_HOME) {
         if (prev_index != UI_DISPLAY_HOME) {
         u->b_detail_page = FALSE;

         N_SPRINTF("[UI] Landing to clock");
         }
         if (prev_index == UI_DISPLAY_SMART_INCOMING_MESSAGE) {
         if (!cling.activity.b_workout_active)
         cling.activity.workout_type = WORKOUT_NONE;

         if (cling.activity.workout_type > WORKOUT_NONE) {
         u->frame_index = UI_DISPLAY_STOPWATCH_STOP;
         }
         }

         } else if (gesture == TOUCH_FINGER_RIGHT) {

         p_matrix = ui_matrix_finger_right;

         if (u->b_detail_page) {
         if (_ui_vertical_animation(u->frame_index)) {
         _set_animation(ANIMATION_TILTING_OUT, TRANSITION_DIR_UP);
         } else {
         _set_animation(ANIMATION_IRIS, TRANSITION_DIR_NONE);
         }
         p_matrix = ui_matrix_finger_right;
         _update_vertical_index(u, TRUE);

         _update_notific_detail_index(u, TRUE);
         } else {
         u->b_detail_page = TRUE;

         _set_animation(ANIMATION_IRIS, TRANSITION_DIR_NONE);

         if ((u->frame_index == UI_DISPLAY_SMART_MESSAGE) ||
         (u->frame_index == UI_DISPLAY_SMART_APP_NOTIF) ||
         (u->frame_index == UI_DISPLAY_SMART_DETAIL_NOTIF))
         {
         return;
         }
         }

         N_SPRINTF("[UI] finger right: %d, %d", u->frame_index, u->b_detail_page);

         // Update frame index
         if ((u->frame_index == UI_DISPLAY_SMART_INCOMING_CALL) ||
         (u->frame_index == UI_DISPLAY_SMART_INCOMING_MESSAGE))
         {
         if (!cling.activity.b_workout_active)
         cling.activity.workout_type = WORKOUT_NONE;

         if (cling.activity.workout_type > WORKOUT_NONE) {
         u->frame_index = UI_DISPLAY_STOPWATCH_STOP;
         } else {
         u->frame_index = p_matrix[u->frame_index];
         }
         } else {
         u->frame_index = p_matrix[u->frame_index];
         }

         u->frame_next_idx = u->frame_index;
         }
         */
}


static I8U _ui_touch_sensing()
{
        UI_ANIMATION_CTX *u = &cling.ui;
        I8U gesture;

        gesture = TOUCH_get_gesture_panel();

        // UI state switching based on valid gesture from Cypress IC
        switch (gesture) {
        case TOUCH_SWIPE_LEFT:
                {
//			_perform_ui_with_a_swipe(u, gesture);
                break;
        }
        case TOUCH_SWIPE_RIGHT:
                {
//			_perform_ui_with_a_swipe(u, gesture);
                break;
        }
        case TOUCH_BUTTON_PRESS_HOLD:
                {
//			_perform_ui_with_button_press_hold(u);
                break;
        }
        case TOUCH_BUTTON_SINGLE:
                {
                _perform_ui_with_button_click(u);
                break;
        }
        case TOUCH_FINGER_MIDDLE:
                {
//			_perform_ui_with_a_finger_touch(u, TOUCH_FINGER_MIDDLE);
                break;
        }
        case TOUCH_FINGER_LEFT:
                {
//			_perform_ui_with_a_finger_touch(u, TOUCH_FINGER_LEFT);
                break;
        }
        case TOUCH_FINGER_RIGHT:
                {
                _perform_ui_with_a_finger_touch(u, TOUCH_FINGER_RIGHT);
                break;
        }
        case TOUCH_DOUBLE_TAP:
                {
//			UI_switch_state(UI_STATE_APPEAR, 0);
//			N_SPRINTF("[UI] double tapped: %d", u->frame_index);
//			u->b_detail_page = FALSE;
                break;
        }
        case TOUCH_BUTTON_PRESS_SOS:
                {
//			u->state = UI_STATE_SOS;
//			u->state_init = TRUE;
//			u->b_detail_page = FALSE;
                break;
        }
        default:
                break;
        }

        if ((gesture >= TOUCH_SWIPE_LEFT) && (gesture <= TOUCH_BUTTON_SINGLE))
                {
                // Stop reminder
                if (cling.reminder.state != REMINDER_STATE_IDLE) {
                        N_SPRINTF("[UI] check for next reminder upon touch");
                        cling.reminder.state = REMINDER_STATE_CHECK_NEXT_REMINDER;
                        ALERT_task_reminder_notify();
                }

                // Stop notification
                if (cling.notific.state != NOTIFIC_STATE_IDLE) {
                        N_SPRINTF("[UI] stop notification vibrator");
                        cling.notific.state = NOTIFIC_STATE_STOPPED_EXTERNAL;
                        ALERT_task_notific_notify();
                }
        }

        // Operation mode update
        _operation_mode_switch(gesture);

        N_SPRINTF("[UI] (%d) curr: %d, index: %d, next: %d", gesture, u->frame_prev_idx, u->frame_index, u->frame_next_idx);

        return gesture;
}

__attribute__((unused)) static void _render_one_icon_16(I8U len, I8U *p_out, const I8U *p_in)
{
        I8U j;
        I8U *p_out_0 = p_out;
        I8U *p_out_1 = p_out_0 + 128;

        // Render the left side
        for (j = 0; j < len; j++) {
                *p_out_0++ = (*p_in++);
                *p_out_1++ = (*p_in++);
        }
}

//static __INLINE void _rotate_8_bytes_core(I8U *in_data, I8U *out_data)
//{
//	I8U i, j;
//	for (j = 0; j < 8; j++) {
//		for (i = 0; i < 8; i ++) {
//			*(out_data+i) |= (((*in_data)<<i) & 0x80)>>(7-j);
//		}
//
//		in_data ++;
//	}
//}
//static void _rotate_8_bytes_opposite_core(I8U *in_data, I8U *out_data)
//{
//	I8U i, j;
//	for (j = 0; j < 8; j++) {
//		for (i = 0; i < 8; i ++) {
//			*(out_data-i) |= (((*in_data)<<i) & 0x80)>>j;
//		}
//
//		in_data ++;
//	}
//}
//
//static void _rotate_90_degree(I8U *in, I8U *out)
//{
//	I8U *in_data;
//	I8U *out_data;
//
//	// first 8 bytes
//	out_data = out;
//	in_data = in;
//	_rotate_8_bytes_core(in_data, out_data);
//
//	// 2nd 8 bytes
//	out_data = out + 128;
//	in_data = in+8;
//	_rotate_8_bytes_core(in_data, out_data);
//
//	// 3rd 8 bytes
//	out_data = out + 256;
//	in_data = in + 16;
//	_rotate_8_bytes_core(in_data, out_data);
//
//	// 4th 8 bytes
//	out_data = out + 384;
//	in_data = in + 24;
//	_rotate_8_bytes_core(in_data, out_data);
//}
//
//static void _rotate_270_degree(I8U *in, I8U *out)
//{
//	I8U *in_data;
//	I8U *out_data;
//
//	// first 8 bytes
//	out_data = out+7;
//	in_data = in;
//	_rotate_8_bytes_opposite_core(in_data, out_data);
//
//	// 2nd 8 bytes
//	out_data = out +7 - 128;
//	in_data = in+8;
//	_rotate_8_bytes_opposite_core(in_data, out_data);
//
//	// 3rd 8 bytes
//	out_data = out + 7 - 256;
//	in_data = in + 16;
//	_rotate_8_bytes_opposite_core(in_data, out_data);
//
//	// 4th 8 bytes
//	out_data = out + 7 - 384;
//	in_data = in + 24;
//	_rotate_8_bytes_opposite_core(in_data, out_data);
//}
//
//static void _render_vertical_time(I8U *string, I8U offset, BOOLEAN b_180_rotation)
//{
//	I8U *p0, *p1, *p2;
//	I8U char_len, i, j;
//	I8U buf1[32];
//	I8U buf2[32];
//	I8U buf3[32];
//	const I8U *pin;
//
//	// Render the hour
//	memset(buf1, 0, 32);
//	memset(buf2, 0, 32);
//	memset(buf3, 0, 32);
//	p0 = buf1;
//	p1 = buf2;
//	p2 = buf3;
//	for (i = 0; i < 2; i++) {
//
//		// Digits in large fonts
//		pin = asset_content+asset_pos[512+string[i]];
//		char_len = asset_len[512+string[i]];
//		for (j = 0; j < char_len; j++) {
//				*p0++ = (*pin++);
//				*p1++ = (*pin++);
//				*p2++ = (*pin++);
//		}
//		p0 += 3;
//		p1 += 3;
//		p2 += 3;
//	}
//	// do the rotation
//	if (b_180_rotation) {
//		_rotate_270_degree(buf1, cling.ui.p_oled_up+384+offset);
//		_rotate_270_degree(buf2, cling.ui.p_oled_up+384+offset+8);
//		_rotate_270_degree(buf3, cling.ui.p_oled_up+384+offset+16);
//	} else {
//		_rotate_90_degree(buf1, cling.ui.p_oled_up+offset+16);
//		_rotate_90_degree(buf2, cling.ui.p_oled_up+offset+8);
//		_rotate_90_degree(buf3, cling.ui.p_oled_up+offset);
//	}
//}
//
//static void _fill_vertical_local_clock(BOOLEAN b_180_rotation)
//{
//	I8U string[8];
//	I8U char_len, j;
//	I8U *p0, *p1, *p2;
//	const I8U *pin;
//
//	// Render the clock sign
//	if (cling.ui.clock_sec_blinking) {
//		cling.ui.clock_sec_blinking = FALSE;
//
//		p0 = cling.ui.p_oled_up+61;
//		p1 = p0+128;
//		p2 = p1+128;
//
//		// Digits in large fonts
//		pin = asset_content+asset_pos[512+':'];
//		char_len = asset_len[512+':'];
//		for (j = 0; j < char_len; j++) {
//				*p0++ = (*pin++);
//				*p1++ = (*pin++);
//				*p2++ = (*pin++);
//		}
//	} else {
//		cling.ui.clock_sec_blinking = TRUE;
//	}
//
//	// Render the hour
//	sprintf((char *)string, "%02d", cling.time.local.hour);
//	if (b_180_rotation)
//		_render_vertical_time(string, 32, b_180_rotation);
//	else
//		_render_vertical_time(string, 72, b_180_rotation);
//
//	// Render the minute
//	sprintf((char *)string, "%02d", cling.time.local.minute);
//	if (b_180_rotation)
//		_render_vertical_time(string, 72, b_180_rotation);
//	else
//		_render_vertical_time(string, 32, b_180_rotation);
//
//}
//
static I8U _fill_local_clock(char *string)
{
        I8U len;

        if (cling.ui.b_icon_blinking) {
                cling.ui.b_icon_blinking = FALSE;
                len = sprintf((char *)string, "%02d:%02d", cling.time.local.hour,
                        cling.time.local.minute);
        }
        else {
                cling.ui.b_icon_blinking = TRUE;
                len = sprintf((char *)string, "%02d %02d", cling.time.local.hour,
                        cling.time.local.minute);
        }

        // Add BOLD fonts
        string[0] += 88;
        string[1] += 88;
        string[3] += 88;
        string[4] += 88;

        return len;
}

static I16U _render_middle_section_large(I8U len, I8U *string, I8U margin, I8U alignment)
{
        I16U i, j;
        I8U *p0, *p1, *p2 __attribute__((unused));
        I8U *p4, *p5, *p6 __attribute__((unused));
        const I8U *pin;
        I8U char_len, ptr;
        I16U offset = 0;

        for (i = 0; i < len; i++) {
                // Add extra margin for character '1'
                p0 = cling.ui.p_oled_up + offset + 64;
                p1 = p0 + 64;
                if (string[i] == 32) {
                        char_len = 8;
                }
                else {
                        // Digits in large fonts
                        pin = asset_content + asset_pos[256 + string[i]];
                        char_len = asset_len[256 + string[i]];
                        for (j = 0; j < char_len; j++) {
                                *p0++ = (*pin++);
                                *p1++ = (*pin++);
                        }
                }
                if (i != (len - 1))
                        offset += char_len + margin;
                else
                        offset += char_len;

        }

        // Shift all the display to the middle
        if (alignment == ALIGNMENT_CENTER) {
                p0 = cling.ui.p_oled_up + 64;
                p1 = p0 + 64;
                ptr = (64 - offset) >> 1;

                if (ptr > 0) {
                        p0 += 63;
                        p4 = p0 - ptr;
                        p1 += 63;
                        p5 = p1 - ptr;
                        for (i = 0; i < 64 - ptr; i++) {
                                *p0-- = *p4--;
                                *p1-- = *p5--;
                        }
                        for (; i < 64; i++) {
                                *p0-- = 0;
                                *p1-- = 0;
                        }
                }
        }

        return offset;
}

static I16U _render_middle_section_small(I16U offset, I8U len, I8U *string, I8U margin, I8U alignment)
{
        I16U i, j;
        I8U *p0;
        const I8U *pin;
        I8U char_len, ptr __attribute__((unused));

        for (i = 0; i < len; i++) {
                // Add extra margin for character '1'
                p0 = cling.ui.p_oled_up + offset + 64 + 64 +64;
                if (string[i] == 32) {
                        char_len = 8;
                }
                else {
                        // Digits in large fonts
                        pin = asset_content + asset_pos[string[i]];
                        char_len = asset_len[string[i]];
                        for (j = 0; j < char_len; j++) {
                                *p0++ = (*pin++);
                        }
                }
                if (i != (len - 1))
                        offset += char_len + margin;
                else
                        offset += char_len;

        }

        // Shift all the display to the middle
        if (alignment == ALIGNMENT_CENTER) {
                // Alignment 'center'
        } else if (alignment == ALIGNMENT_LEFT) {
                // Alignment 'left'
        } else {
                // Alignment 'right'
        }

        return offset;
}

static I16U _render_middle_section_extra(I8U *p_oled, I8U offset_in, I8U len, I8U *string, I8U margin,
        I8U alignment)
{
        I16U i, j;
        I8U *p0, *p1, *p2;
        I8U *p4, *p5, *p6;
        const I8U *pin;
        I8U char_len, ptr;
        I16U offset = offset_in;

        for (i = 0; i < len; i++) {
                // Add extra margin for character '1'
                p0 = p_oled + offset;
                p1 = p0 + 64;
                p2 = p1 + 64;
                if (string[i] == 32) {
                        char_len = 11;
                }
                else {
                        // Digits in large fonts
                        pin = asset_content + asset_pos[512 + string[i]];
                        char_len = asset_len[512 + string[i]];
                        for (j = 0; j < char_len; j++) {
                                *p0++ = (*pin++);
                                *p1++ = (*pin++);
                                *p2++ = (*pin++);
                        }
                }
                if (i != (len - 1))
                        offset += char_len + margin;
                else
                        offset += char_len;

        }

        // Shift all the display to the middle
        if (alignment == ALIGNMENT_CENTER) {
                p0 = p_oled;
                p1 = p0 + 64;
                p2 = p1 + 64;
                ptr = (64 - offset) >> 1;

                if (ptr > 0) {
                        p0 += 63;
                        p4 = p0 - ptr;
                        p1 += 63;
                        p5 = p1 - ptr;
                        p2 += 63;
                        p6 = p2 - ptr;
                        for (i = 0; i < 64 - ptr; i++) {
                                *p0-- = *p4--;
                                *p1-- = *p5--;
                                *p2-- = *p6--;
                        }
                        for (; i < 64; i++) {
                                *p0-- = 0;
                                *p1-- = 0;
                                *p2-- = 0;
                        }
                }
        } else if (alignment == ALIGNMENT_LEFT) {
                // Alignment left --
                // Nothing to be done - fonts is alignment left by default
        } else {
                // Alignment right --
                p0 = p_oled;
                p1 = p0 + 64;
                p2 = p1 + 64;
                ptr = offset;

                if (ptr > 0) {
                        p0 += 63;
                        p4 = p0 - ptr;
                        p1 += 63;
                        p5 = p1 - ptr;
                        p2 += 63;
                        p6 = p2 - ptr;
                        for (i = 0; i < 64 - ptr; i++) {
                                *p0-- = *p4--;
                                *p1-- = *p5--;
                                *p2-- = *p6--;
                        }
                        for (; i < 64; i++) {
                                *p0-- = 0;
                                *p1-- = 0;
                                *p2-- = 0;
                        }
                }
        }

        return offset;
}

//static void _render_icon_more(I16U offset)
//{
//	I16U j;
//	I8U *p0;
//	const I8U *pin;
//	I8U char_len;
//
//	p0 = cling.ui.p_oled_up+128+128+128+offset;
//
//	// "More" icon
//	pin = asset_content+ICON_TOP_MORE;
//	char_len = ICON_TOP_MORE_LEN;
//	for (j = 0; j < char_len; j++) {
//			*p0++ = (*pin++);
//	}
//}
//
//static void _display_dynamic(I8U *pIn, I8U len2, I8U *string2)
//{
//	I8U *p, *pin;
//	I8U *p6;
//	I8U i, j, ptr, offset2, char_len;
//
//	// indicator
//	p = pIn;
//	offset2 = 0;
//	for (i = 0; i < len2; i++) {
//		pin = (I8U *)(asset_content+asset_pos[string2[i]]);
//		char_len = asset_len[string2[i]];
//		for (j = 0; j < char_len; j++) {
//				*p++ = (*pin++);
//		}
//
//		offset2 += char_len;
//		if (i < len2) {
//			offset2 += 3;
//		}
//
//		p += 5;
//
//	}
//
//	// Center it in the middle
//	ptr = (128 - offset2)>>1;
//	p = pIn;
//	if (ptr > 0) {
//		p += 127; p6 = p - ptr;
//		for (i = 0; i < 128-ptr; i++) {
//			*p-- = *p6--;
//		}
//		for (; i < 128; i++) {
//			*p-- = 0;
//		}
//	}
//}
//

static void _render_battery_perc(I16U perc, BOOLEAN b_blinking)
{
        I16U i, j, char_len, margin;

        I8U *p0, *p1;
        I16U offset = 0; // Pixel offet at top row
        I8U string[64];
        I8U len = 0;
        const I8U *pin;

        string[len++] = ICON_CHARGING_BATTERY_IDX;

        // Rendering the icon
        offset = 17;
        for (i = 0; i < len; i++) {
                p0 = cling.ui.p_oled_up + offset + 64;
                p1 = p0 + 64;
                pin = asset_content + asset_pos[256 + string[i]];
                for (j = 0; j < asset_len[256 + string[i]]; j++) {
                        *p0++ = (*pin++);
                        *p1++ = (*pin++);
                }
        }

        // Filling up the percentage
        perc = cling.system.mcu_reg[REGISTER_MCU_BATTERY];

        len = 0;
        offset = 37;
        string[len++] = ICON_CHARGING_IDX;

        if (perc > 33) {
                offset = 30;
                string[len++] = ICON_CHARGING_IDX;
        }

        if (perc > 66) {
                offset = 23;
                string[len++] = ICON_CHARGING_IDX;
        }

        // Blinking animation
        if (!b_blinking) {
                len--;
                offset += 7;
        }

        if (!len) {
                return;
        }
        else {
                // Filling the percentage
                margin = 1;
                for (i = 0; i < len; i++) {
                        // Add extra margin for character '1'
                        p0 = cling.ui.p_oled_up + offset + 64;
                        p1 = p0 + 64;
                        // Digits in large fonts
                        pin = asset_content + asset_pos[256 + string[i]];
                        char_len = asset_len[256 + string[i]];
                        for (j = 0; j < char_len; j++) {
                                *p0++ = (*pin++);
                                *p1++ = (*pin++);
                        }
                        if (i != (len - 1))
                                offset += char_len + margin;
                        else
                                offset += char_len;
                }
        }
}

static void _middle_row_render(I8U mode, I8U alignment)
{
        I16U i;
        I16U offset = 0, char_len; // Pixel offet at top row
        I8U *p0, *p1, *pin;
        I8U string[32];
        I8U len = 0,  margin;
        WEATHER_CTX weather  __attribute__((unused));
        I32U stat;
        I32U integer, fractional;
        I8U bar_len = 0, string_pos __attribute__((unused)) = 0;
        I8U b_large_fonts = TRUE;
        BOOLEAN b_progress_bar = FALSE;
        // Clean up top row
        memset(cling.ui.p_oled_up, 0, 256);

        // Set up margin between characters
        margin = 2;

        // Render the left side
        if (mode == UI_MIDDLE_MODE_CLOCK) {
                len = _fill_local_clock((char *)string);
        }
        else if (mode == UI_MIDDLE_MODE_STEPS) {
                TRACKING_get_activity(cling.ui.vertical_index, TRACKING_STEPS, &stat);
                len = sprintf((char *)string, "%d", stat);
        }
        else if (mode == UI_MIDDLE_MODE_BALANCE) {
                //cling.system.balance = 8888888;
                integer = cling.system.balance / 100;
                fractional = cling.system.balance - integer * 100;
                integer = (integer>99999)?99999:integer;
                len = sprintf((char *)string, "%d", integer);
                // Render integer in a extra large fonts
                _render_middle_section_extra(cling.ui.p_oled_up, offset, len, string, margin, alignment);
                // Render fractional in a small fonts
                len = sprintf((char *)string, "RMB");
                _render_middle_section_small(0, len, string, margin, ALIGNMENT_LEFT);
                len = sprintf((char *)string, ".%02d", fractional);
                _render_middle_section_small(40, len, string, margin, ALIGNMENT_LEFT);
                return;

                N_SPRINTF("BALANCE CHECK -- %d.%d", integer, fractional);

        }
        else if (mode == UI_MIDDLE_MODE_DISTANCE) {
                TRACKING_get_activity(cling.ui.vertical_index, TRACKING_DISTANCE, &stat);
                integer = stat / 1000;
                fractional = stat - integer * 1000;
                fractional /= 10;
                len = sprintf((char *)string, "%d.%02d", integer, fractional);
        }
        else if (mode == UI_MIDDLE_MODE_BATTERY) {
                _render_battery_perc(cling.system.mcu_reg[REGISTER_MCU_BATTERY], TRUE);
                return;
        }
        else if (mode == UI_MIDDLE_MODE_CALORIES) {
                TRACKING_get_activity(cling.ui.vertical_index, TRACKING_CALORIES, &stat);
                len = sprintf((char *)string, "%d", stat);
        }
        else if (mode == UI_MIDDLE_MODE_WEATHER) {
        }
        else if (mode == UI_MIDDLE_MODE_BLE_CODE) {
                p0 = cling.ui.p_oled_up + offset + 64;
                p1 = p0 + 64;
                pin = (uint8_t*)(asset_content + asset_pos[256 + ICON_MIDDLE_BLUETOOTH]);
                char_len = asset_len[256 + ICON_MIDDLE_BLUETOOTH];
                for (i = 0; i < char_len; i++) {
                        *p0++ = (*pin++);
                        *p1++ = (*pin++);
                }
                offset += char_len + margin;
                SYSTEM_get_ble_code(string);
                len = 4; // 4 BLE code characters plus one BLE icon
                alignment = ALIGNMENT_LEFT;
        }
        else if (mode == UI_MIDDLE_MODE_LINKING) {
                len = 0;
                if (cling.ui.b_icon_blinking == 0) {
                        string[len++] = ICON_MIDDLE_AUTH_LINK_3;
                }
                else if (cling.ui.b_icon_blinking == 1) {
                        string[len++] = ICON_MIDDLE_AUTH_LINK_2;
                        string[len++] = ICON_MIDDLE_AUTH_LINK_3;
                        string[len++] = ICON_MIDDLE_AUTH_LINK_4;
                }
                else {
                        string[len++] = ICON_MIDDLE_AUTH_LINK_1;
                        string[len++] = ICON_MIDDLE_AUTH_LINK_2;
                        string[len++] = ICON_MIDDLE_AUTH_LINK_3;
                        string[len++] = ICON_MIDDLE_AUTH_LINK_4;
                        string[len++] = ICON_MIDDLE_AUTH_LINK_5;
                }
                margin = 4;
                b_large_fonts = FALSE;
        }
        else if (mode == UI_MIDDLE_MODE_OTA) {
                b_progress_bar = TRUE;
                len = sprintf((char *)string, "%d %%", cling.ota.percent);
                bar_len = cling.ota.percent;
//                b_center = FALSE;
                b_large_fonts = FALSE;
                offset = (((oled_get_y_solution() >> 3) >> 1) - 1) * (oled_get_x_solution());
//                if (len < 0) {
//                        len = 0;
//                }
        }
        else {

        }

        if (len > 0) {
                if (b_large_fonts) {
                        offset = _render_middle_section_extra(cling.ui.p_oled_up+64, offset, len, string, margin, alignment);
                }
                else {
                        N_SPRINTF("UI: render large fonts (%d)", len);
                        offset = _render_middle_section_large(len, string, margin, alignment);
                }
        }

        // Add progress bar at the bottom
        if (b_progress_bar) {
                p0 = cling.ui.p_oled_up
                        + ((oled_get_y_solution() >> 3) >> 1) * (oled_get_x_solution());
                bar_len = (bar_len * oled_get_x_solution()) / 100;
                for (i = 0; i < bar_len; i++) {
                        *p0++ = 0xf0;
                }
                for (; i < oled_get_x_solution(); i++) {
                        *p0++ = 0x80;
                }
        }

}

//static I8U _render_top_sec(I8U *string, I8U len, I8U offset)
//{
//	I8U i, j, margin = 1;
//	const I8U *pin;
//	I8U *p0;
//
//	for (i = 0; i < len; i++) {
//		p0 = cling.ui.p_oled_up+offset; // Add extra 10 pixels to make it center
//		if (string[i] == ' ') {
//			offset += 4;
//		} else {
//			pin = asset_content+asset_pos[string[i]];
//			for (j = 0; j < asset_len[string[i]]; j++) {
//					*p0++ = (*pin++);
//			}
//			if (i != (len-1))
//				offset += asset_len[string[i]] + margin;
//			else
//				offset += asset_len[string[i]];
//		}
//	}
//
//	return offset;
//}
//
//static void _render_firmware_ver()
//{
//	I8U string[64];
//	I16U len;
//	I16U major;
//	I16U minor;
//
//	major = cling.system.mcu_reg[REGISTER_MCU_REVH]>>4;
//	minor = cling.system.mcu_reg[REGISTER_MCU_REVH]&0x0f;
//	minor <<= 8;
//	minor |= cling.system.mcu_reg[REGISTER_MCU_REVL];
//
//	len = sprintf((char *)string, "VER:%d.%d(%d)", major, minor,cling.whoami.touch_ver[2]);
//	FONT_load_characters(cling.ui.p_oled_up+(128-len*6), (char *)string, 8, 128, FALSE);
//}
//
//static void _render_2_indicator(I8U len1, const I8U *in1, I8U offset1, I8U len2, const I8U *in2, I8U offset2)
//{
//	I8U *p0, *p1, *pin;
//	I8U j;
//
//	// Render the left side (offset set to 60 for steps comment)
//	p0 = cling.ui.p_oled_up+offset1+98;
//	p1 = p0 + 128;
//	pin = (I8U *)in1;
//	for (j = 0; j < len1; j++) {
//		*p0++ = (*pin++);
//		*p1++ = (*pin++);
//	}
//
//	// Render the left side (offset set to 60 for steps comment)
//	p0 = cling.ui.p_oled_up+offset2+128+128+128+98;
//	pin = (I8U *)in2;
//	for (j = 0; j < len2; j++) {
//			*p0++ = (*pin++);
//	}
//}
//
//static void _render_indicator(I8U len1, const I8U *in1, I8U offset)
//{
//	I8U *p0, *pin, *p1;
//	I8U j;
//
//	// Render the left side (offset set to 60 for steps comment)
//	p0 = cling.ui.p_oled_up+offset;
//	p1 = p0 + 128;
//	pin = (I8U *)in1;
//	for (j = 0; j < len1; j++) {
//			*p0++ = (*pin++);
//			*p1++ = (*pin++);
//	}
//}
//
//
//static void _top_icon_render(I8U mode)
//{
//	I16U i;
//	I16U offset = 0; // Pixel offet at top row
//	I8U string[64];
//	I8U len=0;
//	WEATHER_CTX weather;
//
//	// Render the left side
//	if (mode == UI_TOP_MODE_NONE) {
//	} else if (mode == UI_TOP_MODE_STEPS) {
//			// Render the left side
//		_render_one_icon(ICON_TOP_STEPS_LEN, cling.ui.p_oled_up+offset, asset_content+ICON_TOP_STEPS);
//	} else if (mode == 	UI_TOP_MODE_DISTANCE) {
//		_render_one_icon(ICON_TOP_DISTANCE_LEN, cling.ui.p_oled_up+offset, asset_content+ICON_TOP_DISTANCE);
//	} else if (mode == UI_TOP_MODE_CALORIES) {
//		_render_one_icon(ICON_TOP_CALORIES_LEN, cling.ui.p_oled_up+offset, asset_content+ICON_TOP_CALORIES);
//	} else if (mode == UI_TOP_MODE_SLEEP) {
//		_render_one_icon(ICON_TOP_SLEEP_LEN, cling.ui.p_oled_up+offset, asset_content+ICON_TOP_SLEEP);
//	} else if (mode == UI_TOP_MODE_UV_IDX) {
//		_render_one_icon(ICON_TOP_UV_LEN, cling.ui.p_oled_up+offset, asset_content+ICON_TOP_UV);
//	} else if (mode == UI_TOP_MODE_INCOMING_CALL) {
//		_render_one_icon(ICON_TOP_CALLS_LEN, cling.ui.p_oled_up+offset, asset_content+ICON_TOP_CALLS);
//	} else if (mode == UI_TOP_MODE_2DIGITS_INDEX) {
//
//		len = sprintf((char *)string, "%02d", cling.ui.vertical_index);
//		FONT_load_characters(cling.ui.p_oled_up, (char *)string, 16, 128, FALSE);
//
//	} else if (mode == UI_TOP_MODE_MESSAGE) {
//		_render_one_icon(ICON_TOP_MESSAGE_LEN, cling.ui.p_oled_up+offset, asset_content+ICON_TOP_MESSAGE);
//
//	} else if (mode == UI_TOP_MODE_REMINDER) {
//		if (cling.reminder.state != REMINDER_STATE_IDLE) {
//				if (cling.ui.clock_sec_blinking) {
//					cling.ui.clock_sec_blinking = FALSE;
//					_render_one_icon(ICON_TOP_REMINDER_LEN, cling.ui.p_oled_up+offset, asset_content+ICON_TOP_REMINDER);
//				} else {
//					cling.ui.clock_sec_blinking = TRUE;
//					// Empty icon to have blinking effects
//				}
//		} else {
//
//				// Render top row REMINDER icon
//				_render_one_icon(ICON_TOP_REMINDER_LEN, cling.ui.p_oled_up+offset, asset_content+ICON_TOP_REMINDER);
//		}
//	} else if (mode == UI_TOP_MODE_HEART_RATE) {
//		if (TOUCH_is_skin_touched()) {
//				if (cling.ui.clock_sec_blinking) {
//					cling.ui.clock_sec_blinking = FALSE;
//					_render_one_icon(ICON_TOP_HEART_RATE_LEN, cling.ui.p_oled_up+offset, asset_content+ICON_TOP_HEART_RATE);
//				} else {
//					cling.ui.clock_sec_blinking = TRUE;
//					// Empty icon to have blinking effects
//				}
//		} else {
//			// Render top row heart rate icon
//			_render_one_icon(ICON_TOP_HEART_RATE_LEN, cling.ui.p_oled_up+offset, asset_content+ICON_TOP_HEART_RATE);
//		}
//	} else if (mode == UI_TOP_MODE_SKIN_TEMP) {
//		N_SPRINTF("[UI] display skin temp (2)");
//		_render_one_icon(ICON_TOP_SKIN_TEMP_LEN, cling.ui.p_oled_up+offset, asset_content+ICON_TOP_SKIN_TEMP);
//	} else if (mode == UI_TOP_MODE_WEATHER) {
//		WEATHER_get_weather(0, &weather);
//		_render_one_icon(asset_len[ICON_WEATHER_IDX+weather.type], cling.ui.p_oled_up+offset, asset_content+asset_pos[ICON_WEATHER_IDX+weather.type]);
//	} else if (mode == UI_TOP_MODE_RETURN) {
//		_render_one_icon(ICON_TOP_RETURN_LEN, cling.ui.p_oled_up, asset_content+ICON_TOP_RETURN);
//	} else if (mode == UI_TOP_MODE_OK) {
//		_render_one_icon(ICON_TOP_OK_LEN, cling.ui.p_oled_up+111, asset_content+ICON_TOP_OK);
//	} else if (mode == UI_TOP_MODE_WORKOUT_START) {
//		_render_2_indicator(ICON_TOP_WORKOUT_START_ICON_LEN, asset_content+ICON_TOP_WORKOUT_START_ICON, 5,
//												ICON_TOP_WORKOUT_START_TEXT_LEN, asset_content+ICON_TOP_WORKOUT_START_TEXT, 0);
//	} else if (mode == UI_TOP_MODE_WORKOUT_STOP) {
//		_render_2_indicator(ICON_TOP_WORKOUT_STOP_ICON_LEN, asset_content+ICON_TOP_WORKOUT_STOP_ICON, 5,
//												ICON_TOP_WORKOUT_STOP_TEXT_LEN, asset_content+ICON_TOP_WORKOUT_STOP_TEXT, 0);
//	}
//
//
//	if (len > 0) {
//		for (i = 0; i < len; i++) {
//			// Render the left side
//			_render_one_icon(asset_len[string[i]], cling.ui.p_oled_up+offset, asset_content+asset_pos[string[i]]);
//
//			offset += asset_len[string[i]] + 1;
//		}
//	}
//
//}
//
//static void _left_system_render(I8U *in, BOOLEAN b_charging, BOOLEAN b_ble_conn)
//{
//	I8U *p0, *pin;
//	I8U j;
//	I8U curr_batt_level = cling.system.mcu_reg[REGISTER_MCU_BATTERY];
//	I8U p_v = 0x1c;
//
//	if (b_ble_conn) {
//		p0 = in + ICON_BOTTOM_IND_CHARGING_LEN + 2;
//		pin = (I8U *)(asset_content+ICON_TOP_SMALL_BLE);
//		for (j = 0; j < ICON_TOP_SMALL_BLE_LEN; j++) {
//				*p0++ = (*pin++);
//		}
//	}
//
//	// Render the right side (offset set to 60 for steps comment)
//	p0 = in;
//
//	if (b_charging) {
//		pin = (I8U *)(asset_content+ICON_BOTTOM_IND_CHARGING);
//		for (j = 0; j < ICON_BOTTOM_IND_CHARGING_LEN; j++) {
//				*p0++ = (*pin++);
//		}
//	} else {
//		pin = (I8U *)(asset_content+ICON_BOTTOM_IND_NO_CHARGING);
//		for (j = 0; j < ICON_BOTTOM_IND_NO_CHARGING_LEN; j++) {
//				*p0++ = (*pin++);
//		}
//	}
//
//	// Filling up the percentage
//	curr_batt_level /= 11;
//
//	if (curr_batt_level == 0)
//		return;
//
//	if (curr_batt_level > 9)
//		curr_batt_level = 9;
//
//	// Note: the battery button icon is 9 pixels of length
//	p0 = in+2;
//	for (j = 0; j < curr_batt_level; j++) {
//		*p0++ |= p_v;
//	}
//
//	N_SPRINTF("[UI] left rendering: %d", cling.system.mcu_reg[REGISTER_MCU_BATTERY]);
//}
//
//static void render_ble_batt_rotation(BOOLEAN b_charging, BOOLEAN b_ble_conn, BOOLEAN b_90_degree)
//{
//	I8U data_buf[128];
//
//	memset(data_buf, 0, 128);
//
//	_left_system_render(data_buf, b_charging, b_ble_conn);
//
//	if (b_90_degree)
//		_rotate_90_degree(data_buf, cling.ui.p_oled_up+120);
//	else
//		_rotate_270_degree(data_buf, cling.ui.p_oled_up+384);
//}
//
//static void _render_calendar(I8U *buf, SYSTIME_CTX time)
//{
//	I8U string[64];
//	I8U len, offset=93;
//	char *week[] = {"MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"};
//
//	if (cling.ui.fonts_type) {
//		string[0] = ICON_DOW_IDX+time.dow;
//
//		len = 1;
//
//		len += sprintf((char *)(string+1), " %d",time.day);
//
//		_render_top_sec(string, len, offset);
//
//		len = sprintf((char *)string, " %d", time.day);
//		FONT_load_characters(buf+(128-len*6), (char *)string, 8, 128, FALSE);
//	} else {
//		len = sprintf((char *)string, "%s %d", week[time.dow], time.day);
//		FONT_load_characters(buf+(128-len*6), (char *)string, 8, 128, FALSE);
//	}
//}
//
//static void _render_calendar_rotation(SYSTIME_CTX time, BOOLEAN b_90_degree)
//{
//	I8U data_buf[128];
//	I8U string[32];
//
//	memset(data_buf, 0, 128);
//
//	sprintf((char *)string, "%d/%02d",time.month, time.day);
//	FONT_load_characters(data_buf, (char *)string, 8, 128, FALSE);
//	if (b_90_degree)
//		_rotate_90_degree(data_buf, cling.ui.p_oled_up);
//	else
//		_rotate_270_degree(data_buf, cling.ui.p_oled_up+384+120);
//}
//
//static void _render_dow_rotation(SYSTIME_CTX time, BOOLEAN b_90_degree)
//{
//	I8U data_buf[128];
//	I8U string[32];
//	I8U j;
//	const I8U *pin;
//	I8U *p0;
//	char *week_en[] = {"MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"};
//
//	memset(data_buf, 0, 128);
//
//	if (cling.ui.fonts_type) {
//		p0 = data_buf; // Add extra 10 pixels to make it center
//		pin = asset_content+asset_pos[ICON_DOW_IDX+time.dow];
//		for (j = 0; j < asset_len[ICON_DOW_IDX+time.dow]; j++) {
//				*p0++ = (*pin++);
//		}
//
//	} else {
//		sprintf((char *)string, "%s", week_en[time.dow]);
//		FONT_load_characters(data_buf, (char *)string, 8, 128, FALSE);
//	}
//
//	if (b_90_degree)
//		_rotate_90_degree(data_buf, cling.ui.p_oled_up+8);
//	else // 270 degree
//		_rotate_270_degree(data_buf, cling.ui.p_oled_up+384+112);
//}
//
//static void _render_battery_perc_rotation(BOOLEAN b_90_degree)
//{
//	I8U data_buf[128];
//	I8U string[32];
//
//	memset(data_buf, 0, 128);
//
//	sprintf((char *)string, "%d %%", cling.system.mcu_reg[REGISTER_MCU_BATTERY]);
//	FONT_load_characters(data_buf, (char *)string, 8, 128, FALSE);
//	if (b_90_degree)
//		_rotate_90_degree(data_buf, cling.ui.p_oled_up);
//	else // 270 degree
//		_rotate_270_degree(data_buf, cling.ui.p_oled_up+384+120);
//}
//
//static void _render_clock(SYSTIME_CTX time)
//{
//	I8U string[64];
//	I8U len;
//
//  if (cling.ui.frame_index != UI_DISPLAY_VITAL_HEART_RATE)	{
//		if (cling.ui.clock_sec_blinking) {
//		  cling.ui.clock_sec_blinking = FALSE;
//		  len = sprintf((char *)string, "%d:%02d",time.hour, time.minute);
//	  } else {
//		  cling.ui.clock_sec_blinking = TRUE;
//		  len = sprintf((char *)string, "%d %02d",time.hour, time.minute);
//	  }
//  } else {
//		len = sprintf((char *)string, "%d:%02d",time.hour, time.minute);
//	}
//
//	FONT_load_characters(cling.ui.p_oled_up+(128-len*6), (char *)string, 8, 128, FALSE);
//}
//
//static void _right_row_render(I8U mode)
//{
//	I8U string[64];
//	I8U len;
//	SYSTIME_CTX delta;
//
//	// Render the right side
//	if (mode == UI_BOTTOM_MODE_NONE) {
//	} else if (mode == UI_BOTTOM_MODE_2DIGITS_INDEX) {
//
//		if (cling.reminder.ui_alarm_on)
//			return;
//		len = sprintf((char *)string, "%02d", cling.ui.vertical_index);
//		FONT_load_characters(cling.ui.p_oled_up+(128-len*8), (char *)string, 16, 128, FALSE);
//	} else if (mode == UI_BOTTOM_MODE_APP_NOTIFIC_2DIGITS_INDEX) {
//		len = sprintf((char *)string, "%02d", cling.ui.app_notific_index);
//		FONT_load_characters(cling.ui.p_oled_up+(128-len*8), (char *)string, 16, 128, FALSE);
//	} else if (mode == UI_BOTTOM_MODE_CLOCK) {
//		_render_clock(cling.time.local);
//	} else if (mode == UI_BOTTOM_MODE_CALENDAR) {
//		if (cling.ui.clock_orientation == 1) {
//			_render_calendar(cling.ui.p_oled_up, cling.time.local);
//		} else if (cling.ui.clock_orientation == 2) {
//			_render_calendar_rotation(cling.time.local, FALSE);
//			_render_dow_rotation(cling.time.local, FALSE);
//		} else {
//			_render_calendar_rotation(cling.time.local, TRUE);
//			_render_dow_rotation(cling.time.local, TRUE);
//		}
//	} else if (mode == UI_BOTTOM_MODE_DELTA_DATE_BACKWARD) {
//
//		RTC_get_delta_clock_backward(&delta, cling.ui.vertical_index);
//		_render_calendar(cling.ui.p_oled_up, delta);
//
//	} else if (mode == UI_BOTTOM_MODE_MAX) {
//		_render_indicator(ICON_BOTTOM_IND_MAX_LEN, asset_content+ICON_BOTTOM_IND_MAX, 123);
//
//	} else if (mode == UI_BOTTOM_MODE_FIRMWARE_VER) {
//		_render_firmware_ver();
//	} else if (mode == UI_BOTTOM_MODE_CHARGING_PERCENTAGE) {
//		if (cling.ui.clock_orientation == 1) {
//			len = sprintf((char *)string, "%d %%", cling.system.mcu_reg[REGISTER_MCU_BATTERY]);
//			FONT_load_characters(cling.ui.p_oled_up+(128-len*6), (char *)string, 8, 128, FALSE);
//		} else if (cling.ui.clock_orientation == 2) {
//			_render_battery_perc_rotation(FALSE);
//		} else {
//			_render_battery_perc_rotation(TRUE);
//		}
//	} else if (mode == UI_BOTTOM_MODE_OK) {
//		_render_one_icon(ICON_TOP_OK_LEN, cling.ui.p_oled_up+110, asset_content+ICON_TOP_OK);
//	}
//}
//

//static I8U _render_carousel_icon_single(I16U x, I8U char_len, I8U index)
//{
//	I8U j;
//	I8U *p0, *p1, *p2;
//	const I8U *pin;
//
//	if (!index)
//		return 0;
//
//	// We might add some margin here
//	p0 = cling.ui.p_oled_up+128+x;
//	p1 = p0 + 128;
//	p2 = p1 + 128;
//
//	pin = asset_content+asset_pos[512+index];
//	for (j = 0; j < char_len; j++) {
//			*p0++ = (*pin++);
//			*p1++ = (*pin++);
//			*p2++ = (*pin++);
//	}
//
//	return char_len;
//}
//
//static void _core_display_carousel(I8U left, I8U top, I8U right, BOOLEAN b_render)
//{
//	I8U start_offset;
//	I8U char_len;
//
//	// Clean up top row
//	memset(cling.ui.p_oled_up, 0, 256);
//
//	// Render the left icon
//	char_len = asset_len[512+left];
//	start_offset = 0;
//	_render_carousel_icon_single(start_offset, char_len, left);
//
//	char_len = asset_len[512+top];
//	start_offset = 42-char_len;
//	start_offset >>= 1;
//	start_offset += 42;
//	_render_carousel_icon_single(start_offset, char_len, top);
//
//	char_len = asset_len[512+right];
//	start_offset = 128-char_len;
//	_render_carousel_icon_single(start_offset, char_len, right);
//
//	if (b_render) {
//		// Finally, we render the frame
//		_render_screen();
//	}
//}
//
static void _core_home_display_horizontal(I8U middle, I8U bottom, BOOLEAN b_render)
{
        BOOLEAN b_charging __attribute__((unused)) = FALSE;
        BOOLEAN b_conn __attribute__((unused)) = FALSE;
        I8U i __attribute__((unused));
        //I8U *p0, *p1, *p2, *p3;

        // Main info
        _middle_row_render(middle, ALIGNMENT_CENTER);

//	// Info on the right
//	if (BATT_is_charging())
//		b_charging = TRUE;
//	if (BTLE_is_connected())
//		b_conn = TRUE;
//
//	// Charging icon & BLE connection
//	if (middle == UI_MIDDLE_MODE_CLOCK) {
//		if (cling.ui.clock_orientation == 1) {
//			_left_system_render(cling.ui.p_oled_up, b_charging, b_conn);
//		} else if (cling.ui.clock_orientation == 2) {
//			render_ble_batt_rotation(b_charging, b_conn, FALSE);
//		} else {
//			render_ble_batt_rotation(b_charging, b_conn, TRUE);
//		}
//	} else {
//		_left_system_render(cling.ui.p_oled_up, b_charging, b_conn);
//	}
//
//	// Info on the left
//	_right_row_render(bottom);
//
//	// For clock page, we clean up a bit for the 'more' icon
//
//	if ((middle == UI_MIDDLE_MODE_CLOCK) && cling.ui.b_detail_page) {
//		p0 = cling.ui.p_oled_up+110;
//		p1 = p0 + 128;
//		p2 = p1 + 128;
//		p3 = p2 + 128;
//		for (i = 110; i < 128; i ++) {
//			*p0++ = 0;
//			*p1++ = 0;
//			*p2++ = 0;
//			*p3++ = 0;
//		}
//		N_SPRINTF("[UI] clean up space for detail icon");
//		_render_icon_more(110);
//	}

        if (b_render) {
                // Finally, we render the frame
                _render_screen();
        }
}

static void _core_display_horizontal(I8U top, I8U middle, I8U bottom, BOOLEAN b_render)
{
        // Main info
        _middle_row_render(middle, ALIGNMENT_CENTER);

        // Info on the left
//	_top_icon_render(top);
//
//	_right_row_render(bottom);

        if (b_render) {
                // Finallywe render the frame
                _render_screen();
        }
}

static void _display_unauthorized_home()
{
        N_SPRINTF("[UI] Render unauthorized page: %d", CLK_get_system_time());

        _core_home_display_horizontal(UI_MIDDLE_MODE_BLE_CODE, UI_BOTTOM_MODE_FIRMWARE_VER, TRUE);
}

static void _display_authenticating()
{
        _core_home_display_horizontal(UI_MIDDLE_MODE_LINKING, UI_BOTTOM_MODE_NONE, TRUE);
}

static void _display_tracking_stats(UI_ANIMATION_CTX *u, BOOLEAN b_render, I8U top, I8U middle,
        I8U bottom)
{
        N_SPRINTF("[UI] tracking frame: %d", middle);
        if (u->vertical_index == 0) {
                _core_display_horizontal(top, middle, bottom, b_render);
        }
        else {
                _core_display_horizontal(top, middle, UI_BOTTOM_MODE_DELTA_DATE_BACKWARD, b_render);
        }
}

static void _display_frame_tracker(I8U index, BOOLEAN b_render)
{
        UI_ANIMATION_CTX *u = &cling.ui;

        switch (index) {
        case UI_DISPLAY_TRACKER_STEP:
                _display_tracking_stats(u, b_render, UI_TOP_MODE_STEPS, UI_MIDDLE_MODE_STEPS,
                        UI_BOTTOM_MODE_CLOCK);
                break;
        case UI_DISPLAY_TRACKER_DISTANCE:
                _display_tracking_stats(u, b_render, UI_TOP_MODE_DISTANCE, UI_MIDDLE_MODE_DISTANCE,
                        UI_BOTTOM_MODE_CLOCK);
                break;
        case UI_DISPLAY_TRACKER_CALORIES:
                _display_tracking_stats(u, b_render, UI_TOP_MODE_CALORIES, UI_MIDDLE_MODE_CALORIES,
                        UI_BOTTOM_MODE_CLOCK);
                break;
        case UI_DISPLAY_TRACKER_BATTERY:
                _display_tracking_stats(u, b_render, UI_TOP_MODE_SLEEP, UI_MIDDLE_MODE_BATTERY,
                        UI_BOTTOM_MODE_CLOCK);
                break;
        case UI_DISPLAY_TRACKER_BALANCE:
                _display_tracking_stats(u, b_render, UI_TOP_MODE_BALANCE, UI_MIDDLE_MODE_BALANCE,
                        UI_BOTTOM_MODE_CLOCK);
                break;
        case UI_DISPLAY_BLE_CODE:
                _display_unauthorized_home();
                break;
        default:
                break;
        }
}

static void _display_weather(UI_ANIMATION_CTX *u, BOOLEAN b_render)
{
        _core_display_horizontal(UI_TOP_MODE_WEATHER, UI_MIDDLE_MODE_WEATHER, UI_BOTTOM_MODE_CLOCK,
                b_render);
}

static void _display_idle_alert()
{
        I8U string1[32] __attribute__((unused));
//
//	memset(cling.ui.p_oled_up, 0, 512);
//
//	if (cling.ui.fonts_type == LANGUAGE_TYPE_ENGLISH) {
//	  sprintf((char *)string1, "time for a move");
//	} else if (cling.ui.fonts_type == LANGUAGE_TYPE_SIMPLIFIED_CHINESE) {
//		sprintf((char *)string1, "璇ヨ捣鏉ュ姩鍔ㄤ簡");
//	} else {
//		sprintf((char *)string1, "瑭茶捣渚嗗嫊鍕曚簡");
//	}
//
//	FONT_load_characters(cling.ui.p_oled_up+128, (char *)string1, 16, 128, TRUE);
//
//	_render_screen();
}
//
//static void _display_frame_workout(I8U index, BOOLEAN b_render)
//{
//	I8U string1[32];
//	I8U string2[32];
//	I8U len2=0, workout_idx=0;
//	char *workout_en[] = {"Walk", "Run", "Row", "Elliptical", "Stairs", "Cycle", "Aerobic", "Piloxing", "Others", "Outdoor", "Indoor"};
//	char *workout_s_cn[] = {"寰掓", "璺戞", "鍒掕埞", "妞渾鏈� ", "鐖ゼ姊� ", "鍗曡溅", "鏈夋哀鎿� ", "Piloxing", "鍏跺畠", "鎴峰", "瀹ゅ唴"};
//	char *workout_t_cn[] = {"寰掓", "璺戞", "鍒掕埞", "姗㈠湏姗� ", "鐖〒姊� ", "鍠粖", "鏈夋哀鎿� ", "Piloxing", "鍏跺畠", "鎴跺", "瀹ゅ収"};
//	char *workout_indicator[] = {
//		"-,,,,,,,,",
//		",-,,,,,,,",
//		",,-,,,,,,",
//		",,,-,,,,,",
//		",,,,-,,,,",
//		",,,,,-,,,",
//		",,,,,,-,,",
//		",,,,,,,-,",
//		",,,,,,,,-",
//		"-,",
//		",-"};
//
//	memset(cling.ui.p_oled_up, 0, 512);
//
//	workout_idx = index - UI_DISPLAY_WORKOUT_WALKING;
//
//	if (cling.ui.fonts_type == LANGUAGE_TYPE_ENGLISH) {
//		sprintf((char *)string1, "%s", workout_en[workout_idx]);
//	}	else if (cling.ui.fonts_type == LANGUAGE_TYPE_SIMPLIFIED_CHINESE) {
//		sprintf((char *)string1, "%s", workout_s_cn[workout_idx]);
//	} else {
//		sprintf((char *)string1, "%s", workout_t_cn[workout_idx]);
//	}
//
//	len2 = sprintf((char *)string2, "%s", workout_indicator[workout_idx]);
//
//	FONT_load_characters(cling.ui.p_oled_up+128, (char *)string1, 16, 128, TRUE);
//
//	_display_dynamic(cling.ui.p_oled_up+128*3, len2, string2);
//
//	_top_icon_render(UI_TOP_MODE_RETURN);
//	_top_icon_render(UI_TOP_MODE_OK);
//
//	if (b_render) {
//
//		// Finally, we render the frame
//		_render_screen();
//	}
//}
//
//static void _display_stopwatch_core(I8U *string1, I8U len1, I8U *string2, I8U len2, I8U mode, BOOLEAN b_center)
//{
//	I8U *p0, *pin, *p1, *p2;
//	I8U *p4, *p5, *p6;
//	I8U i, j, ptr, offset1, char_len;
//
//	// Text line
//	p0 = cling.ui.p_oled_up;
//	p1 = p0 + 128;
//	p2 = p1 + 128;
//	offset1 = 0;
//	for (i = 0; i < len1; i++) {
//
//		if (string1[i] == '1') {
//			p0 += 2;
//			p1 += 2;
//			p2 += 2;
//			offset1 += 2;
//		}
//
//		pin = (I8U *)(asset_content+asset_pos[512+string1[i]]);
//		char_len = asset_len[512+string1[i]];
//		for (j = 0; j < char_len; j++) {
//				*p0++ = (*pin++);
//				*p1++ = (*pin++);
//				*p2++ = (*pin++);
//		}
//		offset1 += char_len;
//		p0 ++;
//		p1 ++;
//		p2 ++;
//		if (i < len1)
//			offset1 ++;
//
//		if (string1[i] == '1') {
//			p0 += 3;
//			p1 += 3;
//			p2 += 3;
//			offset1 += 3;
//		}
//
//	}
//
//	p0 = cling.ui.p_oled_up;
//	p1 = p0+128;
//	p2 = p1+128;
//	if (b_center) {
//
//		ptr = (128 - offset1)>>1;
//
//		if (ptr > 0) {
//			p0 += 127; p4 = p0 - ptr;
//			p1 += 127; p5 = p1 - ptr;
//			p2 += 127; p6 = p2 - ptr;
//			for (i = 0; i < 128-ptr; i++) {
//				*p0-- = *p4--;
//				*p1-- = *p5--;
//				*p2-- = *p6--;
//			}
//			for (; i < 128; i++) {
//				*p0-- = 0;
//				*p1-- = 0;
//				*p2-- = 0;
//			}
//		}
//	} else if (b_center > 1) {
//		ptr = 128 - offset1;
//
//		if (ptr > 0) {
//			p0 += 127; p4 = p0 - ptr;
//			p1 += 127; p5 = p1 - ptr;
//			p2 += 127; p6 = p2 - ptr;
//			for (i = 0; i < offset1; i++) {
//				*p0-- = *p4--;
//				*p1-- = *p5--;
//				*p2-- = *p6--;
//			}
//			for (; i < 128; i++) {
//				*p0-- = 0;
//				*p1-- = 0;
//				*p2-- = 0;
//			}
//		}
//	}
//
//	if (len2 > 0) {
//		// indicator
//		p2 = cling.ui.p_oled_up+128+128+128+60;
//		for (i = 0; i < len2; i++) {
//			pin = (I8U *)(asset_content+asset_pos[string2[i]]);
//			char_len = asset_len[string2[i]];
//			for (j = 0; j < char_len; j++) {
//					*p2++ = (*pin++);
//			}
//			p2 += 5;
//		}
//	}
//
//	_top_icon_render(mode);
//}
//
//static void _display_stopwatch_start(I8U *string1, I8U len1, I8U mode)
//{
//	I8U *p0, *pin, *p1, *p2;
//	I8U *p4, *p5, *p6;
//	I8U i, j, ptr, offset1, char_len;
//
//	// Text line
//	p0 = cling.ui.p_oled_up;
//	p1 = p0 + 128;
//	p2 = p1 + 128;
//	offset1 = 0;
//	for (i = 0; i < len1; i++) {
//
//		pin = (I8U *)(asset_content+asset_pos[512+string1[i]]);
//		char_len = asset_len[512+string1[i]];
//		for (j = 0; j < char_len; j++) {
//				*p0++ = (*pin++);
//				*p1++ = (*pin++);
//				*p2++ = (*pin++);
//		}
//		offset1 += char_len;
//		p0 ++;
//		p1 ++;
//		p2 ++;
//		if (i < len1)
//			offset1 ++;
//	}
//
//	p0 = cling.ui.p_oled_up;
//	p1 = p0+128;
//	p2 = p1+128;
//
//	ptr = (128 - offset1)>>1;
//
//	if (ptr > 0) {
//		p0 += 127; p4 = p0 - ptr;
//		p1 += 127; p5 = p1 - ptr;
//		p2 += 127; p6 = p2 - ptr;
//		for (i = 0; i < 128-ptr; i++) {
//			*p0-- = *p4--;
//			*p1-- = *p5--;
//			*p2-- = *p6--;
//		}
//		for (; i < 128; i++) {
//			*p0-- = 0;
//			*p1-- = 0;
//			*p2-- = 0;
//		}
//	}
//
//
//	_top_icon_render(mode);
//}
//
//static void _display_frame_setting(I8U index, BOOLEAN b_render)
//{
//	_core_display_horizontal(UI_TOP_MODE_RETURN, UI_MIDDLE_MODE_BLE_CODE, UI_BOTTOM_MODE_FIRMWARE_VER, b_render);
//}
//
//
//static void _display_frame_stopwatch(I8U index, BOOLEAN b_render)
//{
//	I8U string1[32];
//	I8U string2[32];
//	I8U len1=0, len2 = 0;
//	BOOLEAN b_center = TRUE;
//	I32U t_diff, hour, minute, second;
//
//	memset(cling.ui.p_oled_up, 0, 512);
//
//	switch (index) {
//		case UI_DISPLAY_STOPWATCH_START:
//		{
//			len1 = sprintf((char *)string1, "00:00");
//			_display_stopwatch_start(string1, len1, UI_TOP_MODE_WORKOUT_START);
//			_top_icon_render(UI_TOP_MODE_RETURN);
//			//_render_one_icon(ICON_TOP_RETURN_LEN, cling.ui.p_oled_up, asset_content+ICON_TOP_RETURN);
//			break;
//		}
//		case UI_DISPLAY_STOPWATCH_STOP:
//		{
//			t_diff = CLK_get_system_time();
//			t_diff -= cling.activity.workout_time_stamp_start;
//			t_diff /= 1000;
//			hour = t_diff / 3600;
//			t_diff -= hour*3600;
//			minute = t_diff / 60;
//			t_diff -= minute * 60;
//			second = t_diff;
//			len1 = sprintf((char *)string1, "%d:%02d:%02d", hour, minute, second);
//			len2 = sprintf((char *)string2, "-,,");
//			b_center = FALSE;
//			_display_stopwatch_core(string1, len1, string2, len2, UI_TOP_MODE_WORKOUT_STOP, b_center);
//
//			N_SPRINTF("[UI] stopwatch stop: %d:%d.%d (%d,%d)", hour, minute, second, CLK_get_system_time(), cling.activity.workout_time_stamp_start);
//			break;
//		}
//		case UI_DISPLAY_STOPWATCH_RESULT:
//		{
//			t_diff = cling.activity.workout_time_stamp_stop - cling.activity.workout_time_stamp_start;
//			t_diff /= 1000;
//			hour = t_diff / 3600;
//			t_diff -= hour*3600;
//			minute = t_diff / 60;
//			t_diff -= minute * 60;
//			second = t_diff;
//			len1 = sprintf((char *)string1, "%d:%02d:%02d", hour, minute, second);
//			len2 = 0;
//			b_center = 2;
//			_display_stopwatch_core(string1, len1, string2, len2, UI_TOP_MODE_RETURN, b_center);
//			break;
//		}
//		case UI_DISPLAY_STOPWATCH_HEARTRATE:
//		{
//			if (TOUCH_is_skin_touched()) {
//
//				len1 = sprintf((char *)string1, "%d", PPG_minute_hr_calibrate());
//
//			} else {
//				N_SPRINTF("[UI] Heart rate - not valid");
//
//				len1 = 0;
//				string1[len1++] = ICON_MIDDLE_NO_SKIN_TOUCH;
//				string1[len1] = 0;
//			}
//
//			len2 = sprintf((char *)string2, ",-,");
//			_display_stopwatch_core(string1, len1, string2, len2, UI_TOP_MODE_HEART_RATE, b_center);
//			break;
//		}
//		case UI_DISPLAY_STOPWATCH_CALORIES:
//		{
//			t_diff = cling.activity.day.calories - cling.activity.workout_Calories_start;
//			t_diff += cling.activity.workout_Calories_acc;
//
//			len1 = sprintf((char *)string1, "%d", (t_diff>> 4));
//			len2 = sprintf((char *)string2, ",,-");
//			_display_stopwatch_core(string1, len1, string2, len2, UI_TOP_MODE_CALORIES, b_center);
//			break;
//		}
//		default:
//			break;
//	}
//
//	//_top_icon_render(UI_TOP_MODE_RETURN);
//
//	if (b_render) {
//
//		// Finally, we render the frame
//		_render_screen();
//	}
//}
//
static void _display_frame_smart(I8U index, BOOLEAN b_render)
{
        UI_ANIMATION_CTX *u = &cling.ui;

        switch (index) {
        case UI_DISPLAY_SMART_WEATHER:
                _display_weather(u, b_render);
                break;
//		case UI_DISPLAY_SMART_MESSAGE:
//			_core_display_horizontal(UI_TOP_MODE_RETURN, UI_MIDDLE_MODE_MESSAGE, UI_BOTTOM_MODE_CLOCK, b_render);
//			break;
//		case UI_DISPLAY_SMART_APP_NOTIF:
//			_core_display_horizontal(UI_TOP_MODE_RETURN, UI_MIDDLE_MODE_APP_NOTIF, UI_BOTTOM_MODE_APP_NOTIFIC_2DIGITS_INDEX, b_render);
//			break;
//		case UI_DISPLAY_SMART_DETAIL_NOTIF:
//			_core_display_horizontal(UI_TOP_MODE_NONE, UI_MIDDLE_MODE_DETAIL_NOTIF, UI_BOTTOM_MODE_NONE, b_render);
//			break;
        case UI_DISPLAY_SMART_REMINDER:
                _core_display_horizontal(UI_TOP_MODE_REMINDER, UI_MIDDLE_MODE_REMINDER,
                        UI_BOTTOM_MODE_2DIGITS_INDEX, b_render);
                break;
//		case UI_DISPLAY_SMART_PHONE_FINDER:
//			_core_display_horizontal(UI_TOP_MODE_NONE, UI_MIDDLE_MODE_PHONE_FINDER, UI_BOTTOM_MODE_NONE, b_render);
//			break;
//		case UI_DISPLAY_SMART_INCOMING_CALL:
//			_core_display_horizontal(UI_TOP_MODE_INCOMING_CALL, UI_MIDDLE_MODE_INCOMING_CALL, UI_BOTTOM_MODE_OK, b_render);
//			break;
//		case UI_DISPLAY_SMART_INCOMING_MESSAGE:
//			_core_display_horizontal(UI_TOP_MODE_RETURN, UI_MIDDLE_MODE_INCOMING_MESSAGE, UI_BOTTOM_MODE_CLOCK, b_render);
//			break;
        default:
                break;
        }
}

//static void _display_heart_rate(UI_ANIMATION_CTX *u, BOOLEAN b_render)
//{
//	_core_display_horizontal(UI_TOP_MODE_HEART_RATE, UI_MIDDLE_MODE_HEART_RATE, UI_BOTTOM_MODE_CLOCK, b_render);
//}
//
//static void _display_skin_temperature(UI_ANIMATION_CTX *u, BOOLEAN b_render)
//{
//	N_SPRINTF("[UI] display skin temp (1)");
//	// Current skin temperature
//	_core_display_horizontal(UI_TOP_MODE_SKIN_TEMP, UI_MIDDLE_MODE_SKIN_TEMP, UI_BOTTOM_MODE_CLOCK, b_render);
//}
//
//static void _display_frame_vital(I8U index, BOOLEAN b_render)
//{
//	UI_ANIMATION_CTX *u = &cling.ui;
//
//	switch (index) {
//		case UI_DISPLAY_VITAL_SKIN_TEMP:
//			N_SPRINTF("[UI] vital: skin temp");
//			_display_skin_temperature(u, b_render);
//			break;
//		case UI_DISPLAY_VITAL_HEART_RATE:
//			N_SPRINTF("[UI] vital: heart rate");
//			_display_heart_rate(u, b_render);
//			break;
//		default:
//			break;
//	}
//}
//
//static void _display_frame_carousel(I8U index, BOOLEAN b_render)
//{
//	switch (index) {
//		case UI_DISPLAY_CAROUSEL_1:
//			_core_display_carousel(ICON32_WORKOUT_IDX, ICON32_STOPWATCH_IDX, ICON32_MESSAGE_IDX, b_render);
//			break;
//		case UI_DISPLAY_CAROUSEL_2:
//			_core_display_carousel(ICON32_WEATHER_IDX, ICON32_ALARM_IDX, ICON32_SETTING_IDX, b_render);
//			break;
//		default:
//			break;
//	}
//}
//
static void _display_frame_ota(I8U index, BOOLEAN b_render)
{
        _core_display_horizontal(UI_TOP_MODE_NONE, UI_MIDDLE_MODE_OTA, UI_BOTTOM_MODE_NONE,
                b_render);
}
//
//static void _display_sos(BOOLEAN b_render)
//{
//	_core_display_horizontal(UI_TOP_MODE_NONE, UI_MIDDLE_MODE_SOS, UI_BOTTOM_MODE_CLOCK, b_render);
//}

static void _display_frame_home(BOOLEAN b_render)
{
        if (BATT_is_charging()) {
                _core_home_display_horizontal(UI_MIDDLE_MODE_CLOCK,
                        UI_BOTTOM_MODE_CHARGING_PERCENTAGE, b_render);
        }
        else {
                _core_home_display_horizontal(UI_MIDDLE_MODE_CLOCK, UI_BOTTOM_MODE_CALENDAR,
                        b_render);
        }

        N_SPRINTF("UI: render home page at %d", CLK_get_system_time());
}

static void _display_frame_appear(I8U index, BOOLEAN b_render)
{
        UI_ANIMATION_CTX *u = &cling.ui;

        N_SPRINTF("[UI] frame appear: %d", index);

        if (index == UI_DISPLAY_HOME) {
                _display_frame_home(b_render);
                u->frame_cached_index = index;
        }
        else if ((index >= UI_DISPLAY_TRACKER) && (index <= UI_DISPLAY_TRACKER_END)) {
                _display_frame_tracker(index, b_render);
                u->frame_cached_index = index;
        }
        else if ((index >= UI_DISPLAY_SMART) && (index <= UI_DISPLAY_SMART_END)) {
                _display_frame_smart(index, b_render);
                u->frame_cached_index = index;
        }
        else if ((index >= UI_DISPLAY_VITAL) && (index <= UI_DISPLAY_VITAL_END)) {
//		_display_frame_vital(index, b_render);
                u->frame_cached_index = index;
        }
        else if ((index >= UI_DISPLAY_WORKOUT) && (index <= UI_DISPLAY_WORKOUT_END)) {
//		_display_frame_workout(index, b_render);
                u->frame_cached_index = index;
        }
        else if ((index >= UI_DISPLAY_STOPWATCH) && (index <= UI_DISPLAY_STOPWATCH_END)) {
//		_display_frame_stopwatch(index, b_render);
                u->frame_cached_index = index;
        }
        else if ((index >= UI_DISPLAY_SETTING) && (index <= UI_DISPLAY_SETTING_END)) {
//		_display_frame_setting(index, b_render);
                u->frame_cached_index = index;
        }
        else if ((index >= UI_DISPLAY_CAROUSEL) && (index <= UI_DISPLAY_CAROUSEL_END)) {
//		_display_frame_carousel(index, b_render);
        }
        else if (index == UI_DISPLAY_OTA) {
                _display_frame_ota(index, b_render);
        }
        else {
        }
}

#ifdef _CLINGBAND_RAINFLOWER_MODEL_
static void _render_display_restart()
{
        I8U string[64] __attribute__((unused));
        __attribute__((unused)) I32U len;
        memset(cling.ui.p_oled_up, 0, 256);
        //len = sprintf((char *)string, "E-PAY");
        //_render_middle_section_extra(0, len, string, 2, TRUE);
        _render_screen();
        N_SPRINTF("UI: rendering the start page at %d", CLK_get_system_time());
}
#endif

static void _display_charging(I16U perc, BOOLEAN b_blinking)
{
        // Render the icon the first
        memset(cling.ui.p_oled_up, 0, 256);

        _render_battery_perc(perc, b_blinking);

        // Finally, we render the frame
        _render_screen();

}

void UI_switch_state(I8U state, I32U interval)
{
        UI_ANIMATION_CTX *u = &cling.ui;

        u->state_init = TRUE;
        u->state = state;
        u->frame_interval = interval;
        u->display_to_base = CLK_get_system_time();

        UI_task_notify_rendering();

        N_SPRINTF("[UI] state switching: %d, %d", state, u->display_to_base);
}

BOOLEAN UI_is_idle()
{
        return (cling.ui.state == UI_STATE_IDLE);
}

#define OVERALL_PANNING_FRAMES 7

const I8U panning_table[] = {
        0, 16, 16, 16, 16, 0, 0
};

static void _frame_buffer_move_forward(I8U offset, I8U width)
{
        I8U *p0;
        I8U len = width - offset;
        int i;

        p0 = cling.ui.p_oled_up;

        for (i = 0; i < 4; i++) {
                memcpy(p0, p0 + offset, len);
                p0 += width;
        }

        p0 = cling.ui.p_oled_up + len;

        for (i = 0; i < 4; i++) {
                memset(p0, 0, offset);
                p0 += width;
        }
}

static void _frame_buffer_move_backward(I8U offset, I8U width)
{
        I8U *p0;
        I8U len = width - offset;
        int i;

        p0 = cling.ui.p_oled_up;

        if (len) {
                for (i = 0; i < 4; i++) {
                        memcpy(p0 + offset, p0, len);
                        p0 += width;
                }
        }

        p0 = cling.ui.p_oled_up;

        for (i = 0; i < 4; i++) {
                memset(p0, 0, offset);
                p0 += width;
        }
}

//const I8U iris_tab[4][12] = {
// {128,0,0,
//	48,32,48,
//	48,32,48,
//	128,0,0},
// {128,0,0,
//	32,64,32,
//	32,64,32,
//	128,0,0},
// {16,96,16,
//	16,96,16,
//	16,96,16,
//	16,96,16},
// {8,112,8,
//	8,112,8,
//	8,112,8,
//	8,112,8},
//};
//
//static void _iris_frame_core(I8U offset)
//{
//	I8U *p0;
//	I8U i;
//	I8U len, index;
//	const I8U *p_tab;
//
//	p0 = cling.ui.p_oled_up;
//
//	if (offset == 1) {
//		memset(p0, 0, 512);
//		return;
//	} else if (offset == 5) {
//		return;
//	}
//
//	p_tab = iris_tab[offset-2];
//	for (i = 0; i < 4; i++) {
//		len = *p_tab++; // reset
//		index = len;
//		memset(p0,0,len);
//
//		index += *p_tab++;  // Keep value
//
//		len = *p_tab++; // reset
//		memset(p0+index,0,len); //
//		p0 += 128;
//	}
//
//}

static BOOLEAN _frame_panning(I8U dir, BOOLEAN b_out)
{
        UI_ANIMATION_CTX *u = &cling.ui;
        I8U shift_pos = 0;
        I8U i;

        // Update animation index
        u->animation_index++;

        if (u->animation_index > OVERALL_PANNING_FRAMES) {
                u->animation_index = 0;
                return TRUE;
        }

        // Shift screen
        if (b_out) {
                // Panning out current frame
                _display_frame_appear(u->frame_prev_idx, FALSE);

                for (i = 0; i < u->animation_index; i++) {
                        shift_pos += panning_table[i];
                }

                if (dir == TRANSITION_DIR_LEFT) {
                        _frame_buffer_move_forward(shift_pos, 64);
                }
                else {
                        _frame_buffer_move_backward(shift_pos, 64);
                }
        }
        else {
                // Panning in next frame
                _display_frame_appear(u->frame_next_idx, FALSE);
                shift_pos = 64;
                for (i = 0; i < u->animation_index; i++) {
                        shift_pos -= panning_table[i];
                }

                if (dir == TRANSITION_DIR_LEFT) {
                        _frame_buffer_move_backward(shift_pos, 64);
                }
                else {
                        _frame_buffer_move_forward(shift_pos, 64);
                }
        }

        // update frame buffer and render it
        _render_screen();

        // Keep animating
        SYSCLK_restart_timer(ui_timer, 20);

        return FALSE;
}

//static BOOLEAN _frame_iris()
//{
//	UI_ANIMATION_CTX *u = &cling.ui;
//
//	// Update animation index
//	u->animation_index ++;
//
//	if (u->animation_index >= 6) {
//		u->animation_index = 0;
//		return TRUE;
//	}
//
//	// Iris  in next frame
//	_display_frame_appear(u->frame_next_idx, FALSE);
//
//	//
//	_iris_frame_core(u->animation_index);
//
//	// update frame buffer and render it
//	_render_screen();
//
//	return FALSE;
//}
//
//static void _frame_buffer_move_upward(I8U offset)
//{
//	I8U *p0, *p1;
//	I8U i;
//	I8U len = 4 - offset;
//
//	p0 = cling.ui.p_oled_up;
//	p1 = p0+128*offset; // Get offset of sourcing buffer
//
//	for (i = 0; i < len; i++) {
//		memcpy(p0, p1, 128);
//		p0 += 128;
//		p1 += 128;
//	}
//	for (; i < 4; i++) {
//		memset(p0, 0, 128);
//		p0 += 128;
//	}
//}
//
//static void _frame_buffer_move_downward(I8U offset)
//{
//	I8U *p0, *p1;
//	I8U i;
//	I8U len = 4 - offset;
//
//	p0 = cling.ui.p_oled_up+128*3;
//	p1 = p0-128*offset; // Get offset of sourcing buffer
//
//	for (i = 0; i < len; i++) {
//		memcpy(p0, p1, 128);
//		p0 -= 128;
//		p1 -= 128;
//	}
//	for (; i < 4; i++) {
//		memset(p0, 0, 128);
//		p0 -= 128;
//	}
//}
//
//static BOOLEAN _frame_tilting(I8U dir, BOOLEAN b_out)
//{
//	UI_ANIMATION_CTX *u = &cling.ui;
//	I8U shift_pos = 0;
//
//	// Update animation index
//	u->animation_index ++;
//
//	if (u->animation_index >= 4) {
//		u->animation_index = 0;
//		return TRUE;
//	}
//
//	// Shift screen
//	if (b_out) {
//		// Panning out current frame
//		_display_frame_appear(u->frame_prev_idx, FALSE);
//
//		shift_pos = u->animation_index;
//
//		if (dir == TRANSITION_DIR_UP) {
//			_frame_buffer_move_upward(shift_pos);
//		} else {
//			_frame_buffer_move_downward(shift_pos);
//		}
//	} else {
//		// Panning in next frame
//		_display_frame_appear(u->frame_next_idx, FALSE);
//
//		shift_pos = 4 - u->animation_index;
//
//		if (dir == TRANSITION_DIR_UP) {
//			_frame_buffer_move_downward(shift_pos);
//		} else {
//			_frame_buffer_move_upward(shift_pos);
//		}
//	}
//
//	// update frame buffer and render it
//	_render_screen();
//
//	return FALSE;
//}
//
static void _frame_animating()
{
        UI_ANIMATION_CTX *u = &cling.ui;

        N_SPRINTF("[UI] animating: %d, %d, %d, %d, %d",
                u->animation_mode, u->animation_index, u->frame_prev_idx,u->frame_index, u->frame_next_idx);

        switch (u->animation_mode) {
        case ANIMATION_PANNING_OUT:
                if (_frame_panning(u->direction, TRUE)) {
                        u->animation_mode = ANIMATION_PANNING_IN;
                }
                break;
        case ANIMATION_PANNING_IN:
                if (_frame_panning(u->direction, FALSE)) {
                        UI_switch_state(UI_STATE_APPEAR, 0);
                }
                break;
        case ANIMATION_TILTING_OUT:
                //			if (_frame_tilting(u->direction, TRUE)) {
//				u->animation_mode = ANIMATION_TILTING_IN;
//			}
                break;
        case ANIMATION_TILTING_IN:
                //			//if (_frame_tilting(u->direction, FALSE)) {
//			_frame_tilting(u->direction, FALSE);
//			{
//				u->state = UI_STATE_APPEAR;
//			}
                break;
        case ANIMATION_IRIS:
                //			if (_frame_iris()) {
//				UI_switch_state(UI_STATE_APPEAR, 0);
//			}
                break;
        default:
                break;
        }
}

static BOOLEAN _ui_frame_blinking()
{
        UI_ANIMATION_CTX *u = &cling.ui;

        if ((u->frame_index >= UI_DISPLAY_SMART) && (u->frame_index <= UI_DISPLAY_SMART_END)) {
                return FALSE;
        }
        else {
                return TRUE;
        }
}

__attribute__((unused)) static void _display_notifications(UI_ANIMATION_CTX *u, I32U t_curr, I8U type)
{
        u->touch_time_stamp = t_curr;
        if (type == NOTIFICATION_TYPE_MESSAGE) {

#ifdef _ENABLE_ANCS_
                if (cling.notific.cat_id == BLE_ANCS_CATEGORY_ID_INCOMING_CALL)
                u->frame_index = UI_DISPLAY_SMART_INCOMING_CALL;
                else
                u->frame_index = UI_DISPLAY_SMART_INCOMING_MESSAGE;
#endif
                // Go display incoming message/call
                _display_frame_smart(u->frame_index, TRUE);
        }
        else if (type == NOTIFICATION_TYPE_IDLE_ALERT) {
                _display_idle_alert();
        }
        else if (type == NOTIFICATION_TYPE_REMINDER) {
                u->frame_index = UI_DISPLAY_SMART_REMINDER;
                _display_frame_smart(u->frame_index, TRUE);
        }
        N_SPRINTF("[UI] Notification (%d): %d", cling.notific.cat_id, (t_curr-u->touch_time_stamp));

        // Freshing for 5 seconds, then go dark
        u->state = UI_STATE_DARK;
        SYSCLK_restart_timer(ui_timer, 5000);
}
//
//#if 0
//static I8U uistate;
//static I8U ui_frame_idx;
//static BOOLEAN uiactive;
//#endif
//extern I32U sim_started;
extern void CLING_test_notify();

int UI_state_machine()
{
        UI_ANIMATION_CTX *u = &cling.ui;
        I32U t_curr __attribute__((unused)), t_diff __attribute__((unused)), t_threshold __attribute__((unused));
        BOOLEAN b_notify_task = FALSE;

        t_curr = CLK_get_system_time();

        if (!u->display_active) {
                // Do not run UI state machine if oled panel is yet active
                N_SPRINTF("UI: waiting for oled up running %d (%d)", CLK_get_system_time(), u->state);
                N_SPRINTF("UI: Display inactive %d (%d),  %d, %d, %d", cling.time.time_since_1970, cling.time.local.second,
                        u->state, u->display_active, u->frame_index);
                return b_notify_task;
        }
        else {
                N_SPRINTF("UI: new active state: %d @ %d, %d", u->state, CLK_get_system_time(), u->frame_index);
        }

        if (u->state != UI_STATE_IDLE) {

                // Charging display
                if (!LINK_is_authorized())  // 30 seconds if device is yet to be authorized
                        t_threshold = 30; // in second
                else if (u->true_display)
                        t_threshold = cling.user_data.screen_on_general; // in second
                else
                        t_threshold = 2;

                t_threshold *= 1000; // second -> milli-second

                if (t_curr > (u->touch_time_stamp + t_threshold)) {
                        N_SPRINTF("UI: screen time out -> dark!");
                        u->state = UI_STATE_DARK;
                }
        }

        switch (u->state) {
        case UI_STATE_IDLE:
                break;
        case UI_STATE_CLING_START:
                {
#ifdef _CLINGBAND_RAINFLOWER_MODEL_
                _render_display_restart();
#endif
                u->state = UI_STATE_HOME;

                break;
        }
        case UI_STATE_HOME:
                {
                if (LINK_is_authorized()) {

                        if (BATT_is_charging())
                        {
                                N_SPRINTF("[UI] BATTERY CHARGING - %d", cling.system.mcu_reg[REGISTER_MCU_BATTERY]);
                                u->state = UI_STATE_CHARGING;
                        }
                        else {
                                u->state = UI_STATE_APPEAR;
                        }
                        b_notify_task = TRUE;
                        // Let's give UI screen about 4 more seconds once we have clock shown up
                        // Just to be nice!
                        u->touch_time_stamp = t_curr;
                        N_SPRINTF("[UI] home switching to: %d (%d)", u->state, CLK_get_system_time());
                }
                else {
                        u->state = UI_STATE_UNAUTHORIZED;
                        b_notify_task = TRUE;
                        N_SPRINTF("[UI] home switching to unauthorized: %d", CLK_get_system_time());
                }
                break;
        }
        case UI_STATE_CLOCK_GLANCE:
                {
                if (BATT_is_charging()) {
                        //N_SPRINTF("[UI] LOW BATTERY(or charging): %d, %d", cling.system.mcu_reg[REGISTER_MCU_BATTERY], cling.batt.b_usb_attached);
                        u->state = UI_STATE_CHARGING;
                        b_notify_task = TRUE;
                }
                else if (!LINK_is_authorized()) {
                        u->state = UI_STATE_UNAUTHORIZED;
                        b_notify_task = TRUE;
                }
                else {
                        _display_frame_home(TRUE); // Display clock
                        u->state = UI_STATE_APPEAR;
                        u->touch_time_stamp = t_curr;
                        SYSCLK_restart_timer(ui_timer, 500);
                }
                break;
        }
        case UI_STATE_AUTHORIZING:
                {
                if (LINK_is_authorizing()) {
                        _display_authenticating();
                        u->b_icon_blinking++;
                        if (u->b_icon_blinking > 2)
                                u->b_icon_blinking = 0;
                        u->state = UI_STATE_AUTHORIZING;
                        SYSCLK_restart_timer(ui_timer, 500);
                }
                else if (!LINK_is_authorized()) {
                        u->state = UI_STATE_UNAUTHORIZED;
                        b_notify_task = TRUE;
                }
                else {
                        UI_switch_state(UI_STATE_HOME, 0);
                }
                break;
        }
        case UI_STATE_UNAUTHORIZED:
                {
                if (LINK_is_authorizing()) {
                        u->state = UI_STATE_AUTHORIZING;
                        b_notify_task = TRUE;
                }
                else if (!LINK_is_authorized()) {
                        _display_unauthorized_home();
                        SYSCLK_restart_timer(ui_timer, 500);
                }
                else {
                        u->state = UI_STATE_HOME;
                        b_notify_task = TRUE;
                }
                break;
        }
        case UI_STATE_NOTIFICATIONS:
                {
                //_display_notifications(u, t_curr, u->notif_type);
                SYSCLK_restart_timer(ui_timer, 500);
                break;
        }
        case UI_STATE_LOW_POWER:
                {
                N_SPRINTF("[UI] low power (or charging) shown at %d", CLK_get_system_time());
                _display_charging(cling.system.mcu_reg[REGISTER_MCU_BATTERY], TRUE);
                u->state = UI_STATE_APPEAR;
                SYSCLK_restart_timer(ui_timer, 2000);
                break;
        }
        case UI_STATE_CHARGING:
                {
                if (!LINK_is_authorized()) {
                        u->state = UI_STATE_UNAUTHORIZED;
                        b_notify_task = TRUE;
                }
                else {
                        // Charging display

                        if (cling.ui.b_icon_blinking)
                                cling.ui.b_icon_blinking = FALSE;
                        else
                                cling.ui.b_icon_blinking = TRUE;

                        if (cling.system.mcu_reg[REGISTER_MCU_BATTERY] >= 100)
                                cling.ui.b_icon_blinking = TRUE;

                        _display_charging(cling.system.mcu_reg[REGISTER_MCU_BATTERY],
                                cling.ui.b_icon_blinking);

                        SYSCLK_restart_timer(ui_timer, 400);
                }
                break;
        }
        case UI_STATE_TOUCH_SENSING:
                {
                N_SPRINTF("UI: Sensing Gesture: %d, %d", cling.touch.b_valid_gesture, cling.touch.gesture);

                // Don't do anything if system is updating firmware
                if (OTA_if_enabled()) {

                        // Skip all the gestures
                        u->frame_index = UI_DISPLAY_OTA;
                        UI_switch_state(UI_STATE_APPEAR, 1000);
                        u->touch_time_stamp = t_curr;
                        break;
                }
                else if (LINK_is_authorizing()) {
                        UI_switch_state(UI_STATE_UNAUTHORIZED, 1000);
                }
                else if (_ui_touch_sensing()) {
                        N_SPRINTF("[UI] new gesture --- %d", t_curr);
                        u->touch_time_stamp = t_curr;
                        break;
                }
                else if (!LINK_is_authorized()) {
                        // Go to home page if device is unauthorized.
                        UI_switch_state(UI_STATE_UNAUTHORIZED, 1000);
                        break;
                }
                else if (_ui_frame_blinking()) {
                        // Otherwise, blinking every 800 ms.
                        N_SPRINTF("[UI] Go blinking the icon: %d at %d", u->frame_index, CLK_get_system_time());
                        u->state = UI_STATE_APPEAR;
                        SYSCLK_restart_timer(ui_timer, 400);
                        break;
                }
                else {
                        // If we don't see any gesture in 4 seconds, dark out screen
                        SYSCLK_restart_timer(ui_timer, 400);
                }

                break;
        }
        case UI_STATE_ANIMATING:
                // perform frame animation before a final apperance
                _frame_animating();
                break;
        case UI_STATE_APPEAR:
                #if 0
                if (u->frame_index == UI_DISPLAY_PREVIOUS) {
                        u->frame_index = u->frame_cached_index;
                        N_SPRINTF("[UI] load cached index: %d", u->frame_index);
                }
#endif
                // Display this frame.
                _display_frame_appear(u->frame_index, TRUE);
                N_SPRINTF("[UI] Appear normal page at: %d", CLK_get_system_time());
                u->state = UI_STATE_TOUCH_SENSING;
                SYSCLK_restart_timer(ui_timer, 500);
                break;
        case UI_STATE_SOS:
                {
                break;
        }
        case UI_STATE_DARK:
                {
                N_SPRINTF("[UI] screen go dark at: %d", CLK_get_system_time());
                // Turn off OLED panel
                OLED_set_panel_off();
                u->state = UI_STATE_IDLE;
                u->vertical_index = 0;
                u->true_display = FALSE;
                u->b_detail_page = FALSE;
                // Reset alarm clock flag
                cling.reminder.ui_alarm_on = FALSE;
                u->frame_index = 0;

                break;
        }
        default:
                break;
        }
        return b_notify_task;
        if (b_notify_task) {
                UI_task_notify_rendering();
        }
}

void UI_turn_on_display(UI_ANIMATION_STATE state, I32U time_offset)
{
        // Turn on OLED panel
        OLED_set_panel_on();

        // touch time update
        cling.ui.touch_time_stamp = CLK_get_system_time() - time_offset;
        cling.ui.true_display = TRUE;

        if (state != UI_STATE_IDLE) {
                UI_switch_state(state, 0);
        }
        else {
                if (UI_is_idle()) {
                        // UI initial state, A glance of current time
                        UI_switch_state(UI_STATE_CLOCK_GLANCE, 0);
                }
        }
}
//
//void UI_reset_workout_mode()
//{
//	if (cling.ui.frame_index < UI_DISPLAY_STOPWATCH_STOP) {
//		//
//		if (cling.ui.frame_index < UI_DISPLAY_SMART) {
//			cling.activity.workout_type = WORKOUT_NONE;
//			cling.activity.workout_place = WORKOUT_PLACE_NONE;
//			cling.activity.b_workout_active = FALSE;
//		} else if (cling.ui.frame_index > UI_DISPLAY_SMART_END) {
//			cling.activity.workout_type = WORKOUT_NONE;
//			cling.activity.workout_place = WORKOUT_PLACE_NONE;
//			cling.activity.b_workout_active = FALSE;
//		}
//	}
//
//	if (cling.ui.frame_index > UI_DISPLAY_STOPWATCH_CALORIES) {
//		cling.activity.workout_type = WORKOUT_NONE;
//		cling.activity.workout_place = WORKOUT_PLACE_NONE;
//		cling.activity.b_workout_active = FALSE;
//	}
//}
//
//
