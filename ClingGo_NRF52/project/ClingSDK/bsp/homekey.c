/***************************************************************************//**
 * 
 * File: homekey.c
 *
 * Description: Home key basics - click, (double click, click and hold)
 *
 * Created on April/29/2015
 * 
 *****************************************************************************/

#include "main.h"
#include "homekey.h"

void HOMEKEY_click_init()
{
}

/****************************************************************************
 Check button status for on-the-fly events.
 ****************************************************************************/
extern void CLING_test_notify();

static I32U sim_count = 0;
static OS_TIMER Homekey_timer = NULL;
extern void SFE_homekey_timer_callback(OS_TIMER tmr);

void HOMEKEY_check_sim()
{
        HOMEKEY_CLICK_STAT *k   __attribute__((unused))= &cling.key;

        if (sim_count < 5) {
                sim_count++;
                cling.touch.b_valid_gesture = TRUE;
                cling.touch.gesture = TOUCH_BUTTON_SINGLE;
                Y_SPRINTF("[HOMEKEY] click at %d", CLK_get_system_time());

                // Turn on OLED panel
                OLED_set_panel_on();

                cling.ui.true_display = TRUE;

                // Update touch event time
                cling.ui.touch_time_stamp = CLK_get_system_time();

                if (UI_is_idle()) {
                        // UI initial state, a glance of current clock
                        UI_switch_state(UI_STATE_CLOCK_GLANCE, 0);
                        Y_SPRINTF("[KEY] set UI: button clicked (glance) at %d", CLK_get_system_time());
                }
                else {
                        //
                        UI_switch_state(UI_STATE_TOUCH_SENSING, 0);
                        Y_SPRINTF("[KEY] set UI: button clicked (sensing) at %d", CLK_get_system_time());
                }

                if (!Homekey_timer) {
                        // Make sure alert timer module is created!
                        Homekey_timer = SYSCLK_get_timer(1000, SFE_homekey_timer_callback);
                        if (!Homekey_timer) {
                                Y_SPRINTF("Reminder: Timer Started Failed ");
                                return;
                        }
                }
                else {
                        SYSCLK_restart_timer(Homekey_timer, 1000);
                }
        }
}

static uint8_t long_key_pressed = FALSE;
void home_key_long_pressed_detected()
{
        long_key_pressed = TRUE;
}

BOOLEAN HOMEKEY_check_on_click_event()
{
        HOMEKEY_CLICK_STAT *k   __attribute__((unused))= &cling.key;

        // Make sure OLED display panel is faced up.
        if (LINK_is_authorized()) {

                cling.touch.gesture = TOUCH_BUTTON_SINGLE;
                Y_SPRINTF("[HOMEKEY] click ------");
                N_SPRINTF("HOMEKEY:  TURN ON SCREEN --------");
                cling.ui.true_display = TRUE;

        }else {
                if(long_key_pressed ==  TRUE){
                        cling.ui.true_display = TRUE;
                }
                Y_SPRINTF("HOMEKEY:  Key pressed");
        }

        if (cling.ui.true_display == TRUE) {
                /*disable key long pressed mark bit*/
                long_key_pressed = FALSE;
                // Turn on OLED panel
                OLED_set_panel_on();

                // Update touch event time
                cling.ui.touch_time_stamp = CLK_get_system_time();

                if (UI_is_idle()) {
                        // UI initial state, a glance of current clock
                        UI_switch_state(UI_STATE_CLOCK_GLANCE, 0);
                        Y_SPRINTF("[KEY] set UI: button clicked (glance)");
                        cling.touch.b_valid_gesture = FALSE;
                }
                else if (!BATT_is_charging()) {
                        UI_switch_state(UI_STATE_TOUCH_SENSING, 0);
                        Y_SPRINTF("[KEY] set UI: button clicked (sensing)");
                        cling.touch.b_valid_gesture = TRUE;
                }
                return TRUE;
        }
        return FALSE;
}

