/******************************************************************************
 * Copyright 2013-2016 hicling Systems (MikeWang)
 *
 * FileName: clingtask.c
 *
 * Description: cling task in  freertos.
 *
 * Modification history:
 *     2016/7/23, v1.0 create this file mike.
 *******************************************************************************/
#include "main.h"
#include "osal.h"
#include "RTC.h"
#include "flash_hal.h"
#include "clingtask.h"
#include "exti_hal.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "pinmap.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
CLING_MAIN_CTX cling;

/*********************************************************************
 * LOCAL VARIABLES
 */
static TASK_HANDLE_SETS *pClingInput = NULL;
static OS_TIMER Homekey_timer = NULL;
/*********************************************************************
 * LOCAL DECLARATION
 */

void homekey_long_press_callback(OS_TIMER tmr)
{
        /*touch ic hold int line low longer than three second*/
        if(pin_get_status(GPIO_TOUCH_INT) == 0){
                OS_TASK_NOTIFY(pClingInput->input_handle, OS_TASK_SFE_KEY_LONG_PRESSED_BITS, OS_NOTIFY_SET_BITS);
        }
        OS_TIMER_STOP(tmr, 0);
}

void SFE_homekey_timer_callback(OS_TIMER tmr)
{
    OS_TASK_NOTIFY(pClingInput->input_handle, OS_TASK_SFE_KEY_INT_BITS, OS_NOTIFY_SET_BITS);
    OS_TIMER_STOP(tmr, 0);
}

void SFE_batt_notify(void)
{
        OS_TASK_NOTIFY(pClingInput->input_handle, OS_TASK_SFE_BATT_EVT_BITS, OS_NOTIFY_SET_BITS);

}

static CLASS(HalExti) *p_exti_instance = NULL;

void SFE_exti_callback(PinName pin)
{
        // Note: Do NOT logging in exti routine, in addition,
        // Note: Use OS_TASK_NOTIFY_FROM_ISR for notification
        if (pin == ACC_SENSOR_INT) {  // Accelerometer pin index
                if(pClingInput != NULL && pClingInput->cling_core_fsm_handle != NULL){
                        OS_TASK_NOTIFY_FROM_ISR(pClingInput->cling_core_fsm_handle, OS_TASK_FSM_ACC_DATA_INT_BITS, OS_NOTIFY_SET_BITS);
                }
                /*which means 1280 has been is operated by nfc reader */
                if(pin_get_status(GPIO_FILE_DON) == 1){
                        if(pClingInput != NULL && pClingInput->ble_handle != NULL){
                                OS_TASK_NOTIFY_FROM_ISR(pClingInput->ble_handle, BLE_APP_NOTIFY_CLING_PAY_BALANCE_SYNC_MASK, OS_NOTIFY_SET_BITS);
                        }
                }
        } else if (pin == GPIO_TOUCH_INT) {
                if(pClingInput != NULL && pClingInput->input_handle != NULL){
                        OS_TASK_NOTIFY_FROM_ISR(pClingInput->input_handle, OS_TASK_SFE_KEY_INT_BITS, OS_NOTIFY_SET_BITS);
                }
        } else if(pin == GPIO_FILE_DON) {
                if(pClingInput != NULL && pClingInput->ble_handle != NULL){
                        OS_TASK_NOTIFY_FROM_ISR(pClingInput->ble_handle, BLE_APP_NOTIFY_CLING_PAY_BALANCE_SYNC_MASK, OS_NOTIFY_SET_BITS);
                }
        }
}


void SFE_touch_key_init(void)
{
        pin_set_low(GPIO_TOUCH_PWR_EN);
        BASE_delay_msec(20);
        pin_set_high(GPIO_TOUCH_PWR_EN);
        BASE_delay_msec(10);
        CLING_HW_EXTI_INIT(SFE_exti_callback);
        p_exti_instance = HalExti_get_instance();
        p_exti_instance->enable_all(p_exti_instance);
}

/*
 * @brief System Initialization and creation of the BLE task

 */
void cling_task_input(void *pvParameters)
{
        pClingInput = (TASK_HANDLE_SETS *)(pvParameters);
        uint32_t notified_value = 0;
        OS_BASE_TYPE xResult   __attribute__((unused));

        while (1) {

                xResult = OS_TASK_NOTIFY_WAIT(0x0, OS_TASK_NOTIFY_ALL_BITS, &notified_value,
                        OS_TASK_NOTIFY_FOREVER);
                /* Guaranteed to succeed since we're waiting forever for the notification */
                OS_ASSERT(xResult == OS_OK);

                // Touch key event
                if (notified_value & OS_TASK_SFE_KEY_INT_BITS) {
                        if(Homekey_timer==NULL){
                                Homekey_timer = SYSCLK_get_timer(3000, homekey_long_press_callback);
                        }else{
                                SYSCLK_restart_timer(Homekey_timer, 3000);
                        }
                        HOMEKEY_check_on_click_event();
                }

                if(notified_value & OS_TASK_SFE_KEY_LONG_PRESSED_BITS){
                        home_key_long_pressed_detected();
                        HOMEKEY_check_on_click_event();
                }

                if (notified_value & OS_TASK_SFE_BATT_EVT_BITS) {
                        BATT_event_process();
                }

        }
        /* the work of the SysInit task is done */
        OS_TASK_DELETE(OS_GET_CURRENT_TASK());
}

#ifdef __cplusplus
}
#endif
