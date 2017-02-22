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
static TASK_HANDLE_SETS *pClingOutput = NULL;

/*********************************************************************
 * LOCAL DECLARATION
 */

void UI_task_notify_get_balance()
{
        OS_TASK_NOTIFY(pClingOutput->ble_handle, BLE_APP_NOTIFY_CLING_PAY_BALANCE_SYNC_MASK,
                OS_NOTIFY_SET_BITS);
}
void UI_task_notify_oled()
{
        OS_TASK_NOTIFY(pClingOutput->output_handle, OS_TASK_UI_OLED_PANEL_BITS, OS_NOTIFY_SET_BITS);
}

void UI_oled_timer_callback(OS_TIMER tmr)
{
        OS_TASK_NOTIFY(pClingOutput->output_handle, OS_TASK_UI_OLED_PANEL_BITS, OS_NOTIFY_SET_BITS);
        OS_TIMER_STOP(tmr, 0);
}

void UI_task_notify_rendering()
{
        OS_TASK_NOTIFY(pClingOutput->output_handle, OS_TASK_UI_RENDER_FRAME_BITS,
                OS_NOTIFY_SET_BITS);
}

void UI_rendering_timer_callback(OS_TIMER tmr)
{
        OS_TASK_NOTIFY(pClingOutput->output_handle, OS_TASK_UI_RENDER_FRAME_BITS,
                OS_NOTIFY_SET_BITS);
        OS_TIMER_STOP(tmr, 0);
}

void ALERT_reminder_timer_callback(OS_TIMER tmr)
{
        OS_TASK_NOTIFY(pClingOutput->output_handle,
                OS_TASK_ALERT_REMINDER_BITS, OS_NOTIFY_SET_BITS);
        OS_TIMER_STOP(tmr, 0);
}

void ALERT_task_reminder_notify()
{
        OS_TASK_NOTIFY(pClingOutput->output_handle, OS_TASK_ALERT_REMINDER_BITS,
                OS_NOTIFY_SET_BITS);
}

void ALERT_task_notific_notify()
{
        OS_TASK_NOTIFY(pClingOutput->output_handle, OS_TASK_ALERT_NOTIFIC_BITS, OS_NOTIFY_SET_BITS);
}

void ALERT_notific_timer_callback(OS_TIMER tmr)
{
        OS_TASK_NOTIFY(pClingOutput->output_handle,
                OS_TASK_ALERT_NOTIFIC_BITS, OS_NOTIFY_SET_BITS);
        OS_TIMER_STOP(tmr, 0);
}

void ALERT_task_idle_notify()
{
        NOTIFIC_start_idle_alert();
        OS_TASK_NOTIFY(pClingOutput->output_handle, OS_TASK_ALERT_NOTIFIC_BITS, OS_NOTIFY_SET_BITS);
}


void notify_ble_to_switch_adv_frequnce()
{
        /*notify ble task to switch to fast adv mode when touch is actived*/
        if(pClingOutput != NULL && pClingOutput->ble_handle != NULL){
                OS_TASK_NOTIFY(pClingOutput->ble_handle, BLE_APP_NOTIFY_CLING_STRAT_ADV_REQUEST_MASK, OS_NOTIFY_SET_BITS);
        }
}
int CLNGPAY_balance_callback(uint32_t balance)
{
#define BALANCE_BACK_TO_ZERO_VOTE_THREAHOLD   2
        int diff = (int32_t)(balance) - (int32_t)(cling.system.balance);
        static uint8_t zero_vote = 0;
        /*somtimes,when something wrong happened in z32, this result retruned gonna be 0, this situation has to be told away*/
        if (balance == 0) {
                if (diff < -50) {
                        zero_vote = 0;
                        cling.system.balance = balance;
                }
                else {
                        zero_vote++;
                        if (zero_vote > BALANCE_BACK_TO_ZERO_VOTE_THREAHOLD) {
                                cling.system.balance = balance;
                        }
                }
        }
        else {
                zero_vote = 0;
                cling.system.balance = balance;
        }
        return 1;
}

/*
 * @brief System Initialization and creation of the BLE task

 */
void cling_task_output(void *pvParameters)
{
        pClingOutput = (TASK_HANDLE_SETS*)(pvParameters);
        uint32_t notified_value = 0;
        OS_BASE_TYPE xResult   __attribute__((unused));

        while (1) {
                xResult = OS_TASK_NOTIFY_WAIT(0x0, OS_TASK_NOTIFY_ALL_BITS, &notified_value,
                        OS_TASK_NOTIFY_FOREVER);
                /* Guaranteed to succeed since we're waiting forever for the notification */
                OS_ASSERT(xResult == OS_OK);

                if (notified_value & OS_TASK_UI_RENDER_FRAME_BITS) {
                        // UI animation update
                        while(UI_state_machine());
                }

                if (notified_value & OS_TASK_UI_OLED_PANEL_BITS) {
                        // OLED state machine
                        while(OLED_state_machine());
                }

                if (notified_value & OS_TASK_ALERT_REMINDER_BITS) {
                        // Reminder state machine
                        while(REMINDER_state_machine());
                }

                if (notified_value & OS_TASK_ALERT_NOTIFIC_BITS) {
                        // Notific state machine
                        while(NOTIFIC_state_machine());
                }

                if (notified_value & OS_TASK_PAYMENT_COMM_SEND_BITS) {
                        // When a commend is ready to send
                        Y_SPRINTF("PAYMENT: Send a payment Z32 7816 command");
                }
        }
        /* the work of the SysInit task is done */
        OS_TASK_DELETE(OS_GET_CURRENT_TASK());
}

#ifdef __cplusplus
}
#endif
