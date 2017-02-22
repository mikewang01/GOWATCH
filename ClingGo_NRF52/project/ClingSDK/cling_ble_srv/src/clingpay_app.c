/******************************************************************************
 * Copyright 2013-2016 HIcling Systems (Mike)
 *
 * FileName: clingpay_cmd.c
 *
 * Description: entry file of user application
 *
 * Modification history:
 *     2016/1/1, v1.0 create this file.
 *******************************************************************************/
#include "clingpay_cmd.h"
#include "clingpay_mac.h"
#include "stdio.h"
#include <unistd.h>
#include "main.h"
#include "platform_devices.h"
#include "ad_uart.h"
#include "ble_gap_mgr.h"
#include "clingpay_app.h"
/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
enum{
   BALANCE_FSM_BEGIN = 0,
   BALANCE_FSM_POWER_UP,
   BALANCE_FSM_GET_BALANCE,
   BALANCE_FSM_END,
};
/*********************************************************************
 * static VARIABLES
 */
static int payment_process(struct pay_rev_package *p_pkg);
static uint8_t get_balance_state = BALANCE_FSM_BEGIN;
static  int(*retrive_balance_cb)(uint32_t balance) = NULL;
static TASK_HANDLE_SETS *hdls = NULL;
static uint32 balance = 0;
/*********************************************************************
 * EXTERNAL VARIABLES
 */
int cling_pay_app_init(TASK_HANDLE_SETS *sets)
{
        hdls = sets;
        return reg_cling_pay_cmd_callback(payment_process);
}



static int balance_process_cb(uint8_t *p_buf, uint16_t length)
{
        switch(get_balance_state){
        case BALANCE_FSM_BEGIN:
                if(p_buf == NULL){
                        OS_TASK_NOTIFY_FROM_ISR(hdls->ble_handle, BLE_APP_NOTIFY_CLING_PAY_BALANCE_MASK, eSetBits);
                }
                break;
        case BALANCE_FSM_POWER_UP:
                if(p_buf == NULL){
                        OS_TASK_NOTIFY_FROM_ISR(hdls->ble_handle, BLE_APP_NOTIFY_CLING_PAY_BALANCE_MASK, eSetBits);
                }
                break;
        case BALANCE_FSM_GET_BALANCE:
                balance = 0;
                for(int i = 0; i< length; i++){
                        balance = ((balance<<8)|p_buf[i]);
                }
                OS_TASK_NOTIFY_FROM_ISR(hdls->ble_handle, BLE_APP_NOTIFY_CLING_PAY_BALANCE_MASK, eSetBits);
                break;
        case BALANCE_FSM_END:break;

        }
        return 0;
}

int get_balance_fsm()
{
      switch(get_balance_state){
        case BALANCE_FSM_BEGIN:{
                get_balance_state = BALANCE_FSM_POWER_UP;
                Clingpay_Activate_ISO7816();
        }break;
        case BALANCE_FSM_POWER_UP:{
                get_balance_state = BALANCE_FSM_GET_BALANCE;
                uint16_t len;
                Clingpay_Retrieve_Balance(NULL, &len);
        }
        break;
        case BALANCE_FSM_GET_BALANCE:
                Y_SPRINTF("[clingpat app]balance = %d/r/n",  balance);
                if(retrive_balance_cb != NULL){
                        retrive_balance_cb(balance);
                }
                Clingpay_Deactivate_ISO7816();
                get_balance_state = BALANCE_FSM_END;
                break;
        case BALANCE_FSM_END:break;

        }
       return 0;
}

int get_balance(int(*cb)(uint32_t balance))
{
        if(cb != NULL){
                retrive_balance_cb = cb;
        }
        get_balance_state = BALANCE_FSM_BEGIN;
        set_z32_rev_cb(balance_process_cb);
        Clingpay_Activate_ISO7816();
        get_balance_state = BALANCE_FSM_BEGIN;

        return 0;
}




/*used to implement state machine for dialog communicaiton*/
static int payment_process(struct pay_rev_package *p_pkg)
{
        // When a commend is ready to send
//                        Y_SPRINTF("PAYMENT: Send a payment Z32 7816 command");
//          p_pkg->header.data_lenth = 0;
          if (p_pkg->header.cmd_type == BTC_APDU) {
                    uint16_t len = p_pkg->header.data_lenth;
                    Clingpay_Send_APDU(p_pkg->data.body, &(len));
                    p_pkg->header.data_lenth = len;
          }
          else if (p_pkg->header.cmd_type == BTC_DISCONNECT) {
                  Clingpay_Deactivate_ISO7816();
          }
          else if (p_pkg->header.cmd_type == BTC_CONNECT) {
                  Clingpay_Activate_ISO7816();
          }
          else if (p_pkg->header.cmd_type == BTC_ATR) {
                  uint16_t len = p_pkg->header.data_lenth;
                  Clingpay_RETRIEVE_ATR(p_pkg->data.body, &(len));
                  p_pkg->header.data_lenth = len;
          }else if(p_pkg->header.cmd_type == BTC_BIND){
#if 0
                  Y_SPRINTF("[phone id]:");
                  for(int i = 0; i< p_pkg->header.data_lenth; i++){
                              printf("0x%02x ", p_pkg->data.body[i]);
                  }
                  Y_SPRINTF("\r\n");
#endif
                  CLASS(GapMgr)*gap_mgr = GapMgr_get_instance();
                  size_t dev_id_length;
                  gap_mgr->get_dev_id_str(gap_mgr, p_pkg->data.body , &dev_id_length);
                  p_pkg->header.data_lenth =  dev_id_length;
                  Y_SPRINTF("BTC_DEV_ID = %s", p_pkg->data.body);
                  /*this is not neccesary any how  ,this is put here for the reponse from z32 adn then  dialog is
                   *able to generate one feedback to app*/
                  Clingpay_Activate_ISO7816();
          }else if(p_pkg->header.cmd_type == BTC_QR_CODE){
                  Y_SPRINTF("BTC_QR_CODE REVED");
//                  printf("BTC_QR_CODE: ");
//                  for(int i = 0; i < p_pkg->header.data_lenth; i++){
//                          printf("0x%02x ", p_pkg->data.body[i]);
//                  }
//                  printf("\r\n");
                  CLASS(GapMgr)*gap_mgr = GapMgr_get_instance();
                  size_t dev_id_length;
                  gap_mgr->get_dev_id_str(gap_mgr, p_pkg->data.body , &dev_id_length);
                  p_pkg->header.data_lenth =  dev_id_length;

          }else if(p_pkg->header.cmd_type == BTC_QR_IMAGE){
                  Y_SPRINTF("BTC_QR_IMAGE RECIEVED");
//                  printf("BTC_QR_IMAGE: ");
//                  for(int i = 0; i < p_pkg->header.data_lenth; i++){
//                          printf("0x%02x ", p_pkg->data.body[i]);
//                  }
//                  printf("\r\n");
                  CLASS(GapMgr)*gap_mgr = GapMgr_get_instance();
                  size_t dev_id_length;
                  gap_mgr->get_dev_id_str(gap_mgr, p_pkg->data.body , &dev_id_length);
                  p_pkg->header.data_lenth =  dev_id_length;
          }
          return 0;
}
