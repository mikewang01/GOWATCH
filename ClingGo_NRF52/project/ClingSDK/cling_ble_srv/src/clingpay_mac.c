/******************************************************************************
 * Copyright 2013-2014 Espressif Systems (Wuxi)
 *
 * FileName: user_json.c
 *
 * Description: JSON format set up and parse.
 *              Check your hardware transmation while use this data format.
 *
 * Modification history:
 *     2014/5/09, v1.0 create this file.
 *******************************************************************************/
#include "clingpay_mac.h"
#include "clingpay_cmd.h"
#include "stdio.h"
#include "assert.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h> 
#include <errno.h>
#include "ble_clingpay.h"
#include "main.h"
#include "osal.h"



/*LOOP TEST SWITCH*/
#define CLING_PAY_LOOP_TEST_ENABLED  (0)
#define PACKAGE_HEADER_SYMBOL    0X7D
#define PACKAGE_TRANSFER_SYMBOL  0X7F
#define PACKAGE_END_SYMBOL		 0X7E

#define DATA_0X7D_TRANSFERRED	 0X01
#define DATA_0X7E_TRANSFERRED	 0X02
#define DATA_0X7F_TRANSFERRED	 0X03

/*SEMAPHORE RELATED*/
#define SEMAPHORE 	 int
#define LOCKED		 1
#define UNLOCKED       0
#define IS_SEMAPHORE_LOCKED(__x) (__x == LOCKED)
#define LOCK_SEMAPHORE(__X) 	 (__X =  LOCKED)
#define UNLOCK_SEMAPHORE(__x)	 (__x = UNLOCKED)

#if 0
#define  MAC_SEND_BUFFER(__X, __Y) do{\
	if(reg_serial_fd > 0){\
		if(write(reg_serial_fd, __X,__Y) < 0);\
			Y_SPRINTF("serial write error numbeer =%d\n",errno);\
	}\
}while(0)
#endif

#define  MAC_SEND_BUFFER(__X, __Y) do{\
	if(conn_hdl != BLE_CONN_HANDLE_INVALID){\
		clingpay_cus_notify_send(pay_svc, conn_hdl, 0xFFE1, (uint8_t *)(__X), (__Y));\
	}\
}while(0)

#define  ENTER_CRITICAL_SECTION() 
#define  LEAVE_CRITICAL_SECTION() 

/*THE UPPER LIMIT OF TIMES THE DATA BEEN SENDED*/

#define  RESEND_TIMES_UPPER_THREADHOLD  0

#define BLE_CONN_HANDLE_INVALID 0xFFFF  /**< Invalid Connection Handle. */
/*********************************************************************
 * TYPEDEFS
 */
typedef enum {
        EMPTY,
        UNDER_WRITE,
        WRITE_OVER
} RcvMsgBuffState;

typedef struct {
        unsigned int RcvBuffSize;
        unsigned char pRcvMsgBuff[1024];
        unsigned char *pWritePos;
        unsigned char *pReadPos;
        unsigned char TrigLvl; //JLU: may need to pad
        RcvMsgBuffState BuffState;
} RcvMsgBuff;

enum package_send_state {
        PACKAGE_WAITING_FOR_SENDDING = 0,
        PACKAGE_WAITING_FOR_ACK,
        PACKAGE_SENDDED_SUCCESSFULLY

};

/*used to implement state machine*/
enum package_state {
        PACKAGE_BEGIN = 0,
        PACKAGE_TRANSFER,
        PACKAGE_PAYLOAD,
        PACKAGE_END,
        PACKAGE_NULL
};

enum package_type {
        PACKAGE_ACK = 0,
        PACKAGE_DATA,
        PACKAGE_CMD

};

/*********************************************************************
 * GLOBAL VARIABLES
 */
#define MESSAGE_MAX_CHAIN_LENTH 	8
#define CLING_TX_UART_FIFO_POLL_PERIOAD 100

/*********************************************************************
 * static VARIABLES
 */

static int reg_serial_fd = 0;
/*send data call back function*/
static int (*mac_send_data)(char*, unsigned int) = NULL;

static RcvMsgBuff serial_buff;

static uint16_t conn_hdl = BLE_CONN_HANDLE_INVALID;/**< Invalid Connection Handle. */;
static ble_service_t *pay_svc = NULL;

/*recieve fifo related pointer*/
static enum package_state current_state = PACKAGE_NULL;
static struct mac_layer_payload_rev *rev_payload_list_header = NULL;
static struct mac_layer_payload_rev *rev_payload_list_tail = NULL;

static void (*cmd_recieved_callback)(char cmd) = NULL;
static void (*data_recieved_callback)(uint8_t *buf, int size) = NULL;

static SEMAPHORE rev_fifo_semphore = UNLOCKED;

/**/
static struct mac_layer_payload_send *send_package_list_header = NULL;
static struct mac_layer_payload_send *send_package_list_tail   __attribute__((unused)) = NULL;
static SEMAPHORE send_fifo_semphore = UNLOCKED;

//static os_timer_t  uart_send_timer;
static int macsend_ack_recieved();
static int send_package_assemble(struct mac_layer_payload_send *payload_temp,
        enum package_type type);
static int add_payload2revlist(unsigned char *pbuffer, int size);
static void received_data_process(RcvMsgBuff *para);
static int  on_disconnect();
static int  on_connect(ble_service_t *svc, uint16_t conn_idx);
static OS_TIMER send_demo_timer;
static TASK_HANDLE_SETS *hanlde_sets = NULL;


static void isr_routine(OS_TIMER t)
{
        N_SPRINTF("[ClingPay] trigger sending demon\r\n");
        if(hanlde_sets != NULL){
                if(hanlde_sets->ble_handle != NULL){

                        OS_TASK_NOTIFY(hanlde_sets->ble_handle ,BLE_APP_NOTIFY_CLING_PAY_MASK, eSetBits);
                }
        }
        return ;
}

int clingpay_mac_fsm()
{
     mac_sendlist_mantain_demon();
     return 1;
}

int clingpay_mac_init(TASK_HANDLE_SETS *sets)
{

        hanlde_sets = sets;
        Y_SPRINTF("[ClingPay]: Cling pay init");
        clingpay_cmd_init(hanlde_sets);
        /*register callback funtion*/
        //app_cust1_val_wr_callback_reg(ble_rev_process);
//      cling_service_reg_write_callback(ble_rev_process);
        clingpay_service_reg_conn_callback(on_connect);
        clingpay_service_reg_disconn_callback(on_disconnect);

        send_demo_timer = NULL;
        send_demo_timer = OS_TIMER_CREATE("pay_timer", OS_MS_2_TICKS(10000), pdTRUE, (void*)0, isr_routine);

        return 1;
}


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
#ifndef _CLING_PC_SIMULATION_
static int on_connect(ble_service_t *svc, uint16_t conn_idx)
{

        conn_hdl = conn_idx;
        pay_svc = svc;
        Y_SPRINTF("[PAY MAC] Connected! ");
        return 0;
}
#endif

/*
 **@brief Function for handling the Disconnect event.
 *
 */
static int on_disconnect()
{

        conn_hdl = BLE_CONN_HANDLE_INVALID;
        pay_svc = NULL;
        Y_SPRINTF("[PAY MAC] DisConnected! ");
        return 0;
}


/******************************************************************************
 * FunctionName : receive_one_char_callback
 * Description  : call back function when received one char
 * Parameters   : unsigned char rev_char
 * Returns      : none
 *******************************************************************************/
int  receive_one_char_callback(unsigned char rev_char)
{

        RcvMsgBuff *para = &serial_buff;
#if 0
        MAC_SEND_BUFFER(&rev_char, 1);
#endif
//        printf("++0x%02x ", rev_char);
        /*this is a state machien from now on*/
        switch (current_state) {
        case PACKAGE_END:
                case PACKAGE_NULL: {
                if (rev_char == PACKAGE_HEADER_SYMBOL) {
                        /*change current state machine*/
                        current_state = PACKAGE_BEGIN;
                        /*reset message buffer write pointer*/
                        para->pWritePos = para->pRcvMsgBuff;
                        para->pReadPos = para->pRcvMsgBuff;
                        para->BuffState = EMPTY;
                }
        }
                break;

        case PACKAGE_BEGIN: {
                if (rev_char == PACKAGE_HEADER_SYMBOL) {
                        /*change current state machine*/
                        current_state = PACKAGE_BEGIN;
                        /*reset message buffer write pointer*/
                        para->pWritePos = para->pRcvMsgBuff;
                        para->pReadPos = para->pRcvMsgBuff;
                        para->BuffState = EMPTY;

                }
                else if (rev_char == PACKAGE_END_SYMBOL) {
                        /*change current state machine*/
                        current_state = PACKAGE_END;

                }
                else if (rev_char == PACKAGE_TRANSFER_SYMBOL) { /*data need transferred*/

                        /*change current state machine*/
                        current_state = PACKAGE_TRANSFER;
                }
                else { /*normal data*/

                        para->BuffState = UNDER_WRITE;
                        /*change current state machine*/
                        current_state = PACKAGE_PAYLOAD;
                        *(para->pWritePos) = rev_char;
                        para->pWritePos++;
                }

        }
                break;

        case PACKAGE_TRANSFER: {
                if (rev_char == PACKAGE_HEADER_SYMBOL) {
                        /*change current state machine*/
                        current_state = PACKAGE_BEGIN;
                        /*reset message buffer write pointer*/
                        para->pWritePos = para->pRcvMsgBuff;
                        para->pReadPos = para->pRcvMsgBuff;
                        para->BuffState = EMPTY;

                }
                else if (rev_char == PACKAGE_END_SYMBOL) {
                        /*change current state machine*/
                        current_state = PACKAGE_END;

                }
                else if (rev_char == PACKAGE_TRANSFER_SYMBOL) { /*data need transferred*/

                        /*change current state machine abandon cuurent pakage*/
                        current_state = PACKAGE_END;
                }
                else { /*normal data*/
                        /*this means transferred data recieved*/
                        if (rev_char == DATA_0X7D_TRANSFERRED || rev_char == DATA_0X7E_TRANSFERRED
                                || rev_char == DATA_0X7F_TRANSFERRED) {
                                para->BuffState = UNDER_WRITE;
                                /*change current state machine*/
                                current_state = PACKAGE_PAYLOAD;
                                *(para->pWritePos) = 0x7d + rev_char - 1;
                                para->pWritePos++;

                        }
                        else {
                                /*change current state machine abandon current pakage*/
                                current_state = PACKAGE_END;

                        }
                }

        }
                break;

        case PACKAGE_PAYLOAD: {
                if (rev_char == PACKAGE_HEADER_SYMBOL) {
                        /*change current state machine*/
                        current_state = PACKAGE_BEGIN;
                        /*reset message buffer write pointer*/
                        para->pWritePos = para->pRcvMsgBuff;
                        para->pReadPos = para->pRcvMsgBuff;
                        para->BuffState = EMPTY;

                }
                else if (rev_char == PACKAGE_END_SYMBOL) {

                        para->BuffState = WRITE_OVER;
                        current_state = PACKAGE_END;
                        received_data_process(para);
                        return 0;
                        /*change current state machine*/

                }
                else if (rev_char == PACKAGE_TRANSFER_SYMBOL) { /*data need transferred*/

                        /*change current state machine abandon cuurent pakage*/
                        current_state = PACKAGE_TRANSFER;
                }
                else { /*normal data*/
                        /*this means transferred data recieved*/

                        *(para->pWritePos) = rev_char;
                        para->pWritePos++;

                }

        }
                break;

        default:
                break;
        }
#if 0
        MAC_SEND_BUFFER((u8*)&current_state, 1);
#endif
        return 1;
}
/******************************************************************************
 * FunctionName : receive_one_char_callback
 * Description  : call back function when received one char
 * Parameters   : unsigned char rev_char
 * Returns      : none
 *******************************************************************************/
void set_recieved_cmd_call_back(void (*callback)(char))
{

        cmd_recieved_callback = callback;

}
/******************************************************************************
 * FunctionName : receive_data call bacl
 * Description  : call back function when received one char
 * Parameters   : unsigned char rev_char
 * Retu rns      : none
 *******************************************************************************/
void set_recieved_data_call_back(void (*callback)(uint8_t*, int ))
{

        if(callback != NULL){
                data_recieved_callback = callback;
        }

}

/******************************************************************************
 * FunctionName : receive_one_char_callback
 * Description  : call back function when received one char
 * Parameters   : unsigned char rev_char
 * Returns      : none
 *******************************************************************************/

static void received_data_process(RcvMsgBuff *para)
{

        /*this means this is a ack package*/
        if (para->pRcvMsgBuff[0] == PACKAGE_ACK) {
//                macsend_ack_recieved();
                Y_SPRINTF("ack recieved\n");
        }
        else if (para->pRcvMsgBuff[0] == PACKAGE_DATA) {
                /*add messge to reveived messge list waiting for processing*/
//                send_package_assemble(NULL, PACKAGE_ACK);
                add_payload2revlist(para->pRcvMsgBuff + 1,
                        (para->pWritePos - para->pRcvMsgBuff - 1));

                /*after receieving ,send back ack package*/

#if CLING_PAY_LOOP_TEST_ENABLED
                mac_send_payload(para->pRcvMsgBuff + 1, para->pWritePos - para->pRcvMsgBuff - 1);
#endif
        }
        else if (para->pRcvMsgBuff[0] == PACKAGE_CMD) {

                send_package_assemble(NULL, PACKAGE_ACK);
                if (cmd_recieved_callback != NULL) {
                        cmd_recieved_callback(para->pRcvMsgBuff[1]);
                }
                /*for the real time purpose, call back function is prefered*/
                /*after receieving ,ssend back ack package*/
        }

}

/******************************************************************************
 * FunctionName : add_payload2list
 * Description  : internally usedly add a data struct to the chain list
 * Parameters   : unsigned char rev_char
 * Returns      : none
 *******************************************************************************/
static int add_payload2revlist(unsigned char *pbuffer, int size)
{

        Y_SPRINTF("add_payload2revlist\n");
        /*if this lock is occupied by other process then return*/
        if (IS_SEMAPHORE_LOCKED(rev_fifo_semphore)) {
                return -1;
        }
#if 0
        Y_SPRINTF("data=");
        for(int i =0;i< size/16 + 1;i++) {
                for(int j = 0; j< 16; j++){
                        if(i*16 + j > size){
                                break;
                        }
                        printf("0x%02x ",pbuffer[i*16 + j]);
                }
                printf("i =  %d\r\n ", (i+1)*16);
        }
#endif

        /*enter critical section so that this is a atom acess*/
        LOCK_SEMAPHORE(rev_fifo_semphore);
//        cling_data_mac_pocess(pbuffer, size);
        if(data_recieved_callback != NULL){
                data_recieved_callback(pbuffer, size);
        }else{

                goto FAILED;
        }
        /*leave criticla section ,release semphore here*/
        UNLOCK_SEMAPHORE(rev_fifo_semphore);
        return 0;
        FAILED:
        /*leave criticla section ,release semphore here*/
        UNLOCK_SEMAPHORE(rev_fifo_semphore);
        return -1;
}

/******************************************************************************
 * FunctionName : obbtain_payload_from_revlist
 * Description  : internally used ,obbtain a data struct to the chain list
 * Parameters   : struct mac_layer_payload_rev *ppayload: mac layer layload pointer
 * Returns      : none
 *******************************************************************************/
int obtain_payload_from_revlist(struct mac_layer_payload_rev **ppayload)
{

        /*if this lock is occupied by other process then return*/
        if (IS_SEMAPHORE_LOCKED(rev_fifo_semphore)) {
                return -1;
        }

        /*enter critical section to lock semphore so to make sure this is a atom acess*/
        LOCK_SEMAPHORE(rev_fifo_semphore);

        struct mac_layer_payload_rev *ptemp = rev_payload_list_header;
        /*initialte parameter passed to this function*/
        *ppayload = NULL;
        /*if no data existed*/
        if (ptemp == NULL) {

                goto FAILED;

        }
        else {
                /*there is only one data struct exsited in fifo*/
                if (ptemp == rev_payload_list_tail) {
                        *ppayload = ptemp;
                        rev_payload_list_header = NULL;
                        rev_payload_list_tail = NULL;
                        goto SUCESS;
                }
                /*more than two data block existed, SO first step is to find the the last data block*/
                for (; ptemp->next != rev_payload_list_tail; ptemp = ptemp->next)
                        ;
                /*assign the last */
                *ppayload = rev_payload_list_tail;
                rev_payload_list_tail = ptemp;
                ptemp->next = NULL;
        }
        SUCESS:
        /*leave criticla section ,release semphore here*/
        UNLOCK_SEMAPHORE(rev_fifo_semphore);
        return 0;
        FAILED:
        /*leave criticla section ,release semphore here*/
        UNLOCK_SEMAPHORE(rev_fifo_semphore);
        return -1;
}

/******************************************************************************
 * FunctionName : obbtain_payload_from_revlist
 * Description  : internally used ,obbtain a data struct to the chain list
 * Parameters   : struct mac_layer_payload_rev *ppayload: mac layer layload pointer
 * Returns      : none
 *******************************************************************************/
int mac_send_payload(char *ppayload, int lenth)
{

        //Y_SPRINTF("==============ble fota============%d\n", lenth);
        /*if this lock is occupied by other process then return*/
        if (IS_SEMAPHORE_LOCKED(send_fifo_semphore)) {
                return -1;
        }

#if 0
        int size = lenth;
        Y_SPRINTF("send data=");
        for(int i =0;i< size/16 + 1;i++) {
                for(int j = 0; j< 16; j++){
                        if(i*16 + j > size){
                                break;
                        }
                        printf("0x%02x ",ppayload[i*16 + j]);
                }
                printf("i =  %d\r\n ", (i+1)*16);
        }
#endif
        /*enter critical section to lock semphore so to make sure this is a atom acess*/
        LOCK_SEMAPHORE(send_fifo_semphore);
        /*check  heap is enough or not*/

        struct mac_layer_payload_send *payload_temp = (struct mac_layer_payload_send*)OS_MALLOC_FUNC(
                sizeof(struct mac_layer_payload_send));
        if (NULL == payload_temp) {
                goto FAILED;
        }
        /*RESET RESEND TIMES*/
        payload_temp->resend_times = 0;

#if 1
         payload_temp->ppayload = (unsigned char*)OS_MALLOC_FUNC(lenth);
        if (NULL == payload_temp->ppayload) {
                goto FAILED;
        }
#endif
        /*copy data from buffer*/
        memcpy(payload_temp->ppayload, ppayload, lenth);
        payload_temp->state = PACKAGE_WAITING_FOR_SENDDING;
        payload_temp->lenth = lenth;
        payload_temp->next = NULL;
#if 0
        /**/
        if (NULL == send_package_list_header) {

                send_package_list_header = payload_temp;
                /*if there is no more data listed in chain send this immmidiately*/
                payload_temp->state = PACKAGE_WAITING_FOR_ACK;/*this must be set v=before being sended*/
                send_package_assemble(send_package_list_header, PACKAGE_DATA);
                OS_TIMER_START(send_demo_timer, 0);
                /*restart flag updating progress*/
                //os_timer_disarm(&(uart_send_timer));
                //os_timer_setfn(&(uart_send_timer), (os_timer_func_t *)mac_sendlist_mantain_demon, 0);
                //os_timer_arm(&(uart_send_timer), CLING_TX_UART_FIFO_POLL_PERIOAD, 0);
        }
        else {

                send_package_list_tail->next = payload_temp;
        }
        /*rewind the pointer to the end of chain*/
        send_package_list_tail = payload_temp;
#endif
        send_package_assemble(payload_temp, PACKAGE_DATA);
        OS_FREE_FUNC(payload_temp->ppayload);
        OS_FREE_FUNC(payload_temp);

        /*leave criticla section ,release semphore here*/
        UNLOCK_SEMAPHORE(send_fifo_semphore);
        return 0;
        FAILED:
        OS_FREE_FUNC(payload_temp->ppayload);
        OS_FREE_FUNC(payload_temp);
        /*leave criticla section ,release semphore here*/
        UNLOCK_SEMAPHORE(send_fifo_semphore);
        return -1;

}

/******************************************************************************
 * FunctionName : obbtain_payload_from_revlist
 * Description  : internally used ,obbtain a data struct to the chain list
 * Parameters   : struct mac_layer_payload_rev *ppayload: mac layer layload pointer
 * Returns      : none
 *******************************************************************************/
int mac_sendlist_mantain_demon()
{

        struct mac_layer_payload_send *payload_temp = send_package_list_header;
        /*if this lock is occupied by other process then return*/
        if (IS_SEMAPHORE_LOCKED(send_fifo_semphore)) {
                goto FAILED;
        }
        /*enter critical section to lock semphore so to make sure this is a atom acess*/
        LOCK_SEMAPHORE(send_fifo_semphore);
        /**/
        if (NULL == payload_temp) {
                OS_TIMER_STOP(send_demo_timer, 0);
                Y_SPRINTF("[CLING PAY MAC]payload_temp = NULL");
                /*if the fifo ha already been empty,then it is not gonna rearm the timer again*/
                goto SUCESS;

        }
        else {
                /*current data need to be transmitted*/
                if (payload_temp->state == PACKAGE_WAITING_FOR_SENDDING) {
                        OS_TIMER_STOP(send_demo_timer, 0);
                        send_package_assemble(payload_temp, PACKAGE_DATA);
                        payload_temp->resend_times = 0;
                        /*send data over here*/
                        payload_temp->state = PACKAGE_WAITING_FOR_ACK;
                        OS_TIMER_START(send_demo_timer, 0);

                }
                else if (payload_temp->state == PACKAGE_WAITING_FOR_ACK) {

                        if (payload_temp->resend_times < RESEND_TIMES_UPPER_THREADHOLD) {
                                OS_TIMER_STOP(send_demo_timer, 0);
                                Y_SPRINTF("[CLING PAY MAC]send again\n");
                                send_package_assemble(payload_temp, PACKAGE_DATA);
                                // MAC_SEND_BUFFER(payload_temp->, position);
                                payload_temp->resend_times++;
                                OS_TIMER_START(send_demo_timer, 0);
                        }
                        else {
                                OS_TIMER_STOP(send_demo_timer, 0);
                                Y_SPRINTF("[CLING PAY MAC]send time up\n");
                                /*remove the packege sended successfully*/
                                send_package_list_header = send_package_list_header->next;
                                /*OS_FREE_FUNC payload buffer*/
                                OS_FREE_FUNC(payload_temp->ppayload);
                                OS_FREE_FUNC(payload_temp);
                                /*once abandon one package send nex package*/
                                payload_temp = send_package_list_header;
                                if (payload_temp != NULL) {
                                        Y_SPRINTF("[CLING PAY MAC]time out send list not empty");
                                        payload_temp->state = PACKAGE_WAITING_FOR_ACK;/*this must be set v=before being sended*/
                                        send_package_assemble(payload_temp, PACKAGE_DATA);
                                        OS_TIMER_START(send_demo_timer, 0);
                                }else{
                                        Y_SPRINTF("[CLING PAY MAC]time out send list  empty");
                                }

                        }
                        /*timeout process*/

                }
                else if (payload_temp->state == PACKAGE_SENDDED_SUCCESSFULLY) {
                        OS_TIMER_STOP(send_demo_timer, 0);
                        /*remove the packege sended successfully*/
                        send_package_list_header = send_package_list_header->next;
                        /*OS_FREE_FUNC payload buffer*/
                        OS_FREE_FUNC(payload_temp->ppayload);
                        OS_FREE_FUNC(payload_temp);
                        /*once recieved ack send nex package*/
                        payload_temp = send_package_list_header;
                        if (payload_temp != NULL) {
                                Y_SPRINTF("[CLING PAY MAC]send list not null");
                                payload_temp->state = PACKAGE_WAITING_FOR_ACK;/*this must be set v=before being sended*/
                                send_package_assemble(payload_temp, PACKAGE_DATA);
                                OS_TIMER_START(send_demo_timer, 0);
                        }else{
                                Y_SPRINTF("[CLING PAY MAC]send list empty");
                        }

                }
                /*arm the check routine period again*/
                //os_timer_arm(&(uart_send_timer), CLING_TX_UART_FIFO_POLL_PERIOAD, 0);
        }

        SUCESS:
        /*leave criticla section ,release semphore here*/
        UNLOCK_SEMAPHORE(send_fifo_semphore);
        return 0;
        FAILED:
        /*leave criticla section ,release semphore here*/
        UNLOCK_SEMAPHORE(send_fifo_semphore);
        return -1;
}

/******************************************************************************
 * FunctionName : macsend_ack_recieved()
 * Description  : internally used when a ack package is recieved
 * Parameters   : struct mac_layer_payload_rev *ppayload: mac layer layload pointer
 * Returns      : none
 *******************************************************************************/
__attribute__((unused)) static int macsend_ack_recieved()
{
        struct mac_layer_payload_send *payload_temp = send_package_list_header;
        /**/
        if (NULL == payload_temp) {
                return 0;

        }
        else {

                if (payload_temp->state == PACKAGE_WAITING_FOR_ACK) {
                        OS_TIMER_STOP(send_demo_timer, 0);
                        /*clent has revieced the package successfully */
                        payload_temp->state = PACKAGE_SENDDED_SUCCESSFULLY;
                        mac_sendlist_mantain_demon();
                }
        }

        return 0;

}

/******************************************************************************
 * FunctionName : macsend_ack_recieved()
 * Description  : internally used ,obbtain a data struct to the chain list
 * Parameters   : struct mac_layer_payload_rev *ppayload: mac layer layload pointer
 * Returns      : none
 *******************************************************************************/
static int send_package_assemble(struct mac_layer_payload_send *payload_temp,
        enum package_type type)
{

        unsigned short int i = 0;
        if (type == PACKAGE_ACK) {
                Y_SPRINTF("[cling pay mac]send ack");
                const uint8_t ack_buffer[] = { PACKAGE_HEADER_SYMBOL, PACKAGE_ACK, PACKAGE_END_SYMBOL };
                /*send ddta to client*/
                MAC_SEND_BUFFER(ack_buffer, sizeof(ack_buffer));
        }
        else if (type == PACKAGE_DATA || type == PACKAGE_CMD) {
                static char data_buffer[512 +12];
                unsigned short int position = 0;

                data_buffer[position] = PACKAGE_HEADER_SYMBOL;
                position++;
                data_buffer[position] = type;
                position++;
                /*process the data transferred by uart*/
                for (i = 0; i < (payload_temp->lenth); i++) {
                        /*Sspecail data*/
                        if ((payload_temp->ppayload[i]) == PACKAGE_HEADER_SYMBOL
                                || (payload_temp->ppayload[i]) == PACKAGE_END_SYMBOL
                                || (payload_temp->ppayload[i]) == PACKAGE_TRANSFER_SYMBOL) {

                                data_buffer[position] = PACKAGE_TRANSFER_SYMBOL;
                                position++;
                                data_buffer[position] = (payload_temp->ppayload[i]
                                        - PACKAGE_HEADER_SYMBOL + 1);
                                position++;
                        }
                        else {

                                data_buffer[position] = (payload_temp->ppayload[i]);
                                position++;
                        }

                }
                /*full fill end chractor*/
                data_buffer[position] = PACKAGE_END_SYMBOL;
                position++;
                for(int i = 0;i <position/20 +1 ; i++)
                {
                        MAC_SEND_BUFFER(data_buffer + i*20, 20);
                }
        }

        return 0;

}
/******************************************************************************
 * FunctionName : init_uart_implement
 * Description	: implement uart mac layer interface
 * Parameters	: none
 * Returns		: none
 *******************************************************************************/

int delete_init_uart_implement(CLASS(uart_implement) *arg)
{

        assert(NULL != arg);
        OS_FREE_FUNC(arg);
        return 0;
}

/******************************************************************************
 * FunctionName : init_uart_implement
 * Description	: implement uart mac layer interface
 * Parameters	: none
 * Returns		: none
 *******************************************************************************/

int init_uart_implement(CLASS(uart_implement) *arg)
{

        assert(NULL != arg);
        /*initiate cilng command receive object*/
        arg->init = init_uart_implement;
        arg->de_init = delete_init_uart_implement;
        arg->send_data = mac_send_payload;
        arg->get_data = NULL;
        //uart_init(BIT_RATE_115200, BIT_RATE_115200);
        return 0;

}

/******************************************************************************
 * FunctionName :register_serial_fd
 * Description  : regiter opened serial file description
 * Parameters   : struct mac_layer_payload_rev *ppayload: mac layer layload pointer
 * Returns      : none
 *******************************************************************************/
int register_serial_fd(int fd)
{
        reg_serial_fd = fd;
        return 0;
}
/******************************************************************************
 * FunctionName :register_write_function
 * Description  : regiter serial write callback
 * Parameters   : write: write call back function
 * Returns      : 0: sucess
 *				 -1: error
 *******************************************************************************/
int register_write_function(int (*write)(char*, unsigned int))
{

        mac_send_data = write;
        return 0;
}

