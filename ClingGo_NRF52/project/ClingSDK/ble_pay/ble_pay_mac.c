
/******************************************************************************
 * Copyright 2013-2016 SangHai Hicling (Mike)
 *
 * FileName:  ble_debug_pro_cmd.c
 *
 * Description: ble debug tool cmd layer
 *
 * Modification history:
 *     2015/11/1, v1.0 create this file.
 *     2016/1/21, v1.1 add blocking coomunication support
*******************************************************************************/
#include "ble_pay_mac.h"
#include "ble_pay_cmd.h"
#include "ble_pay_app.h"
#include "ble_pay_service.h"
#include "stdio.h"
#include "assert.h"
#include <stdlib.h>
#include <string.h>
#include "main.h"

#define _BLE_PAY_MAC_LOG N_SPRINTF

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



#define  MAC_SEND_BUFFER(__X, __Y, __Z) do{\
		uint8_t *s = __X;\
		uint16_t total_length = __Y;\
		uint16_t length = __Y;\
		while(total_length > 0){\
			if(ble_connected == false){\
					break;\
			}\
			length = total_length;\
			length = length> 20?20:length;\
			__Z = ble_pay_notify_data_to_pc(&m_uico, m_uico.battery_level_handles[0], s, length);\
			total_length -= length;\
			s+=length;\
		}\
}while(0)

#define  ENTER_CRITICAL_SECTION() 
#define  LEAVE_CRITICAL_SECTION() 

/*THE UPPER LIMIT OF TIMES THE DATA BEEN SENDED*/

#define  RESEND_TIMES_UPPER_THREADHOLD  5
/*********************************************************************
* TYPEDEFS
*/
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
/*********************************************************************
* GLOBAL VARIABLES
*/
#define MESSAGE_MAX_CHAIN_LENTH 	8
#define CLING_TX_UART_FIFO_POLL_PERIOAD 100

/*********************************************************************
* static VARIABLES
*/

//static int reg_serial_fd = 0;
/*send data call back function*/
//static int  (*mac_send_data)(uint8_t*, size_t) = NULL;

static RcvMsgBuff serial_buff;
static ble_pay_t       m_uico = {0};                                     /**< Structure used to identify the heart rate service. */
static bool ble_connected = false;
/*recieve fifo related pointer*/
static enum package_state current_state = PACKAGE_NULL;
static struct mac_layer_payload_rev *rev_payload_list_header = NULL;
static struct mac_layer_payload_rev *rev_payload_list_tail = NULL;

static void (*cmd_recieved_callback)(char cmd) = NULL;
static void (*data_recieved_callback)(uint8_t*, int) = NULL;

static SEMAPHORE rev_fifo_semphore = UNLOCKED;


/**/
static struct mac_layer_payload_send *send_package_list_header = NULL;
//static struct mac_layer_payload_send *send_package_list_tail = NULL;
static SEMAPHORE send_fifo_semphore = UNLOCKED;

//static os_timer_t  uart_send_timer;
//static int macsend_ack_recieved(void);
static int send_package_assemble(struct mac_layer_payload_send *payload_temp, enum package_type type);
static int add_payload2revlist(unsigned char *pbuffer, int size);
void received_data_process(RcvMsgBuff *para);
static void ble_parameter_init(void);



int cling_pay_mac_init()
{
		ble_parameter_init();
		return 1;
}

/******************************************************************************
 * FunctionName : ble_pay_debug_evt_handler
 * Description  : uico layer message reciever and pricesser
 * Parameters   : ble_uico_t * p_bas uico ble object ble_uico_evt_t * specific event sended
 * Returns      : none
*******************************************************************************/
static void ble_pay_debug_evt_handler(ble_pay_t * p_bas, ble_pay_evt_t * p_evt)
{

    switch(p_evt->evt_type) {
            /**< Battery value notification enabled event. */

        case ble_pay_EVT_NOTIFICATION_ENABLED:
            ble_connected = true;
            break;
        case ble_pay_EVT_TEST_TOOL_DISCONNECTED:
        case ble_pay_EVT_NOTIFICATION_DISABLED:
            ble_connected = false;
 
            break;

        case ble_pay_EVT_BUFFER_SENDED_SUCESSFULLY:
           //D last_package_send_ok = true;
            break;

        default:
            // No implementation needed.
            break;
    }

}
/******************************************************************************
 * FunctionName : uico_touch_init
 * Description  : uico
 * Parameters   : void
 * Returns      : none
*******************************************************************************/
static void ble_parameter_init(void)
{
    uint32_t       err_code;
    ble_pay_init_t bas_init;


    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.write_perm);

    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);
    bas_init.evt_handler          = ble_pay_debug_evt_handler;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level[0]   = 99;

    err_code = ble_pay_init(&m_uico, &bas_init);
    APP_ERROR_CHECK(err_code);

}

/******************************************************************************
 * FunctionName : receive_one_char_callback
 * Description  : call back function when received one char
 * Parameters   : unsigned char rev_char
 * Returns      : none
*******************************************************************************/
int receive_one_char_callback(unsigned char rev_char)
{

	RcvMsgBuff *para = &serial_buff;
	//_BLE_PAY_MAC_LOG("\r\ndata=");
	
	//_BLE_PAY_MAC_LOG(":%02x",rev_char);
#if 0
    MAC_SEND_BUFFER(&rev_char, 1);
#endif
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

        } else if (rev_char == PACKAGE_END_SYMBOL) {
            /*change current state machine*/
            current_state = PACKAGE_END;

        } else if (rev_char == PACKAGE_TRANSFER_SYMBOL) { /*data need transferred*/
            /*change current state machine*/
            current_state = PACKAGE_TRANSFER;
        } else { /*normal data*/
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

        } else if (rev_char == PACKAGE_END_SYMBOL) {
            /*change current state machine*/
            current_state = PACKAGE_END;

        } else if (rev_char == PACKAGE_TRANSFER_SYMBOL) { /*data need transferred*/

            /*change current state machine abandon cuurent pakage*/
            current_state = PACKAGE_END;
        } else { /*normal data*/
            /*this means transferred data recieved*/
            if(rev_char == DATA_0X7D_TRANSFERRED || rev_char == DATA_0X7E_TRANSFERRED || rev_char == DATA_0X7F_TRANSFERRED) {
                para->BuffState = UNDER_WRITE;
                /*change current state machine*/
                current_state = PACKAGE_PAYLOAD;
                *(para->pWritePos) = 0x7d + rev_char -1;
                para->pWritePos++;

            } else {
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

        } else if (rev_char == PACKAGE_END_SYMBOL) {


            para-> BuffState = WRITE_OVER;
            received_data_process(para);
            /*change current state machine*/
            current_state = PACKAGE_END;
						return true;

        } else if (rev_char == PACKAGE_TRANSFER_SYMBOL) { /*data need transferred*/

            /*change current state machine abandon cuurent pakage*/
            current_state = PACKAGE_TRANSFER;
        } else { /*normal data*/
            /*this means transferred data recieved*/

            *(para->pWritePos) = rev_char;
            para->pWritePos++;

        }



    }
    break;

    default :
        break;
    }
#if 0
    MAC_SEND_BUFFER((u8*)&current_state, 1);
#endif
		return false;
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
 * FunctionName : set_recieved_data_call_back
 * Description  : call back function when received one char
 * Parameters   : unsigned char rev_char
 * Returns      : none
*******************************************************************************/
void set_recieved_data_call_back(void (*callback)(uint8_t*, int))
{
   data_recieved_callback = callback;
}
/******************************************************************************
 * FunctionName : receive_one_char_callback
 * Description  : call back function when received one char
 * Parameters   : unsigned char rev_char
 * Returns      : none
*******************************************************************************/
void received_data_process(RcvMsgBuff *para)
{

    /*this means this is a ack package*/
    if (para->pRcvMsgBuff[0] == PACKAGE_ACK) {
#ifdef BLE_DEBUG_UNBLOCK_MODE
        macsend_ack_recieved();
#elseif 
			
#endif
			_BLE_PAY_MAC_LOG("ack recieved\n");
    } else if (para->pRcvMsgBuff[0] == PACKAGE_DATA) {
			     add_payload2revlist(para->pRcvMsgBuff + 1, (para->pWritePos - para->pRcvMsgBuff - 1));
    }else if (para->pRcvMsgBuff[0] == PACKAGE_CMD){	
    	if (cmd_recieved_callback != NULL) {
				   cmd_recieved_callback(para->pRcvMsgBuff[1]);
		}
#ifdef BLE_DEBUG_UNBLOCK_MODE
			/*for the real time purpose, call back function is prefered*/
			/*after receieving ,ssend back ack package*/
      send_package_assemble(NULL, PACKAGE_ACK);
#else
			/*for blocked mode  do nothing when recived cmd mode*/
#endif

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

        _BLE_PAY_MAC_LOG("add_payload2revlist\n");
        /*if this lock is occupied by other process then return*/
        if (IS_SEMAPHORE_LOCKED(rev_fifo_semphore)) {
                goto FAILED;
        }
#if 0
        _BLE_PAY_MAC_LOG("data=");
        for(int i =0;i< size/16 + 1;i++) {
                for(int j = 0; j< 16; j++){
                        if(i*16 + j > size){
                                break;
                        }
                        _BLE_PAY_MAC_LOG("0x%02x ",pbuffer[i*16 + j]);
                }
                _BLE_PAY_MAC_LOG("i =  %d\r\n ", (i+1)*16);
        }
#endif

        /*enter critical section so that this is a atom acess*/
        LOCK_SEMAPHORE(rev_fifo_semphore);
//        cling_data_mac_pocess(pbuffer, size);
        if(data_recieved_callback != NULL){
                data_recieved_callback(pbuffer, size);
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

    } else {
        /*there is only one data struct exsited in fifo*/
        if (ptemp == rev_payload_list_tail) {
            *ppayload = ptemp;
            rev_payload_list_header = NULL;
            rev_payload_list_tail = NULL;
            goto SUCESS;
        }
        /*more than two data block existed, SO first step is to find the the last data block*/
        for (; ptemp->next != rev_payload_list_tail; ptemp = ptemp->next);
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
#ifdef BLE_DEBUG_UNBLOCK_MODE
#else
		struct mac_layer_payload_send payload_temp;
		payload_temp.ppayload = (uint8_t*)ppayload;
		payload_temp.state = PACKAGE_WAITING_FOR_SENDDING;
    payload_temp.lenth = lenth;
    payload_temp.next = NULL;
		//MAC_SEND_BUFFER((uint8_t*)ppayload, 19);
		return send_package_assemble(&payload_temp, PACKAGE_DATA);
#endif
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
		/*if the fifo ha already been empty,then it is not gonna rearm the timer again*/
        goto SUCESS;

    } else {
        /*current data need to be transmitted*/
        if (payload_temp->state == PACKAGE_WAITING_FOR_SENDDING) {

            send_package_assemble(payload_temp, PACKAGE_DATA);
            payload_temp->resend_times = 0;
            /*send data over here*/
            payload_temp->state = PACKAGE_WAITING_FOR_ACK;

        } else if (payload_temp->state == PACKAGE_WAITING_FOR_ACK) {

            if (payload_temp->resend_times < RESEND_TIMES_UPPER_THREADHOLD) {
                send_package_assemble(payload_temp, PACKAGE_DATA);
               // MAC_SEND_BUFFER(payload_temp->, position);
                payload_temp->resend_times ++;
            } else {

				_BLE_PAY_MAC_LOG("send time up\n");
                /*remove the packege sended successfully*/
                send_package_list_header = send_package_list_header->next;
                /*free payload buffer*/
                free(payload_temp->ppayload);
                free(payload_temp); 
				/*once abandon one package send nex package*/
				payload_temp = send_package_list_header;
				if (payload_temp != NULL) {
					send_package_assemble(payload_temp, PACKAGE_DATA);
				}

            }
            /*timeout process*/

        } else if (payload_temp->state == PACKAGE_SENDDED_SUCCESSFULLY) {
            /*remove the packege sended successfully*/
            send_package_list_header = send_package_list_header->next;
            /*free payload buffer*/
            free(payload_temp->ppayload);
            free(payload_temp);
			/*once recieved ack send nex package*/
			payload_temp = send_package_list_header;
			if (payload_temp != NULL) {
				send_package_assemble(payload_temp, PACKAGE_DATA);
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
#if 0
/******************************************************************************
 * FunctionName : macsend_ack_recieved()
 * Description  : internally used when a ack package is recieved
 * Parameters   : struct mac_layer_payload_rev *ppayload: mac layer layload pointer
 * Returns      : none
*******************************************************************************/
static int macsend_ack_recieved()
{
    struct mac_layer_payload_send *payload_temp = send_package_list_header;
    /**/
    if (NULL == payload_temp) {
        return 0;

    } else {

        if (payload_temp->state == PACKAGE_WAITING_FOR_ACK) {
            /*clent has revieced the package successfully */
            payload_temp->state = PACKAGE_SENDDED_SUCCESSFULLY;
						mac_sendlist_mantain_demon();
        }
    }


    return 0;


}
#endif
/******************************************************************************
 * FunctionName : macsend_ack_recieved()
 * Description  : internally used ,obbtain a data struct to the chain list
 * Parameters   : struct mac_layer_payload_rev *ppayload: mac layer layload pointer
 * Returns      : none
*******************************************************************************/

static int send_package_assemble(struct mac_layer_payload_send *payload_temp, enum package_type type)
{

    unsigned short int i = 0;
		int error_code = 0;
    if (type == PACKAGE_ACK) {
        uint8_t ack_buffer[] = {PACKAGE_HEADER_SYMBOL, PACKAGE_ACK, PACKAGE_END_SYMBOL};
        /*send ddta to client*/
		
         MAC_SEND_BUFFER(ack_buffer, sizeof(ack_buffer), error_code);
    } else if (type == PACKAGE_DATA || type == PACKAGE_CMD) {
        uint8_t data_buffer[256];
        unsigned short int position = 0;
        data_buffer[position] = PACKAGE_HEADER_SYMBOL;
        position++;
        data_buffer[position] = type;
        position ++;
#if 1
        /*process the data transferred by uart*/
        for(i = 0; i < (payload_temp->lenth); i++) {
            /*specail data*/
            if(payload_temp->ppayload[i] >= PACKAGE_HEADER_SYMBOL && payload_temp->ppayload[i]<=PACKAGE_TRANSFER_SYMBOL) {
                data_buffer[position] = PACKAGE_TRANSFER_SYMBOL;
                position++;
                data_buffer[position] = (payload_temp->ppayload[i] - PACKAGE_HEADER_SYMBOL + 1);
                position++;
            } else {
                data_buffer[position] = (payload_temp->ppayload[i]);
                position++;
            }

        }
        /*full fill end chractor*/
        data_buffer[position] = PACKAGE_END_SYMBOL;
        position++;
#endif
        MAC_SEND_BUFFER(data_buffer,position, error_code);
    }
    return error_code;


}
/******************************************************************************
 * FunctionName : init_uart_implement
 * Description	: implement uart mac layer interface
 * Parameters	: none
 * Returns		: none
*******************************************************************************/
#if 0
int delete_init_uart_implement(CLASS(uart_implement) *arg)
{

    assert(NULL != arg);
		free(arg);
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
    arg->init    = init_uart_implement;
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
#endif



