/******************************************************************************
 * Copyright 2013-2016 SangHai Hicling (Mike)
 *
 * FileName: cling debug app layer
 *
 * Description: entry file of user application
 *
 * Modification history:
 *     2016/1/21, v1.0 create this file.
*******************************************************************************/
#include "ble_pay_mac.h"
#include "ble_pay_cmd.h"
#include "ble_pay_app.h"
#include "stdio.h"
#include "assert.h"
#include "ble_pay_service.h"
#include "main.h"
#include "FM151.h"
#include "sys_power_mgr.h"
/*********************************************************************
* MACROS
*/
#define T base

/*********************************************************************
* TYPEDEFS
*/
enum {
    EVENT_NULL = 0,
    EVENT_BLE,
    EVENT_1280
};



struct T {
    uint16_t type;/*inficate cuurent packafe type*/
    uint16_t size;					/*indicate paload lenth*/
};

struct debug_pkg {
    struct T hdr;
    uint8_t  p[128];
};

struct pay_event {
    uint16_t type;
    void *p;
    uint16_t size;
};


#define MAX_REGISTERED_LIMIT  2
static struct ble_pay_adapter_call_backs  *reg_adptors[MAX_REGISTERED_LIMIT];
/*********************************************************************
* GLOBAL VARIABLES
*/

/*********************************************************************
* static VARIABLES
*/
static struct pay_event local_evt;
//static ble_pay_t       m_uico = {0};                                     /**< Structure used to identify the heart rate service. */
//static bool ble_connected = false;
//static bool last_package_send_ok = true;
//static app_timer_id_t uico_timer_debug_id;

static CLASS(cling_pay_app) *p_instance =  (CLASS(cling_pay_app)*)NULL;
//static int send_acc(uint16_t x_g, uint16_t y_g, uint16_t z_g);
static int (*p_send_7816)(uint8_t *p, size_t lenth) = NULL;
static int (*p_send_ble)(uint8_t *p, size_t lenth) = NULL;
//static void ble_parameter_init(void);
//static int log_debug(CLASS(cling_pay_app) *arg, char* p);
//static int shell_printf(CLASS(cling_pay_app) *arg, char* p);
static int  init_cling_pay_app(CLASS(cling_pay_app) *arg);
static int reg_7816_tx(CLASS(cling_pay_app) *arg, int (*cb)(uint8_t *p, size_t lenth));
static int reg_ble_tx(CLASS(cling_pay_app) *arg, int (*cb)(uint8_t *p, size_t lenth));
static int core_process(CLASS(cling_pay_app) *arg, void *pdr);
static int payment_process(void *data_ptr);
static int reg_adaptor(CLASS(cling_pay_app) *arg, struct ble_pay_adapter_call_backs  *p);
static int send_event_from_app(CLASS(cling_pay_app) *arg, void* event, uint16_t length);
/*********************************************************************
* EXTERNAL VARIABLES
*/

extern int ble_pay_debug_write_interface(uint8_t *p, size_t lenth);
extern void uico_touch_debug_init(void);
static int send_event_from_1280(CLASS(cling_pay_app) *arg, void* event, uint16_t length);
/*********************************************************************
* FUNCTIONS
*/


/******************************************************************************
 * FunctionName : init_cling_pay_app
 * Description	: uart cling message receive object init
 * Parameters	: none
 * Returns		: none
*******************************************************************************/
static int  init_cling_pay_app(CLASS(cling_pay_app) *arg)
{

    N_SPRINTF("[pro_app]: test");
    memset(reg_adptors, 0, sizeof(reg_adptors));
    memset(&local_evt, 0, sizeof(local_evt));
    //uico_touch_debug_init();
    //ble_parameter_init();
    arg->send_event_from_app = send_event_from_app;
		arg->send_event_from_1280 = send_event_from_1280;
    arg->reg_iso7816_tx = reg_7816_tx;
    arg->core_process = core_process;
    arg->reg_ble_tx = reg_ble_tx;
    arg->reg_adaptor  = reg_adaptor;
    //arg->debug_log = log_debug;
    //arg->shell_printf = shell_printf;
    arg->user_data = (void*)&local_evt;

    clingpay_cmd_init();
    ISO7816_Init();
    return 0;
}


/******************************************************************************
 * FunctionName : CLASS(cling_pay_app)* get_instance
 * Description	: get object instance pointer
 * Parameters	: none
 * Returns		: object pointer
*******************************************************************************/
CLASS(cling_pay_app)* cling_pay_app_get_instance()
{

    static CLASS(cling_pay_app) p;
    if(p_instance == NULL) {
        p_instance = &p;
        init_cling_pay_app(p_instance);
    }
    return p_instance;
}
/******************************************************************************
 * FunctionName : CLASS(cling_pay_app)* get_instance
 * Description	: get object instance pointer
 * Parameters	: p: buffer pointer lenth:buffer lenth
 * Returns		: 0 sucess -1:failed
*******************************************************************************/
static int send_event_from_app(CLASS(cling_pay_app) *arg, void* event, uint16_t length)
{
    if(event == NULL || arg== NULL) {
        return -1;
    }
    struct pay_event *p = (struct pay_event*)(arg->user_data);
    p->type = EVENT_BLE;
    p->p = event;
    p->size = length;
    return 0;
}
/******************************************************************************
 * FunctionName : CLASS(cling_pay_app)* get_instance
 * Description	: get object instance pointer
 * Parameters	: p: buffer pointer lenth:buffer lenth
 * Returns		: 0 sucess -1:failed
*******************************************************************************/
static int send_event_from_1280(CLASS(cling_pay_app) *arg, void* event, uint16_t length)
{
    if(event == NULL || arg== NULL) {
        return -1;
    }
    struct pay_event *p = (struct pay_event*)(arg->user_data);
    p->type = EVENT_1280;
    p->p = event;
    p->size = length;
    return 0;
} 
/******************************************************************************
 * FunctionName : CLASS(cling_pay_app)* get_instance
 * Description	: get object instance pointer
 * Parameters	: p: buffer pointer lenth:buffer lenth
 * Returns		: 0 sucess -1:failed
*******************************************************************************/
static int reg_adaptor(CLASS(cling_pay_app) *arg, struct ble_pay_adapter_call_backs  *p)
{
    if(arg == NULL || p== NULL) {
        return -1;
    }
    int i=0;
    for(i=0; i < sizeof(reg_adptors)/sizeof(reg_adptors[0]); i++) {
        if(reg_adptors[i] == NULL) {
            break;
        }
    }
    if(i < sizeof(reg_adptors)/sizeof(reg_adptors[0])) {
        reg_adptors[i] = p;
    } else {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * FunctionName : CLASS(cling_pay_app)* get_instance
 * Description	: get object instance pointer
 * Parameters	: p: buffer pointer lenth:buffer lenth
 * Returns		: 0 sucess -1:failed
*******************************************************************************/
static int reg_7816_tx(CLASS(cling_pay_app) *arg, int (*cb)(uint8_t *p, size_t lenth))
{
    if(arg == NULL || cb== NULL) {
        return -1;
    }
    p_send_7816 = cb;
    return 0;
}
/******************************************************************************
 * FunctionName : CLASS(cling_pay_app)* get_instance
 * Description	: get object instance pointer
 * Parameters	: p: buffer pointer lenth:buffer lenth
 * Returns		: 0 sucess -1:failed
*******************************************************************************/
static int reg_ble_tx(CLASS(cling_pay_app) *arg, int (*cb)(uint8_t *p, size_t lenth))
{
    if(arg == NULL || cb== NULL) {
        return -1;
    }
    p_send_ble = cb;
    return 0;
}

/******************************************************************************
 * FunctionName : CLASS(cling_pay_app)* get_instance
 * Description	: core_process
 * Parameters	: p: buffer pointer lenth:buffer lenth
 * Returns		: 0 sucess -1:failed
*******************************************************************************/
static int core_process(CLASS(cling_pay_app) *arg, void *pdr)
{
    if(arg == NULL) {
        return -1;
    }
    for(int i = 0; i< sizeof(reg_adptors)/sizeof(reg_adptors[0]); i++) {
        if(reg_adptors[i] == NULL) {
            break;
        } else {
						payment_process(arg->user_data);
            while(reg_adptors[i]->core_fsm(NULL) == true);
						payment_process(arg->user_data);
            /*used to implement state machine for dialog communicaiton*/
            //struct pay_event *p = (struct pay_event*)(arg->user_data);
            
        }
    }


    return 0;
}
/*used to implement state machine for dialog communicaiton*/
static int payment_process(void *data_ptr)
{
			//	int main_1280_test(void);
		//	main_1280_test();
		static struct pay_rev_package pkg_cmd;
    /*no message need to be processed so return*/
    if(data_ptr == NULL) {
        return 0;
    }
		/*==========================1280 to app process====================================================*/
    struct pay_event *p = (struct pay_event *)data_ptr;
		if(p->type == EVENT_1280){
			 if(p_send_ble != NULL){
					/*if user is not use the buffer stack provided, transferation is needed here*/
					if(p->p != pkg_cmd.data.body){
						memcpy(pkg_cmd.data.body, p->p, p->size);
					}
					 pkg_cmd.header.cmd_type = (BTC_IO_OK);
				   pkg_cmd.header.data_lenth = p->size;
					 p_send_ble((uint8_t*)&pkg_cmd, pkg_cmd.header.data_lenth);
			 }
			 memset(p, 0, sizeof(struct pay_event));
			 return 1;
		}
		
		/*==========================app to 1280 process=======================================================*/
    struct pay_rev_package *p_pkg = (struct pay_rev_package*)(p->p);

    if(p_pkg == NULL) {
        return 0;
    }
    
    if(sizeof(struct pay_rev_package) == p->size) {
        N_SPRINTF("[ble pay]:BING GO WE GET THE MESSAGE");
    }
		memset(p, 0, sizeof(struct pay_event));
		memcpy(&pkg_cmd, p_pkg, sizeof(struct pay_rev_package));
    if (pkg_cmd.header.cmd_type == BTC_APDU) {
        uint16_t len = pkg_cmd.header.data_lenth;
        //ISO7816_command_process(ISO7816_OP_SEND_APDU, pkg_cmd.data.body, &(len));
			 // pkg_cmd.header.data_lenth = len;
				N_SPRINTF("BTC_APDU");			
				if(p_send_7816 != NULL){
						p_send_7816(pkg_cmd.data.body, (len));
				}

    } else if (pkg_cmd.header.cmd_type == BTC_DISCONNECT) {
				//ISO7816_command_process(ISO7816_OP_DEACTIVATE, NULL, 0);
        // Clingpay_Deactivate_ISO7816();
				deactive_fm151();
        N_SPRINTF("BTC_DEACTIVE");
			  uint8_t a[] = {0x90, 0x00};
			 if(p_send_ble != NULL){
				 pkg_cmd.header.data_lenth = sizeof(a);
					/*if user is not use the buffer stack provided, transferation is needed here*/
				 memcpy(pkg_cmd.data.body, a, pkg_cmd.header.data_lenth);
				 pkg_cmd.header.cmd_type = (BTC_IO_OK);
				 p_send_ble((uint8_t*)&pkg_cmd, pkg_cmd.header.data_lenth);
				}
    } else if (pkg_cmd.header.cmd_type == BTC_CONNECT) {
//			 uint16_t len = pkg_cmd.header.data_lenth;
			  if(NRF_SUCCESS == active_fm151()){
					 uint8_t a[] = {0x90, 0x00};
					 if(p_send_ble != NULL){
						 pkg_cmd.header.data_lenth = sizeof(a);
							/*if user is not use the buffer stack provided, transferation is needed here*/
						 memcpy(pkg_cmd.data.body, a, pkg_cmd.header.data_lenth);
						 pkg_cmd.header.cmd_type = (BTC_IO_OK);
						 p_send_ble((uint8_t*)&pkg_cmd, pkg_cmd.header.data_lenth);
						}
				}
        N_SPRINTF("BTC_ACTIVE");
    } else if (pkg_cmd.header.cmd_type == BTC_ATR) {
        uint16_t len = pkg_cmd.header.data_lenth;
        //Clingpay_RETRIEVE_ATR(pkg_cmd.data.body, &(len));
        pkg_cmd.header.data_lenth = len;
        N_SPRINTF("BTC_ATR");
    } else if(pkg_cmd.header.cmd_type == BTC_BIND) {
        N_SPRINTF("BTC_BIND");
#if 0
        Y_SPRINTF("[phone id]:");
        for(int i = 0; i< pkg_cmd.header.data_lenth; i++) {
            printf("0x%02x ", pkg_cmd.data.body[i]);
        }
        Y_SPRINTF("\r\n");
#endif
        //  CLASS(GapMgr)*gap_mgr = GapMgr_get_instance();
        size_t dev_id_length;
        // gap_mgr->get_dev_id_str(gap_mgr, pkg_cmd.data.body , &dev_id_length);
        pkg_cmd.header.data_lenth =  dev_id_length;
        N_SPRINTF("BTC_DEV_ID = %s", pkg_cmd.data.body);
        /*this is not neccesary any how  ,this is put here for the reponse from z32 adn then  dialog is
         *able to generate one feedback to app*/
        // Clingpay_Activate_ISO7816();
    } else if(pkg_cmd.header.cmd_type == BTC_QR_CODE) {
        N_SPRINTF("BTC_QR_CODE REVED");
//                  printf("BTC_QR_CODE: ");
//                  for(int i = 0; i < pkg_cmd.header.data_lenth; i++){
//                          printf("0x%02x ", pkg_cmd.data.body[i]);
//                  }
//                  printf("\r\n");
        //CLASS(GapMgr)*gap_mgr = GapMgr_get_instance();
        size_t dev_id_length;
        // gap_mgr->get_dev_id_str(gap_mgr, pkg_cmd.data.body , &dev_id_length);
        pkg_cmd.header.data_lenth =  dev_id_length;

    } else if(pkg_cmd.header.cmd_type == BTC_QR_IMAGE) {
        N_SPRINTF("BTC_QR_IMAGE RECIEVED");
//                  printf("BTC_QR_IMAGE: ");
//                  for(int i = 0; i < pkg_cmd.header.data_lenth; i++){
//                          printf("0x%02x ", pkg_cmd.data.body[i]);
//                  }
//                  printf("\r\n");
        // CLASS(GapMgr)*gap_mgr = GapMgr_get_instance();
        size_t dev_id_length;
        // gap_mgr->get_dev_id_str(gap_mgr, pkg_cmd.data.body , &dev_id_length);
        pkg_cmd.header.data_lenth =  dev_id_length;
    }
    return 0;
}


