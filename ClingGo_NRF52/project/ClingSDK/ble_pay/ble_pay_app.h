#ifndef __UART_PROTOCAL_APP_H__
#define __UART_PROTOCAL_APP_H__

#include "stdint.h"
#include "oop_hal.h"
#include "stdio.h"
#define __CLING_PAY_SUPPORTED__   1
struct ble_pay_adapter_call_backs{
		int (*init)(void);
		int (*core_fsm)(void *p_dr);
		int (*rx_cb)(void *p_dr, uint16_t length);	
};

/*object prototype declaration*/
DEF_CLASS(cling_pay_app)
	int (*init)    					(CLASS(cling_pay_app) *arg); 			/*initiate uart object*/
	int (*de_init) 					(CLASS(cling_pay_app) *arg);				/*delete uart object*/
	CLASS(cling_pay_app)  *(*get_instance)();/*make sure this object get created only once*/
	int (*send_event_from_app) 		    (CLASS(cling_pay_app) *arg, void *event, uint16_t size);/*data send   function*/
	int (*send_event_from_1280) 		  (CLASS(cling_pay_app) *arg, void *event, uint16_t size);/*data send   function*/
	int (*reg_iso7816_tx)			(CLASS(cling_pay_app) *arg, int (*cb)(uint8_t *p, size_t lenth));
	int (*reg_ble_tx)			(CLASS(cling_pay_app) *arg, int (*cb)(uint8_t *p, size_t lenth));
	int (*core_process) 		  (CLASS(cling_pay_app) *arg, void*data);/*data send  function*/
  int (*reg_adaptor)(CLASS(cling_pay_app) *arg, struct ble_pay_adapter_call_backs  *p);
	void *user_data;/*point to user private data*/
END_DEF_CLASS(cling_pay_app)

CLASS(cling_pay_app)* cling_pay_app_get_instance(void);
#endif

