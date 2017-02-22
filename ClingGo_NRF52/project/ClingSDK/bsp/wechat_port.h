///********************************************************************************
//
// **** Copyright (C), 2015, xx xx xx xx info&tech Co., Ltd.                ****
//
// ********************************************************************************
// * File Name     : wechat_port.h
// * Author        : MikeWang
// * Date          : 2015-11-30
// * Description   : wechat_port.c header file
// * Version       : 1.0
// * Function List :
// *
// * Record        :
// * 1.Date        : 2015-11-30
// *   Author      : MikeWang
// *   Modification: Created file
//
//*************************************************************************************************************/
//
//#ifndef __WECHAT_PORT_H__
//#define __WECHAT_PORT_H__
//
//
//#ifdef __cplusplus
//#if __cplusplus
//extern "C"{
//#endif
//#endif /* __cplusplus */
//
//#include "stdint.h"
//
///*================================CONFIGRATION===============================================*/
//
//#define WECHAT_SIMPLIFIED_PROTOCOL_SUPPORTED    1
//#define WECHAT_COMPELEX_PROTOCOL_SUPPORTED      0
//
//#if (WECHAT_SIMPLIFIED_PROTOCOL_SUPPORTED == 1 || WECHAT_COMPELEX_PROTOCOL_SUPPORTED == 1)
//	#define __WECHAT_SUPPORTED__
//#endif
//
//#define BLE_UUID_WECHAT_SERVICE							 			0xFEE7
//#define	COMPANY_IDENTIFIER												0x0056
//
//extern uint8_t wechat_delay(void);
//extern uint32_t wechat_get_calories(void);
//extern uint32_t wechat_get_distance(void);
//extern uint32_t wechat_get_step_count(void);
//extern uint32_t wechat_get_target_steps(void);
//extern uint16_t wechat_sports_register_obtain_system_data_call_back_callback_function(uint16_t (*p)(uint16_t));
//extern uint16_t wechat_sports_register_upload_stream_callback(uint16_t (*p)(uint16_t));
//extern uint16_t wechat_sports_update_calories(uint32_t calories);
//extern uint16_t wechat_sports_update_distance(uint32_t distance);
//extern uint16_t wechat_sports_update_steps_count(uint32_t steps_count);
//extern uint16_t wechat_sports_update_steps_count_distance_calories(uint32_t steps_count, uint32_t distance, uint32_t calories);
//extern uint16_t wechat_sports_update_target_steps_count(uint32_t steps_count);
//extern uint16_t  wechat_sports_main_process(void);
////extern uint16_t  wechat_ble_evt_dispatch(ble_evt_t * p_ble_evt);
//extern void get_mac_addr(uint8_t *p_mac_addr);
//extern uint16_t  wechat_init(void);
//#ifdef __cplusplus
//#if __cplusplus
//}
//#endif
//#endif /* __cplusplus */
//
//
//#endif /* __WECHAT_PORT_H__ */
