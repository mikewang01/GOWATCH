///********************************************************************************
//
// **** Copyright (C), 2015, xx xx xx xx info&tech Co., Ltd.                ****
//
// ********************************************************************************
// * File Name     : wechat_port.c
// * Author        : MikeWang
// * Date          : 2015-11-30
// * Description   : .C file function description
// * Version       : 1.0
// * Function List :
// *
// * Record        :
// * 1.Date        : 2015-11-30
// *   Author      : MikeWang
// *   Modification: Created file
//
//*************************************************************************************************************/
//#include "main.h"
//#include "wechat_port.h"
//#include "wechat_service_sports_simplified.h"
//#include "ble_wechat_util.h"
//#include "ble_wechat_service.h"
//#include "comsource.h"
///*****************************************************************************
// * Function      : get_step_count
// * Description   : get steps count port function
// * Input          : None
// * Output        : None
// * Return        : static
// * Others        :
// * Record
// * 1.Date        : 20151130
// *   Author      : MikeWang
// *   Modification: Created function
//
//*****************************************************************************/
//uint32_t wechat_get_step_count()
//{
//    uint32_t i = 0;
//    TRACKING_get_activity(0, TRACKING_STEPS, &i);
//    N_SPRINTF("[wechat port] get steps count = %d", i);
//    return i;
//}
///*****************************************************************************
// * Function      : wechat_get_distance
// * Description   : get diantance port function
// * Input          : None
// * Output        : None
// * Return        :
// * Others        :
// * Record
// * 1.Date        : 20151130
// *   Author      : MikeWang
// *   Modification: Created function
//
//*****************************************************************************/
//uint32_t wechat_get_distance()
//{
//    uint32_t i = 0;
//    TRACKING_get_activity(0, TRACKING_DISTANCE, &i);
//    return i;
//}
///*****************************************************************************
// * Function      : wechat_get_calories
// * Description   : get caloried fucntion
// * Input          : None
// * Output        : None
// * Return        :
// * Others        :
// * Record
// * 1.Date        : 20151130
// *   Author      : MikeWang
// *   Modification: Created function
//
//*****************************************************************************/
//uint32_t wechat_get_calories()
//{
//    uint32_t i = 0;
//    TRACKING_get_activity(0, TRACKING_CALORIES, &i);
//    return i;
//}
///*****************************************************************************
// * Function      : wechat_get_target_steps
// * Description   : get target steps function
// * Input          : None
// * Output        : None
// * Return        :
// * Others        :
// * Record
// * 1.Date        : 20151130
// *   Author      : MikeWang
// *   Modification: Created function
//
//*****************************************************************************/
//uint32_t wechat_get_target_steps()
//{
//    uint32_t i = 0;
//    return i;
//}
//
//
///*****************************************************************************
// * Function      : wechat_delay
// * Description   : delay function used to detect if delay condition has
//                   been satisfied or not
// * Input          : None
// * Output        : None
// * Return        :
// * Others        :
// * Record
// * 1.Date        : 20151130
// *   Author      : MikeWang
// *   Modification: Created function
//
//*****************************************************************************/
//#define WECHAT_SPORTS_DEMON_DELAY_PERIOD 5000 //MS
//uint8_t wechat_delay()
//{
//    static uint32_t last_record_time = 0;
//
//#if 1
//    if (CLK_get_system_time() > (last_record_time + WECHAT_SPORTS_DEMON_DELAY_PERIOD)) {
//        last_record_time = CLK_get_system_time();
//        return TRUE;
//    } else{
//        return FALSE;
//		}
//#else
//    return FALSE;
//#endif
//}
//
///*****************************************************************************
// * Function      : wechat_sports_update_steps_count
// * Description   : set wechat steps count for user interface
// * Input         : uint32_t steps_count
// * Output        : None
// * Return        :
// * Others        :
// * Record
// * 1.Date        : 20151130
// *   Author      : MikeWang
// *   Modification: Created function
//
//*****************************************************************************/
//uint16_t wechat_sports_update_steps_count(uint32_t steps_count)
//{
//    return FALSE;
//}
//
///*****************************************************************************
// * Function      : wechat_sports_update_distance
// * Description   : wechat set diantance parameter for uer interface
// * Input         : uint32_t distance
// * Output        : None
// * Return        :
// * Others        :
// * Record
// * 1.Date        : 20151130
// *   Author      : MikeWang
// *   Modification: Created function
//
//*****************************************************************************/
//uint16_t wechat_sports_update_distance(uint32_t distance)
//{
//    return FALSE;
//}
///*****************************************************************************
// * Function      : wechat_sports_update_calories
// * Description   : wechat interface used to update calories to cell
// * Input         : uint32_t calories
// * Output        : None
// * Return        :
// * Others        :
// * Record
// * 1.Date        : 20151130
// *   Author      : MikeWang
// *   Modification: Created function
//
//*****************************************************************************/
//uint16_t wechat_sports_update_calories(uint32_t calories)
//{
//    return FALSE;
//}
///*****************************************************************************
// * Function      : wechat_sports_update_steps_count_distance_calories
// * Description   : user interface used to update steps and diantance and
//                   lolaried
// * Input         : uint32_t steps_count
//                uint32_t distance
//                uint32_t calories
// * Output        : None
// * Return        :
// * Others        :
// * Record
// * 1.Date        : 20151130
// *   Author      : MikeWang
// *   Modification: Created function
//
//*****************************************************************************/
//uint16_t wechat_sports_update_steps_count_distance_calories(uint32_t steps_count, uint32_t distance, uint32_t calories)
//{
//    return FALSE;
//}
//
///*****************************************************************************
// * Function      : wechat_sports_update_target_steps_count
// * Description   : user api used to update target steps g
// * Input         : uint32_t steps_count
// * Output        : None
// * Return        :
// * Others        :
// * Record
// * 1.Date        : 20151130
// *   Author      : MikeWang
// *   Modification: Created function
//
//*****************************************************************************/
//uint16_t wechat_sports_update_target_steps_count(uint32_t steps_count)
//{
//    return FALSE;
//}
//
///*********************************************************************
// * @fn      register_app_layer_callback
// *
// * @brief   regiter app layer uploading data callback function
// *
// * @param   task_id,message_type.
// *
//* @return  TRUE sucessfully FALSE:FAILED
// */
//static uint16_t (*app_layer_upload_stream_call_back)(uint16_t order) = NULL;
//uint16_t wechat_sports_register_upload_stream_callback(uint16_t (*p)(uint16_t))
//{
//    if(p == NULL) {
//        app_layer_upload_stream_call_back = p;
//    }
//    return TRUE;
//}
//
///*********************************************************************
// * @fn      register_app_layer_callback
// *
// * @brief   regiter app layer uploading data callback function
// *
// * @param   task_id,message_type.
// *
//* @return  TRUE sucessfully FALSE:FAILED
// */
//static uint16_t (*app_layer_obtain_system_data_call_back)(uint16_t order) = NULL;
//uint16_t wechat_sports_register_obtain_system_data_call_back_callback_function(uint16_t (*p)(uint16_t))
//{
//    if(p == NULL) {
//        app_layer_obtain_system_data_call_back = p;
//    }
//    return TRUE;
//}
//
///******************************************************************************
// * FunctionName : get_mac_addr
// * Description  : get mac adress of ble and arrage it as big endian mode
// * Parameters   : void
// * Returns      : none
//*******************************************************************************/
//void get_mac_addr(uint8_t *p_mac_addr)
//{
//    uint32_t error_code;
//    if(p_mac_addr == NULL) {
//        return;
//    }
//    ble_gap_addr_t mac_addr_tmp;
//    error_code = sd_ble_gap_address_get(&mac_addr_tmp);
//    APP_ERROR_CHECK(error_code);
//#ifdef CATCH_LOG
//    N_SPRINTF("\r\n error:%d", error_code);
//    N_SPRINTF("\r\n get mac addr");
//#endif
//    uint8_t *d = mac_addr_tmp.addr;
//    for ( uint8_t i = 6; i > 0;) {
//        i--;
//        p_mac_addr[5 - i] = d[i];
//#ifdef CATCH_LOG
//        N_SPRINTF ( ":%02x", d[i]);
//#endif
//    }
//#ifdef CATCH_LOG
//    putchar ( '\n' );
//#endif
//}
//
///*********************************************************************
// * @fn      register_app_layer_callback
// *
// * @brief   regiter app layer uploading data callback function
// *
// * @param   task_id,message_type.
// *
//* @return  TRUE sucessfully FALSE:FAILED
// */
//extern data_handler *m_mpbledemo2_handler;
//extern ble_wechat_t m_ble_wechat;
//uint16_t  wechat_sports_main_process(void)
//{
//
//#if (WECHAT_SIMPLIFIED_PROTOCOL_SUPPORTED == 1)
//    wechat_sports_update_deamon();
//#endif
//#if (WECHAT_COMPELEX_PROTOCOL_SUPPORTED == 1)
//    m_mpbledemo2_handler->m_data_main_process_func(&m_ble_wechat);
//#endif
//    return TRUE;
//}
//
///*********************************************************************
// * @fn      wechat_ble_evt_dispatch
// *
// * @brief   wechat ble event distributor
// *
// * @param   ble_evt_t * p_ble_evt£º event pointer
// *
//* @return  TRUE sucessfully FALSE:FAILED
// */
//uint16_t  wechat_ble_evt_dispatch(ble_evt_t * p_ble_evt)
//{
//    wechat_ble_evt_dispatch_local(p_ble_evt);
//    return TRUE;
//}
//
///*********************************************************************
// * @fn      wechat_init
// *
// * @brief   wechat service and process initiated
// *
// * @param   task_id,message_type.
// *
//* @return  TRUE sucessfully FALSE:FAILED
// */
//uint16_t  wechat_init(void)
//{
//    wehchat_resource_init();
//		app_layer_obtain_system_data_call_back = app_layer_obtain_system_data_call_back;
//		app_layer_upload_stream_call_back = app_layer_upload_stream_call_back;
//    return TRUE;
//}
//
//
