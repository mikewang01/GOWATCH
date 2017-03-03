/********************************************************************************

 **** Copyright (C), 2015, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : aes_hal.h
 * Author        : MikeWang
 * Date          : 2016-03-28
 * Description   : aes_hal.c header file
 * Version       : 1.0
 * Function List :
 *
 * Record        :
 * 1.Date        : 2016-03-28
 *   Author      : MikeWang
 *   Modification: Created file

 *************************************************************************************************************/

#ifndef _BLE_GAP_MGR_H_
#define _BLE_GAP_MGR_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "stdint.h"
#include "oop_hal.h"
#include "ble_advdata.h"
#include "osal.h"
//#include "ble_gap.h"
//#include "ble_att.h"


/**@brief Adv data structor. */
struct adv_struct {
        struct {
                uint8_t lenth;
                uint8_t adv_type;
        } adv_header;
        struct {
                uint8_t lenth;
                uint8_t *data;
        } body;
};



/*class defination header*/
DEF_CLASS(GapMgr)
        int (*register_task)(OS_TASK p_task);
				int (*get_mac_addr)(CLASS(GapMgr) *arg, uint8_t *mac_addr, size_t *size);
				int (*get_model_id)(CLASS(GapMgr) *arg, uint8_t *model_id, size_t *size);
        int (*get_dev_name)(CLASS(GapMgr) *arg, uint8_t *dev_name);
        int (*init_adv_data)(CLASS(GapMgr) *arg);
        int (*get_dev_id_str)(CLASS(GapMgr) *arg, uint8_t *dev_id, size_t *size);
        int (*switch_2_fast_conn_mode)(CLASS(GapMgr) *arg);
        int (*switch_2_slow_conn_mode)(CLASS(GapMgr) *arg);
        int (*get_current_conn_para)(CLASS(GapMgr) *arg);
        int (*disconnect)(CLASS(GapMgr) *arg);
        int (*start_adv)(CLASS(GapMgr) *arg);
        int (*stop_adv)(CLASS(GapMgr) *arg);
        int (*event_process)(CLASS(GapMgr) *arg, void *hdr);
        int (*adv_freq_swicth_process)(CLASS(GapMgr) *arg);
        int (*reboot_on_disconnect)(CLASS(GapMgr) *arg); /* Reboot on disconnect */
END_DEF_CLASS(GapMgr)

CLASS(GapMgr)* GapMgr_get_instance(void);
/*================================CONFIGRATION===============================================*/
#define CLING_HW_GAP_MGR_INIT() do{\
        GapMgr_get_instance()\
}while(0)

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* _BLE_GAP_MGR_H_ */
