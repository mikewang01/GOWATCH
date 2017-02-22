/********************************************************************************

 **** Copyright (C), 2015, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : wrist_detect.h
 * Author        : MikeWang
 * Date          : 2016-03-28
 * Description   : __BLE_HAL_H__ header file
 * Version       : 1.0
 * Function List :
 *
 * Record        :
 * 1.Date        : 2016-03-28
 *   Author      : MikeWang
 *   Modification: Created file

 *************************************************************************************************************/

#ifndef __BLE_HAL_H__
#define __BLE_HAL_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "stdint.h"
#include "oop_hal.h"
#include "osal.h"
/*================================type def===============================================*/
typedef struct {
        OS_TASK ble_handle;
        OS_TASK cling_core_fsm_handle;
        OS_TASK input_handle;
        OS_TASK output_handle;
}task_handle_sets_t;

typedef  task_handle_sets_t TASK_HANDLE_SETS ;
/*================================CONFIGRATION===============================================*/

extern int btle_fsm_init(void);
extern int btle_fsm_process(void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* __BLE_HAL_H__ */
