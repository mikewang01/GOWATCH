/********************************************************************************

 **** Copyright (C), 2015, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : wrist_detect.h
 * Author        : MikeWang
 * Date          : 2016-03-28
 * Description   : wrist_detect.c header file
 * Version       : 1.0
 * Function List :
 *
 * Record        :
 * 1.Date        : 2016-03-28
 *   Author      : MikeWang
 *   Modification: Created file

 *************************************************************************************************************/

#ifndef __FLASH_HAL_H__
#define __FLASH_HAL_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "stdint.h"
#include "oop_hal.h"
#include "string.h"
//#define __WIRST_DETECT_ENABLE__

/** An interface for the wrist_detect to take media loacated in host device under control
 *
 * @code
 * //Uses read the touch or swipe inforamtion through the i2c protocol and prococess touch interupt
 *
 * #include "wrist_detect.h"
 *
 * CLASS(wrist_detect) *media_ctl_obj = NULL;
 *
 * int main() {
 *                   media_ctl_obj = wrist_detect_get_instance;
 *     while(1) {
 *                              if(conditon){
 *                                  touch_onj->isr_process(touch_ist);
 *                              }
 *     }
 *
 * }
 * void ble_dispatch(ble_evt_t * p_ble_evt){
 *
 *              touch_obj->on_ble(touch_obj, p_ble_evt);
 *}
 * @endcode
 */

/*object prototype declaration*/
/*class defination header*/
DEF_CLASS(HalFlash)
        int (*write_enable)(CLASS(HalFlash) *arg);
        int (*write_disable)(CLASS(HalFlash) *arg);
        int (*get_status_register)(CLASS(HalFlash) *arg);
        size_t (*get_size)(CLASS(HalFlash) *arg);
        size_t (*get_erase_size)(CLASS(HalFlash) *arg);
        size_t (*get_page_size)(CLASS(HalFlash) *arg);
        int (*read_data)(CLASS(HalFlash) *arg, uint32_t addr, uint16_t len,
                uint8_t *data_buf);
        int (*page_program)(CLASS(HalFlash) *arg, uint32_t addr, uint16_t len,
                uint8_t *data_buf);
        int (*erase_one_block_4k)(CLASS(HalFlash) *arg, uint32_t addr);
        int (*erase_one_block_32k)(CLASS(HalFlash) *arg, uint32_t addr);
        int (*erase_one_block_64k)(CLASS(HalFlash) *arg, uint32_t addr);
        int (*erase_all)(CLASS(HalFlash) *arg);
END_DEF_CLASS(HalFlash)

CLASS(HalFlash)* HalFlash_get_instance(void);
/*================================CONFIGRATION===============================================*/

#define  CLING_HW_FLASH_INIT() do{\
        HalFlash_get_instance();\
}while(0)


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* __FLASH_HAL_H__ */
