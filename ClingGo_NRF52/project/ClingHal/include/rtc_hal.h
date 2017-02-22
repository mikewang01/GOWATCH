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

#ifndef __RTC_HAL_H__
#define __RTC_HAL_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "stdint.h"
#include "oop_hal.h"
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
 *}
 * @endcode
 */

/*object prototype declaration*/
/*class defination header*/
DEF_CLASS(HalRtc)
        int (*stop)(CLASS(HalRtc) *arg);
        int (*start)(CLASS(HalRtc) *arg);
        int (*isr_reg)(CLASS(HalRtc) *arg, void (*isr)(void));
END_DEF_CLASS(HalRtc)

CLASS(HalRtc)* HalRtc_get_instance(void);
/*================================CONFIGRATION===============================================*/
#define CLING_HW_RTC_INIT() do{\
        HalRtc_get_instance()\
}while(0)

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* __RTC_HAL_H__ */
