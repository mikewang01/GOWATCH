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

#ifndef __EXTI_HAL_H__
#define __EXTI_HAL_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "stdint.h"
#include "oop_hal.h"
#include "pinmap.h"
//#define __WIRST_DETECT_ENABLE__

/** An interface for the wrist_detect to take media loacated in host device under control
 *
 * @code
 * //Uses init external interrupt example
 *
 * #include "exti_hal.h"
 *
 * CLASS(HalExti) *exti_p = NULL;
 *
 *    void exti_callback(PinName pin)
        {
        //ptinf is not allowed here for exti stack is limited which gonna cause a halt
        //this is a isr callback  so if you want to use a notify, use OS_TASK_NOTIFY_FROM_ISR instead
        }

 * int main() {
 *                           CLING_HW_EXTI_INIT(exti_callback);
                             CLASS(HalExti) *exti_p = HalExti_get_instance();
                             exti_p->enable_all(exti_p);
 *     while(1) {
 *
 *     }
 *
 * }
 *}
 * @endcode
 */

/*object prototype declaration*/
/*class defination header*/
DEF_CLASS(HalExti)
        int (*enable_all)(CLASS(HalExti) *arg);
        int (*disable_all)(CLASS(HalExti) *arg);
        int (*isr_reg)(CLASS(HalExti) *arg, void (*cb)(PinName));
END_DEF_CLASS(HalExti)

CLASS(HalExti)* HalExti_get_instance(void);
/*================================CONFIGRATION===============================================*/
#define CLING_HW_EXTI_INIT(_CB_) do{\
        CLASS(HalExti) *p = HalExti_get_instance();\
        p->isr_reg(p, _CB_);\
}while(0)

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* __EXTI_HAL_H__ */
