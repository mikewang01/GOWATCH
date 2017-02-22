/********************************************************************************

 **** Copyright (C), 2015, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : wrist_detect.h
 * Author        : MikeWang
 * Date          : 2016-03-28
 * Description   : _BATT_HAL_H_ header file
 * Version       : 1.0
 * Function List :
 *
 * Record        :
 * 1.Date        : 2016-03-28
 *   Author      : MikeWang
 *   Modification: Created file

 *************************************************************************************************************/

#ifndef _BATT_HAL_H_
#define _BATT_HAL_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "stdint.h"
#include "oop_hal.h"

/** An interface for the BattMgrHal to take media loacated in host device under control
 *
 * @code
 * //Uses read the batt voltage(mv) example
 *
 * #include "batt_hal.h"
 *
 * CLASS(BattMgrHal) *bat_mgr = NULL;
 *
 *  static void usb_charger_event_observer(usb_charger_event_t evt)
 * {
 *        Y_SPRINTF("[batt mgr] test event %d arrived", (uint8_t) evt);
 * }
 *
 *               Y_SPRINTF("[ble] batt get instance");
 *               batt_handle_t t = p->open(p);
 *              Y_SPRINTF("[ble] batt voltage = %d mv", p->read(p, t));
 *      int main() {
 *               CLING_BATT_MGR_DEV_HAL_INIT(usb_charger_event_observer);
 *               at_mgr = BattMgrHal_get_instance();
 *
 *     while(1) {
 *
 *                batt_handle_t hdl = p->open(at_mgr);
 *                Y_SPRINTF("[ble] batt voltage = %d mv", p->read(p, hdl));
 *                delay(xxx);
 *     }
 *
 * }
 *}
 * @endcode
 */


/*================================type def===============================================*/
typedef void* batt_handle_t;
typedef enum {
        USB_CHARGER_NO_EVENT = 0,
        USB_ATTACH_EVENT,
        USB_DETACH_EVENT,
        USB_START_ENUMMERATION_EVENT,
        USB_CHARGING_EVENT,
        USB_PRECHARGING_EVENT,
        USB_PRECHARGING_ABORTED_EVENT,
        USB_CHARGING_STOPPED_EVENT,
        USB_CHARGING_ABORTED_EVENT,
        USB_CHARGED_EVENT,
        USB_CHARGER_BATT_FULL_EVENT,
        USB_CHARGER_BAD_BATT_EVENT,
        USB_CHARGER_BATT_TMP_LOW_EVENT,
        USB_CHARGER_BATT_TMP_HIGH_EVENT

} usb_charger_event_t;

/*================================CONFIGRATION===============================================*/
/*================================CONFIGRATION===============================================*/
DEF_CLASS(BattMgrHal)
        batt_handle_t (*open)(CLASS(BattMgrHal) *arg);/*Read battery device to obtain batt handle*/
        uint16_t (*read)(CLASS(BattMgrHal) *arg, const batt_handle_t hdl);/*Read battery voltage in milli-volts*/
        uint16_t (*read_async)(CLASS(BattMgrHal) *arg, const batt_handle_t hdl,
                void (*cb)(int value));
        int (*reg_usb_charger_event_cb)(CLASS(BattMgrHal) *arg, void (*cb)(usb_charger_event_t evt));
        int (*close)(CLASS(BattMgrHal) *arg, const batt_handle_t hdl);
END_DEF_CLASS(BattMgrHal)

CLASS(BattMgrHal)* BattMgrHal_get_instance(void);

/*================================CONFIGRATION===============================================*/
/*WRAP IT TP A INITIZIZED ONE */
#define CLING_BATT_MGR_DEV_HAL_INIT(CALLBACK_FUNTION_P)  do{\
        CLASS(BattMgrHal)*p = BattMgrHal_get_instance();\
        void (*cb)(usb_charger_event_t evt) = CALLBACK_FUNTION_P;\
        if(cb != NULL){\
                p->reg_usb_charger_event_cb(p, cb);\
        }\
 }while(0)

 
#define SAADC_DEVICE(adc_channel_num, name, ad_pin, adc_resolution, over_sample)\
				extern const void *const name;
				
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* _BATT_HAL_H_ */
