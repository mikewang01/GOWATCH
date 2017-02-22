#include <battmgr_hal.h>
#include <btle_api.h>
/******************************************************************************
 #include <spidev_hal.h>
 #include <spidev_hal.h>
 * Copyright 2013-2016 hicling Systems (MikeWang)
 *
 * FileName: BattMgrHal.c
 *
 * Description: abstract layer between cling sdk and freertos.
 *
 * Modification history:
 *     2016/7/22, v1.0 create this file mike.
 *******************************************************************************/
#include "oop_hal.h"
//#include "ad_flash.h"
#include "rtc_hal.h"
#include "osal.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "pin_names.h"
//#include "platform_devices.h"
//#include "ad_battery.h"
#include "battmgr_hal.h"
#include "nrf_drv_saadc.h"
#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * MACROS
 */
#undef 	SAADC_DEVICE
#define SAADC_DEVICE(adc_channel_num, name, ad_pin, adc_resolution, over_sample) \
        const adc_device_config_t dev_##name = { \
								.bus_id = 0,\
								.pin_num = ad_pin,\
								.channel_num = adc_channel_num,\
								.hw_init.resolution         = (nrf_saadc_resolution_t)adc_resolution,\
								.hw_init.oversample         = (nrf_saadc_oversample_t)over_sample,\
								.hw_init.interrupt_priority = SAADC_CONFIG_IRQ_PRIORITY,\
								.hw_init.low_power_mode     = SAADC_CONFIG_LP_MODE,\
								.channel_cfg = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(ad_pin),\
        }; \
				const void *const name = &dev_##name;
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
/*********************************************************************
 * TYPEDEFS
 */
#define HW_TWI_ID int
typedef struct adc_device_config_t {
        HW_TWI_ID bus_id;               /**< SPI id as needed by hw_spi_... functions */
				uint8_t pin_num;
				uint8_t channel_num;
        nrf_drv_saadc_config_t hw_init;
	      nrf_saadc_channel_config_t   channel_cfg;
} adc_device_config_t;

struct adc_resouce{
		CLASS(BattMgrHal) *p_instance;
		OS_MUTEX  res_lock;
		OS_EVENT  bus_busy;
		uint16_t  ref_count;
		OS_TASK 	owner;
		adc_device_config_t *cur_dev_cfg;/*use to record current device config*/
};
#include "flatform_dev.h"
/*********************************************************************
 * LOCAL VARIABLES
 */
static CLASS(BattMgrHal) *p_instance = NULL;
static void (*user_layer_callback)(int value) = NULL;
static void (*usb_charger_mgr_event_cb_p)(usb_charger_event_t evt) = NULL;
static usb_charger_event_t latest_event_record = USB_CHARGER_NO_EVENT;
/*********************************************************************
 * LOCAL DECLARATION
 */
static batt_handle_t open(CLASS(BattMgrHal) *arg);
static uint16_t read(CLASS(BattMgrHal) *arg, const batt_handle_t hdl);
static void batt_async_read_callback(void *user_data, int value);
static uint16_t read_async(CLASS(BattMgrHal) *arg, const batt_handle_t hdl, void (*cb)(int value));
static int close(CLASS(BattMgrHal) *arg, const batt_handle_t hdl);
static int reg_usb_charger_event(CLASS(BattMgrHal) *arg, void (*cb)(usb_charger_event_t evt));
static nrf_saadc_value_t bat_result_buffer[2];

static struct adc_resouce bat_res_mgr= {
	.p_instance = NULL,
	.cur_dev_cfg = NULL,
};


static void saadc_event_callback(const nrf_drv_saadc_evt_t * p_event)
{
			/*notify  task who are waiting for adc read process to be completed*/
			OS_EVENT_SIGNAL_FROM_ISR(bat_res_mgr.bus_busy);

}


/******************************************************************************
 * FunctionName : BattMgrHal_init
 * Description  : BattMgrHal object constuction function
 * Parameters   : CLASS(BattMgrHal) *arg: object pointer
 * Returns          : 0: sucess
 *                   -1: error
 *******************************************************************************/

static int BattMgrHal_init(CLASS(BattMgrHal) *arg)
{
        if (arg == NULL) {
                return -1;
        }
        arg->open = open;
        arg->read = read;
        arg->read_async = read_async;
        arg->reg_usb_charger_event_cb = reg_usb_charger_event;
        arg->close = close;
        //GPADC_INIT();
				if(OS_MUTEX_CREATE(bat_res_mgr.res_lock) == false){
						
				}
				OS_EVENT_CREATE(bat_res_mgr.bus_busy);
				bat_res_mgr.ref_count = 0;
				bat_res_mgr.owner = NULL;
        return 0;
}

/******************************************************************************
 * FunctionName : BattMgrHal_get_instance
 * Description  : for user to get single instance object pointer
 * Parameters   : none
 * Returns          : 0: sucess
 *                   -1: error
 *******************************************************************************/

CLASS(BattMgrHal)* BattMgrHal_get_instance(void)
{
        static CLASS(BattMgrHal) p;
        if (p_instance == NULL) {
                memset(&p, 0, sizeof(p));
                p_instance = &p;
                BattMgrHal_init(p_instance);
        }
        return p_instance;
}

/******************************************************************************
 * FunctionName :open
 * Description  : get batt handle
 * Parameters   : arg: obj pointer
 * Returns          : 0: sucess
 -1: error
 *******************************************************************************/
static batt_handle_t open(CLASS(BattMgrHal) *arg)
{
        N_SPRINTF("[batt hal] batt opened");
		int ret  = 0;
				/*resouce has been taken*/
				if(bat_res_mgr.ref_count > 0){
						/*this means same task request resource here*/
						if(OS_GET_CURRENT_TASK() == bat_res_mgr.owner){
								bat_res_mgr.ref_count ++;
						}else{
								OS_MUTEX_GET(bat_res_mgr.res_lock, OS_MUTEX_FOREVER);
								//ret = -1;
						}
				}else{
						OS_MUTEX_GET(bat_res_mgr.res_lock, OS_MUTEX_FOREVER);
						bat_res_mgr.owner = OS_GET_CURRENT_TASK();
						bat_res_mgr.ref_count ++;
				}	
					/*request spi resouce sucessfully*/
			spi_dev_handle_t* t = (spi_dev_handle_t*)BATT;
			 if(ret >= 0){
					  return (spi_dev_handle_t*)ACC;
			 }else{
						return NULL;//ad_spi_open(ACC_SENSOR);
			 }
        return NULL;//ad_battery_open();
}

static uint16_t read(CLASS(BattMgrHal) *arg, const batt_handle_t hdl)
{
					/*if no hanlder provided , then just return error code*/
				if(hdl == NULL){
						return -1;
				}
				adc_device_config_t *t = (adc_device_config_t*)(hdl);
				
				if(bat_res_mgr.cur_dev_cfg != t){
					if(bat_res_mgr.cur_dev_cfg != NULL){
					  nrf_drv_saadc_uninit();
					  nrf_drv_saadc_channel_uninit(bat_res_mgr.cur_dev_cfg->channel_num);
					}
					 
						bat_res_mgr.cur_dev_cfg = t;
					 
						int err_code = nrf_drv_saadc_init(&(bat_res_mgr.cur_dev_cfg->hw_init), saadc_event_callback);
						APP_ERROR_CHECK(err_code);
						nrf_saadc_channel_config_t   channel_cfg = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(bat_res_mgr.cur_dev_cfg->pin_num);
					  err_code = nrf_drv_saadc_channel_init(bat_res_mgr.cur_dev_cfg->channel_num, &(channel_cfg));
						APP_ERROR_CHECK(err_code);
				}
				nrf_drv_saadc_buffer_convert(bat_result_buffer, 1);
				/*triger new sampleing process in non bloking mode*/
				nrf_drv_saadc_sample();
				uint16_t result = OS_EVENT_WAIT(bat_res_mgr.bus_busy, OS_MS_2_TICKS(5000));
				if(result == pdFALSE){
					/*if event wait timeout event happened*/
					 return 0xffff;
				}
				return bat_result_buffer[0];
}

static void batt_async_read_callback(void *user_data, int value)
{

        if (user_layer_callback != NULL) {
                const batt_handle_t hdl = (const batt_handle_t)user_data;
                //user_layer_callback(ad_battery_raw_to_mvolt(hdl, value));
        }
        return;
}

static uint16_t read_async(CLASS(BattMgrHal) *arg, const batt_handle_t hdl, void (*cb)(int value))
{
        if (cb != NULL) {
                user_layer_callback = cb;
        }
        //ad_battery_read_async(hdl, batt_async_read_callback, hdl);
        return 0;
}

static int close(CLASS(BattMgrHal) *arg, const batt_handle_t hdl)
{
				int ret  = 0;
				/*resouce has been taken*/
				if(bat_res_mgr.ref_count > 0){
						bat_res_mgr.ref_count --;
						/*this means same task request resource here*/
						if(bat_res_mgr.ref_count == 0){
								//spi_res_mgr.ref_count ++;
								OS_MUTEX_PUT(bat_res_mgr.res_lock);
						}
				}
        return ret;//ad_spi_open(dev_id);			
}



static int reg_usb_charger_event(CLASS(BattMgrHal) *arg, void (*cb)(usb_charger_event_t evt))
{
        if (cb != NULL) {
                usb_charger_mgr_event_cb_p = cb;
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return 0;
}

void usb_attach_cb()
{
        latest_event_record = USB_ATTACH_EVENT;
        N_SPRINTF("[batt mgr] usb  event %d arrived ", latest_event_record);
        if (usb_charger_mgr_event_cb_p != NULL) {
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return;
}
void usb_detach_cb(void)
{
        latest_event_record = USB_DETACH_EVENT;
        N_SPRINTF("[batt mgr] usb  event %d arrived ", latest_event_record);
        if (usb_charger_mgr_event_cb_p != NULL) {
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return;
}
void usb_start_enumeration_cb(void)
{
        latest_event_record = USB_START_ENUMMERATION_EVENT;
        N_SPRINTF("[batt mgr] usb  event %d arrived ", latest_event_record);
        if (usb_charger_mgr_event_cb_p != NULL) {
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return;
}
void usb_charging(void)
{
        latest_event_record = USB_CHARGING_EVENT;
        N_SPRINTF("[batt mgr] usb  event %d arrived ", latest_event_record);
        if (usb_charger_mgr_event_cb_p != NULL) {
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return;
}
void usb_precharging(void)
{
        latest_event_record = USB_PRECHARGING_EVENT;
        N_SPRINTF("[batt mgr] usb  event %d arrived ", latest_event_record);
        if (usb_charger_mgr_event_cb_p != NULL) {
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return;
}
void usb_precharging_aborted(void)
{
        latest_event_record = USB_PRECHARGING_ABORTED_EVENT;
        N_SPRINTF("[batt mgr] usb  event %d arrived ", latest_event_record);
        if (usb_charger_mgr_event_cb_p != NULL) {
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return;
}
void usb_charging_stopped(void)
{
        latest_event_record = USB_CHARGING_STOPPED_EVENT;
        N_SPRINTF("[batt mgr] usb  event %d arrived ", latest_event_record);
        if (usb_charger_mgr_event_cb_p != NULL) {
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return;
}
void usb_charging_aborted(void)
{
        latest_event_record = USB_CHARGING_ABORTED_EVENT;
        N_SPRINTF("[batt mgr] usb  event %d arrived ", latest_event_record);
        if (usb_charger_mgr_event_cb_p != NULL) {
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return;
}
void usb_charged(void)
{
        latest_event_record = USB_CHARGED_EVENT;
        N_SPRINTF("[batt mgr] usb  event %d arrived ", latest_event_record);
        if (usb_charger_mgr_event_cb_p != NULL) {
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return;
}
void usb_charger_battery_full(void)
{
        latest_event_record = USB_CHARGER_BATT_FULL_EVENT;
        N_SPRINTF("[batt mgr] usb  event %d arrived ", latest_event_record);
        if (usb_charger_mgr_event_cb_p != NULL) {
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return;
}
void usb_charger_bad_battery(void)
{
        latest_event_record = USB_CHARGER_BAD_BATT_EVENT;
        N_SPRINTF("[batt mgr] usb  event %d arrived ", latest_event_record);
        if (usb_charger_mgr_event_cb_p != NULL) {
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return;
}
void usb_charger_temp_low(void)
{
        latest_event_record = USB_CHARGER_BATT_TMP_LOW_EVENT;
        N_SPRINTF("[batt mgr] usb  event %d arrived ", latest_event_record);
        if (usb_charger_mgr_event_cb_p != NULL) {
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return;
}
void usb_charger_temp_high(void)
{
        latest_event_record = USB_CHARGER_BATT_TMP_HIGH_EVENT;
        N_SPRINTF("[batt mgr] usb  event %d arrived ", latest_event_record);
        if (usb_charger_mgr_event_cb_p != NULL) {
                usb_charger_mgr_event_cb_p(latest_event_record);
        }
        return;
}

#ifdef __cplusplus
}
#endif
