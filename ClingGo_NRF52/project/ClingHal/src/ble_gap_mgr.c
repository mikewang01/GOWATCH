/******************************************************************************
 * Copyright 2013-2014 hicling Systems (MikeWang)
 *
 * FileName: rtc_hal.c
 *
 * Description: abstract layer between cling sdk and freertos.
 *
 * Modification history:
 *     2016/7/22, v1.0 create this file mike.
 *******************************************************************************/

#include "oop_hal.h"
#include "rtc_hal.h"
#include "osal.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "ble_gap_mgr.h"
#include "ble_advertising.h"
#include "ble_gap.h"
#include "ancs.h"
#include "ble_hci.h"
#include "ble_gap.h"
/*********************************************************************
 * MACROS
 */
#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * TYPEDEFS
 */
#define MAX_DEV_NAME_LENGTH  16
#define DEVICE_RAIN_FLOWER  1
#if     DEVICE_RAIN_FLOWER
#define DEVICE_NAME_HEADER  "MBAND "
#endif

#define DEVICE_ID_STRING_HEADER "HICR0000"

#define BLE_UUID_WECHAT_SERVICE                                                       0xFEE7
#define COMPANY_IDENTIFIER                                                            0x0056

#define ADV_SPEED_SWITCH_TIMEOUT      10000  //10s advertisement frequency switch time-out
#define ADV_FAST_ADV_TIMEOUT_PERIOD   (320)              //x0.625 ms  200ms
#define ADV_SLOW_ADV_TIMEOUT_PERIOD   (1600)             //x0.625 ms  1000ms

struct adv_interval {
        uint16_t min;
        uint16_t max;
};
/*********************************************************************
 * GLOBAL VARIABLES
 */
/*********************************************************************
 * LOCAL VARIABLES
 */
#define BLE_CONN_IDX_INVALID 0xff

static CLASS(GapMgr)* p_instance = NULL;
static uint8_t dev_name[MAX_DEV_NAME_LENGTH];
static uint16_t conn_hdl = BLE_CONN_IDX_INVALID;

enum adv_mode {
        ADV_STOP_MODE = 0,
        ADV_FAST_MODE,
        ADV_SLOW_MODE,
};

struct ble_gap_mgr_oop {
        struct adv_interval adv_fast_interval;
        struct adv_interval adv_slow_interval;
        enum adv_mode mode;
        OS_MUTEX ble_gap_mgr_mutex;
        OS_TIMER ble_timer;
        CLASS(GapMgr) *this;
};

static struct ble_gap_mgr_oop ble_gap_para_prop = {
        .adv_fast_interval = {
                .min = ADV_FAST_ADV_TIMEOUT_PERIOD,
                .max = ADV_FAST_ADV_TIMEOUT_PERIOD,
        },
        .adv_slow_interval = {
                .min = ADV_SLOW_ADV_TIMEOUT_PERIOD,
                .max = ADV_SLOW_ADV_TIMEOUT_PERIOD,
        },
        .mode = ADV_STOP_MODE,
        .ble_gap_mgr_mutex = NULL,
        .ble_timer = NULL,
        .this = NULL,
};

static const struct adv_interval adv_fast_interval; 
/** GAP connection parameters structure */
typedef struct {
        uint16_t interval_min;                          ///< Minimum connection interval
        uint16_t interval_max;                          ///< Maximum connection interval
        uint16_t slave_latency;                         ///< Slave latency
        uint16_t sup_timeout;                           ///< Supervision timeout
} gap_conn_params_t;

static const gap_conn_params_t fast_conn_params = {
        .interval_min = MSEC_TO_UNITS(20, UNIT_1_25_MS), /**< Minimum acceptable connection interval (40 milli-seconds). */
        .interval_max = MSEC_TO_UNITS(40, UNIT_1_25_MS),/**< Maximum acceptable connection interval (1 second). */
        .slave_latency = 4,/**< Slave latency. */
        .sup_timeout = MSEC_TO_UNITS(2000, UNIT_10_MS) , /**< Connection supervisory timeout (2 seconds). */
};
static const gap_conn_params_t slow_conn_params = {
        .interval_min = MSEC_TO_UNITS(250, UNIT_1_25_MS),/**< Minimum acceptable connection interval (40 milli-seconds). */
        .interval_max = MSEC_TO_UNITS(400, UNIT_1_25_MS),/**< Maximum acceptable connection interval (1 second). */
        .slave_latency = 4,/**< Slave latency. */
        .sup_timeout = MSEC_TO_UNITS(60000, UNIT_10_MS) , /**< Connection supervisory timeout (6 seconds). */
};

static const uint16_t adv_uuids[] = { BLE_UUID_WECHAT_SERVICE,
        0xffe0,
        0xffe1
};
static OS_TASK reg_task = NULL;
//static OS_TIMER rtc_timer;
/*********************************************************************
 * LOCAL DECLARATION
 */

static int get_device_id_str(CLASS(GapMgr) *arg, uint8_t *dev_id, size_t *size);
static int set_adv_data(CLASS(GapMgr) *arg);
static int stop_adv(CLASS(GapMgr) *arg);
static int start_adv(CLASS(GapMgr) *arg);
static int disconnect(CLASS(GapMgr) *arg);
static int event_process(CLASS(GapMgr) *arg, void *hdr);
static int reboot_on_diconnect(CLASS(GapMgr) *arg);
static int set_fast_conn_para(CLASS(GapMgr) *arg);
static int set_slow_conn_para(CLASS(GapMgr) *arg);
static int adv_switch_to_fast_mode(CLASS(GapMgr) *arg);
static void adv_timeout_cb(OS_TIMER t);
static int register_task(OS_TASK p_task);
static int adv_freq_swicth_process(CLASS(GapMgr) *arg);
static int set_device_name(CLASS(GapMgr) *arg, uint8_t *name_str);
static int get_device_mac_addr(CLASS(GapMgr) *arg, uint8_t *mac_p);
int GapMgr_init(CLASS(GapMgr) *arg)
{
        if (arg == NULL) {
                return -1;
        }
        if (OS_MUTEX_CREATE(ble_gap_para_prop.ble_gap_mgr_mutex) == OS_MUTEX_CREATE_FAILED) {
                return -1;
        }
        /*used to maintain adv switch algrithom*/
        ble_gap_para_prop.ble_timer = OS_TIMER_CREATE("ble_timer",
                OS_MS_2_TICKS(ADV_SPEED_SWITCH_TIMEOUT), pdTRUE, (void* )0, adv_timeout_cb);
        arg->start_adv = start_adv;
        arg->stop_adv = stop_adv;
        arg->init_adv_data = set_adv_data;
        arg->get_dev_id_str = get_device_id_str;
				arg->set_dev_name = set_device_name;
        arg->disconnect = disconnect;
        arg->event_process = event_process;
        arg->reboot_on_disconnect = reboot_on_diconnect;
        arg->switch_2_fast_conn_mode = set_fast_conn_para;
        arg->switch_2_slow_conn_mode = set_slow_conn_para;
        arg->register_task = register_task;
        arg->adv_freq_swicth_process = adv_freq_swicth_process;
        ble_gap_para_prop.this = arg;
        return 0;
}
/******************************************************************************
 * FunctionName :GapMgr_get_instance
 * Description  : get hal ble gap mgr instance
 * Parameters   :
 * Returns          : 0: sucess
 -1: error
 *******************************************************************************/
CLASS(GapMgr)* GapMgr_get_instance(void)
{
        static CLASS(GapMgr) p;
        if (p_instance == NULL) {
                p_instance = &p;
                memset(&p, 0, sizeof(p));
                GapMgr_init(p_instance);
        }
        return p_instance;
}

static int register_task(OS_TASK p_task)
{
        reg_task = p_task;
        return 0;
}

static void adv_timeout_cb(OS_TIMER t)
{
        /*if tomer is active stiop it */
        OS_TIMER_STOP(ble_gap_para_prop.ble_timer, 0);
        if (reg_task != NULL) {
                OS_TASK_NOTIFY(reg_task, BLE_APP_NOTIFY_CLING_ADV_SWITCH_SPEED_REQUEST_MASK,
                        OS_NOTIFY_SET_BITS);
        }else{
                /*do nothing if no task registered*/

        }
}

/******************************************************************************
 * FunctionName : set_slow_conn_para
 * Description  : set conn para to slow
 * Parameters   : object pointer
 * Returns          : 0: sucess
 -1: error
 *******************************************************************************/
static int set_slow_conn_para(CLASS(GapMgr) *arg)
{
//        ble_dev_params_t *ble_dev_params = ble_mgr_dev_params_acquire();
//        // Check if the attribute database configuration bit flag needs updating
//        ble_dev_params->att_db_cfg &= (~GAPM_MASK_ATT_SLV_PREF_CON_PAR_EN);
//        ble_mgr_dev_params_release();
					uint32_t                err_code;
					ble_gap_conn_params_t   gap_conn_params;


					memset(&gap_conn_params, 0, sizeof(gap_conn_params));

					gap_conn_params.min_conn_interval = slow_conn_params.interval_min;
					gap_conn_params.max_conn_interval = slow_conn_params.interval_max;
					gap_conn_params.slave_latency     = slow_conn_params.slave_latency;
					gap_conn_params.conn_sup_timeout  = slow_conn_params.sup_timeout;

					err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
					APP_ERROR_CHECK(err_code);
																					
        return 1;//ble_gap_per_pref_conn_params_set(&slow_conn_params);
}

__attribute__((unused)) static int update_conn_para(CLASS(GapMgr) *arg, uint16_t conn_idx,
        const gap_conn_params_t *conn_params)
{
        return 1;//ble_l2cap_conn_param_update(conn_idx, conn_params);
}
/******************************************************************************
 * FunctionName : set_fast_conn_para
 * Description  : set conn para to slow
 * Parameters   : object pointer
 * Returns          : 0: sucess
 -1: error
 *******************************************************************************/

static int set_fast_conn_para(CLASS(GapMgr) *arg)
{

    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;


    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = fast_conn_params.interval_min;
    gap_conn_params.max_conn_interval = fast_conn_params.interval_max;
    gap_conn_params.slave_latency     = fast_conn_params.slave_latency;
    gap_conn_params.conn_sup_timeout  = fast_conn_params.sup_timeout;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

		return err_code;
}

/******************************************************************************
 * FunctionName :start_adv
 * Description  : start adv order
 * Parameters   : object pointer
 * Returns          : 0: sucess
 -1: error
 *******************************************************************************/

static int start_adv(CLASS(GapMgr) *arg)
{
        /*this means device is under conditon of disconnected*/
        if (conn_hdl == BLE_CONN_IDX_INVALID) {
                return adv_switch_to_fast_mode(arg);
        }
        return -1;
}

/******************************************************************************
 * FunctionName :stop_adv
 * Description  : stop adv order
 * Parameters   : object pointer
 * Returns          : 0: sucess
 -1: error
 *******************************************************************************/

static int stop_adv(CLASS(GapMgr) *arg)
{

        /*if tomer is active stiop it */
        if (OS_TIMER_IS_ACTIVE(ble_gap_para_prop.ble_timer)) {
                OS_TIMER_STOP(ble_gap_para_prop.ble_timer, 0);
        }
        OS_MUTEX_GET(ble_gap_para_prop.ble_gap_mgr_mutex, 0);
        ble_gap_para_prop.mode = ADV_STOP_MODE;
        int ret = 0;//ble_gap_adv_stop();
				(void) sd_ble_gap_adv_stop();
        OS_MUTEX_PUT(ble_gap_para_prop.ble_gap_mgr_mutex);
        return ret;

}

static int adv_switch_to_fast_mode(CLASS(GapMgr) *arg)
{
        int ret = 0;
        OS_MUTEX_GET(ble_gap_para_prop.ble_gap_mgr_mutex, 0);
        /*if timer is active stiop it */
        if (OS_TIMER_IS_ACTIVE(ble_gap_para_prop.ble_timer)) {
                OS_TIMER_STOP(ble_gap_para_prop.ble_timer, 0);
        }

        if (ble_gap_para_prop.mode != ADV_FAST_MODE) {
               
                ble_gap_para_prop.mode = ADV_FAST_MODE;
								int err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
        }

        OS_TIMER_START(ble_gap_para_prop.ble_timer, 0);
        OS_MUTEX_PUT(ble_gap_para_prop.ble_gap_mgr_mutex);
        return ret;

}

static int adv_switch_to_slow_mode(CLASS(GapMgr) *arg)
{
        int ret = 0;
        OS_MUTEX_GET(ble_gap_para_prop.ble_gap_mgr_mutex, 0);
        if (OS_TIMER_IS_ACTIVE(ble_gap_para_prop.ble_timer)) {
                OS_TIMER_STOP(ble_gap_para_prop.ble_timer, 0);
        }
        /*EXCUTE SLOW SPPED SWITCH ONLY WHEN IT SI NOT IN SLOW MODE*/
        if (ble_gap_para_prop.mode != ADV_SLOW_MODE) {
              //  ret = ble_gap_adv_stop();
//                arg->init_adv_data(arg);
                //ble_gap_adv_intv_set(ble_gap_para_prop.adv_slow_interval.min,
                    //    ble_gap_para_prop.adv_slow_interval.max);
                ble_gap_para_prop.mode = ADV_SLOW_MODE;
                //ret = ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
        }
        OS_TIMER_START(ble_gap_para_prop.ble_timer, 0);
        OS_MUTEX_PUT(ble_gap_para_prop.ble_gap_mgr_mutex);
        return ret;

}

static int adv_freq_swicth_process(CLASS(GapMgr) *arg)
{
        OS_MUTEX_GET(ble_gap_para_prop.ble_gap_mgr_mutex, 0);
        if (ble_gap_para_prop.mode == ADV_FAST_MODE) {
                adv_switch_to_slow_mode(ble_gap_para_prop.this);
        }
        else if (ble_gap_para_prop.mode == ADV_SLOW_MODE) {
                               stop_adv(NULL);
        }
        else {
                                stop_adv(NULL);
        }
        OS_MUTEX_PUT(ble_gap_para_prop.ble_gap_mgr_mutex);
        return 1;
}

/**@b rief Function for initializing the Advertising functionality.
 */

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 *
 * Required by Apple, the advertising data should at least include:
 *
 * 1) Flags, 2) TX Power level, 3) Local name, 4) Services
 *
 */
// Required by Apple: adverting interval of 1000 ms for at least 60 seconds
//
#define APP_ADV_INTERVAL_FAST                     160                                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 100 ms), Apple suggested. */
#define APP_ADV_TIMEOUT_FAST_IN_SECONDS           30                                        /**< The advertising timeout in units of seconds. */

#define APP_ADV_INTERVAL_SLOW                     1636                                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 1022.5 ms), Apple suggested. */
#define APP_ADV_TIMEOUT_SLOW_IN_SECONDS           240  

static  ble_uuid_t m_adv_uuids[] = {{ANCS_UUID_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}}; /**< Universally unique service identifiers. */
static void _advertising_init(uint8_t       flags)
#ifndef __WECHAT_SUPPORTED__
{
#ifndef _CLING_PC_SIMULATION_
	uint32_t      err_code;
	ble_advdata_t advdata;
	ble_advdata_t srdata;
	ble_adv_modes_config_t adv_cfg = {
	  .ble_adv_whitelist_enabled = false,       /**< Enable or disable use of the whitelist. */
    .ble_adv_directed_enabled = false,       /**< Enable or disable direct advertising mode. */
    .ble_adv_directed_slow_enabled = false,   /**< Enable or disable direct advertising mode. */
    .ble_adv_fast_enabled = true,            /**< Enable or disable fast advertising mode. */
    .ble_adv_slow_enabled = true,            /**< Enable or disable slow advertising mode. */
    .ble_adv_fast_interval = APP_ADV_INTERVAL_FAST,           /**< Advertising interval for fast advertising. */
    .ble_adv_fast_timeout = APP_ADV_TIMEOUT_FAST_IN_SECONDS,           /**< Time-out (in seconds) for fast advertising. */
    .ble_adv_slow_interval = APP_ADV_INTERVAL_SLOW,           /**< Advertising interval for slow advertising. */
    .ble_adv_slow_timeout = APP_ADV_TIMEOUT_SLOW_IN_SECONDS,           /**< Time-out (in seconds) for slow advertising. */
	};
	ble_uuid_t adv_uuids[] = {
			{0xffe0,         BLE_UUID_TYPE_BLE}, 
			{0xffe1,         									BLE_UUID_TYPE_BLE},
	};

	// Build and set advertising data.
	memset(&advdata, 0, sizeof(advdata));

	// Keep minimum set of data during advertising
	memset(&advdata, 0, sizeof(advdata));
	advdata.name_type               = BLE_ADVDATA_NO_NAME;
	advdata.include_appearance      = false;
	advdata.flags                   = (flags);
	advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
	advdata.uuids_complete.p_uuids  = adv_uuids;

//#ifdef _ENABLE_ANCS_
//	advdata.uuids_solicited.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
//	advdata.uuids_solicited.p_uuids  = m_adv_uuids;
//#endif
	
	// Put full name in scan response data, saving power
	memset(&srdata, 0, sizeof(srdata));
	srdata.name_type 								= BLE_ADVDATA_FULL_NAME;

	err_code = ble_advertising_init(&advdata, &srdata, &adv_cfg, NULL, NULL);
	APP_ERROR_CHECK(err_code);
#endif
}
#else
{
#ifndef _CLING_PC_SIMULATION_
	uint32_t      err_code;
	ble_advdata_t advdata;
	ble_advdata_t rspdata;
	uint8_t	 	m_addl_adv_manuf_data[BLE_GAP_ADDR_LEN];
	ble_uuid_t adv_uuids[] = {
			{BLE_UUID_WECHAT_SERVICE,         BLE_UUID_TYPE_BLE},
			{0xffe0,         									BLE_UUID_TYPE_BLE},
			{0xffe1,         									BLE_UUID_TYPE_BLE},
	};
	ble_adv_modes_config_t adv_cfg = {
	  .ble_adv_whitelist_enabled = false,       /**< Enable or disable use of the whitelist. */
    .ble_adv_directed_enabled = false,       /**< Enable or disable direct advertising mode. */
    .ble_adv_directed_slow_enabled = false,   /**< Enable or disable direct advertising mode. */
    .ble_adv_fast_enabled = true,            /**< Enable or disable fast advertising mode. */
    .ble_adv_slow_enabled = true,            /**< Enable or disable slow advertising mode. */
    .ble_adv_fast_interval = APP_ADV_INTERVAL_FAST,           /**< Advertising interval for fast advertising. */
    .ble_adv_fast_timeout = APP_ADV_TIMEOUT_FAST_IN_SECONDS,           /**< Time-out (in seconds) for fast advertising. */
    .ble_adv_slow_interval = APP_ADV_INTERVAL_SLOW,           /**< Advertising interval for slow advertising. */
    .ble_adv_slow_timeout = APP_ADV_TIMEOUT_SLOW_IN_SECONDS,           /**< Time-out (in seconds) for slow advertising. */
	};
#if 1
	/*fill up advertisement configration*/
	uint8_t mac_address[BLE_GAP_ADDR_LEN];
	get_mac_addr(mac_address);
	memcpy(m_addl_adv_manuf_data, mac_address, BLE_GAP_ADDR_LEN);
	// Build and set advertising data
	memset(&advdata, 0, sizeof(advdata));
	advdata.name_type               = BLE_ADVDATA_NO_NAME;
	advdata.include_appearance      = false;
	advdata.flags                   = (flags);
	advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
	advdata.uuids_complete.p_uuids  = adv_uuids;

#endif
#ifdef _ENABLE_ANCS_
	advdata.uuids_solicited .uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	advdata.uuids_solicited.p_uuids  = m_adv_uuids;
#endif
	/*initiate mac information and company id  as the requirement of wechat*/
	ble_advdata_manuf_data_t        manuf_data;
	manuf_data.company_identifier = COMPANY_IDENTIFIER;
	manuf_data.data.size          = sizeof(m_addl_adv_manuf_data);
	manuf_data.data.p_data        = m_addl_adv_manuf_data;	
	// Put full name in scan response data and mac information needed by wechat, saving power
	memset(&rspdata, 0, sizeof(rspdata));
	rspdata.name_type               = BLE_ADVDATA_FULL_NAME;
	rspdata.p_manuf_specific_data = &manuf_data;
	//ble_advdata_manuf_data_t
	err_code = ble_advertising_init(&advdata, &srdata, &adv_cfg, NULL, NULL);
	APP_ERROR_CHECK(err_code);
#endif
}
#endif
static int set_adv_data(CLASS(GapMgr) *arg)
{
      _advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
			return 0;
}

static int set_device_name(CLASS(GapMgr) *arg, uint8_t *name_str)
{
        if ((name_str) == NULL) {
                return -1;
        }																					
				uint8_t  mac_adr[BLE_GAP_ADDR_LEN];
			  get_device_mac_addr(arg, mac_adr);
				memcpy(dev_name, name_str, strlen((char*)name_str));
				sprintf((char*)&dev_name[strlen((char*)name_str)], " %02X%02X", mac_adr[BLE_GAP_ADDR_LEN - 1], mac_adr[BLE_GAP_ADDR_LEN - 2]);																				
				ble_gap_conn_sec_mode_t sec_mode;

				BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

				int err_code = sd_ble_gap_device_name_set(&sec_mode,
																								(const uint8_t *)dev_name,
																								strlen((char*)dev_name));
				APP_ERROR_CHECK(err_code);

				err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
				APP_ERROR_CHECK(err_code);
																								
        
																								
				return 1;// ble_gap_device_name_set((const char*)dev_name, ATT_PERM_READ);
}

static int get_device_id_str(CLASS(GapMgr) *arg, uint8_t *dev_id, size_t *size)
{
        char mac_addr_str[14];
      //  if (ble_gap_address_get(&mac_addr) != BLE_STATUS_OK) {
    //            return -1;
  //      }
				uint32_t error_code;
				ble_gap_addr_t mac_addr;
				error_code = sd_ble_gap_addr_get(&mac_addr);
				APP_ERROR_CHECK(error_code);
        /*transfer mac adress from data to string*/
        sprintf((char*)mac_addr_str, "%02X%02X%02X%02X%02X%02X", mac_addr.addr[5], mac_addr.addr[4],
                mac_addr.addr[3], mac_addr.addr[2], mac_addr.addr[1], mac_addr.addr[0]);
				Y_SPRINTF("mac adress = %02X%02X%02X%02X%02X%02X", mac_addr.addr[5], mac_addr.addr[4],
                mac_addr.addr[3], mac_addr.addr[2], mac_addr.addr[1], mac_addr.addr[0]);
        /*prepare device name string*/
        uint8_t *p = dev_id;
        memcpy(p, DEVICE_ID_STRING_HEADER, strlen(DEVICE_ID_STRING_HEADER));
        p += strlen(DEVICE_ID_STRING_HEADER);
        memcpy(p, mac_addr_str, strlen(mac_addr_str));
        p += strlen(mac_addr_str);
        *p = '\0';
        *size = p - dev_id;
        return 0;
}
static int get_device_mac_addr(CLASS(GapMgr) *arg, uint8_t *mac_p)
{
        uint8_t mac_addr_str[14]   __attribute__((unused));
       // if (ble_gap_address_get(&mac_addr) != BLE_STATUS_OK) {
     //           return -1;
   //     }
				if(mac_p == NULL){
						return -1; 
				}
				uint32_t error_code;
				ble_gap_addr_t mac_addr;
				error_code = sd_ble_gap_addr_get(&mac_addr);
				APP_ERROR_CHECK(error_code);
				uint8_t *d = mac_addr.addr;
				for ( uint8_t i = BLE_GAP_ADDR_LEN; i > 0;) {
						i--;
						mac_addr_str[BLE_GAP_ADDR_LEN - i - 1] = d[i];
				}
				
        memcpy(mac_p, mac_addr.addr, sizeof(mac_addr.addr));
        return 0;
}

static int disconnect(CLASS(GapMgr) *arg)
{
        if (conn_hdl != BLE_CONN_IDX_INVALID) {
                	int err_code = sd_ble_gap_disconnect(conn_hdl, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION ); //BLE_HCI_STATUS_CODE_SUCCESS);
									APP_ERROR_CHECK(err_code);
        }
        return 0;
}

static void periph_deinit(void)
{
        /* Timer1 is used for ticks in OS disable it for now */
       // hw_timer1_disable();
}

static void reboot(void)
{
        /*
         * Reset platform
         */
        __disable_irq();
				NVIC_SystemReset();
        //REG_SETF(CRG_TOP, SYS_CTRL_REG, SW_RESET, 1);
}

static int reboot_on_diconnect(CLASS(GapMgr) *arg)
{
        if (conn_hdl != BLE_CONN_IDX_INVALID) {
                //ble_gap_disconnect(conn_hdl, BLE_HCI_ERROR_REMOTE_USER_TERM_CON);
        }
        int32_t *int_vector_table = (int32_t *)0;
        int32_t *image_address = (int32_t*)0x8000000;
        Y_SPRINTF("Starting image at 0x%X, reset vector 0x%X.\r\n", (unsigned int)image_address,
                (unsigned int)image_address[1]);
        /*
         * In OS environment some interrupt could already be enabled, disable all before
         * interrupt vectors are changed.
         */
        __disable_irq();

        /* Copy interrupt vector table from image */
        memcpy(int_vector_table, image_address, 0x100);
        /*
         * If bootloader changed any configuration (GPIO, clocks) it should be uninitialized here
         */
        periph_deinit();
        reboot();
        return 0;
}





static int event_process(CLASS(GapMgr) *arg, void *hdr)
{
	
			  ble_evt_t * p_ble_evt = (ble_evt_t *)hdr;
        switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED: {
                Y_SPRINTF("[GAPMGR]: stop adv");
                stop_adv(arg);
                conn_hdl = p_ble_evt->evt.gap_evt.conn_handle;
//                gap_conn_params_t id;
//                ble_gap_per_pref_conn_params_get(&id);
//                update_conn_para(arg, conn_hdl, &slow_conn_params);
                 //ble_gap_conn_param_update(conn_hdl, &fast_conn_params);
                  set_fast_conn_para(arg);
        }
                break;
        case BLE_GAP_EVT_DISCONNECTED:
                /*first mark */
                conn_hdl = BLE_CONN_IDX_INVALID;
                start_adv(arg);
                Y_SPRINTF("[GAPMGR]: start adv");
                break;
        case BLE_GAP_EVT_CONN_PARAM_UPDATE: {
               // ble_evt_gap_conn_param_updated_t *p = (ble_evt_gap_conn_param_updated_t *)hdr;
                conn_hdl = p_ble_evt->evt.gap_evt.conn_handle;
        }
                break;
        }
        return 0;
}
#ifdef __cplusplus
}
#endif
