/**
 ****************************************************************************************
 *
 * @file ble_peripheral_task.c
 *
 * @brief BLE peripheral task
 *
 * Copyright (C) 2015. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "osal.h"
#include "stdio.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#define  NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "ble_common.h"
#include  "btle_api.h"
#include "ble_cus.h"
#include "sdk_config.h"
#include "nrf_drv_twi.h"
//#include "ble_gap_mgr.h"
//#include "ble_clingpay.h"



#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                /**< Include the Service Changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION >= 3)
    #define NRF_BLE_MAX_MTU_SIZE        GATT_MTU_SIZE_DEFAULT                   /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#else
    #define NRF_BLE_MAX_MTU_SIZE        BLE_GATT_MTU_SIZE_DEFAULT               /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define CENTRAL_LINK_COUNT               0                                /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "ClingGoNRF52"                     /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "HICling"            /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 300                              /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       180                              /**< The advertising time-out in units of seconds. */

#define APP_TIMER_PRESCALER              0                                /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL      2000                             /**< Battery level measurement interval (ms). */
#define MIN_BATTERY_LEVEL                81                               /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                100                              /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT          1                                /**< Increment between each simulated battery level measurement. */

#define HEART_RATE_MEAS_INTERVAL         1000                             /**< Heart rate measurement interval (ms). */
#define MIN_HEART_RATE                   140                              /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                   300                              /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT             10                               /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL             300                              /**< RR interval interval (ms). */
#define MIN_RR_INTERVAL                  100                              /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                  500                              /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT            1                                /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL 5000                             /**< Sensor Contact Detected toggle interval (ms). */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(400, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(650, UNIT_1_25_MS) /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                    0                                /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)  /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   5000                             /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    30000                            /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE             /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                               /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                       /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define OSTIMER_WAIT_FOR_QUEUE           2                                /**< Number of ticks to wait for the timer queue to be ready */

#define APP_FEATURE_NOT_SUPPORTED        BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */


static TASK_HANDLE_SETS *task_handles = NULL;
static uint16_t  m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static ble_bas_t m_bas;                                   /**< Structure used to identify the battery service. */
static ble_hrs_t m_hrs;                                   /**< Structure used to identify the heart rate service. */
static bool      m_rr_interval_enabled = true;            /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

static sensorsim_cfg_t   m_battery_sim_cfg;               /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;             /**< Battery Level sensor simulator state. */
static sensorsim_cfg_t   m_heart_rate_sim_cfg;            /**< Heart Rate sensor simulator configuration. */
static sensorsim_state_t m_heart_rate_sim_state;          /**< Heart Rate sensor simulator state. */
static sensorsim_cfg_t   m_rr_interval_sim_cfg;           /**< RR Interval sensor simulator configuration. */
static sensorsim_state_t m_rr_interval_sim_state;         /**< RR Interval sensor simulator state. */

static ble_uuid_t m_adv_uuids[] =                         /**< Universally unique service identifiers. */
{
    {BLE_UUID_HEART_RATE_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

static TimerHandle_t m_battery_timer;        /**< Definition of battery timer. */
static TimerHandle_t m_heart_rate_timer;     /**< Definition of heart rate timer. */
static TimerHandle_t m_rr_interval_timer;    /**< Definition of RR interval timer. */
static TimerHandle_t m_sensor_contact_timer; /**< Definition of sensor contact detected timer. */

extern SemaphoreHandle_t m_ble_event_ready;  /**< Semaphore raised if there is a new event to be processed in the BLE thread. */

static TaskHandle_t m_ble_stack_thread;      /**< Definition of BLE stack thread. */
static TaskHandle_t m_logger_thread;         /**< Definition of Logger thread. */

static void advertising_start(void);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
extern void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
/*
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}
*/


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured. Role: %d.  : %d, Procedure: %d\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer time-out.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void battery_level_meas_timeout_handler(TimerHandle_t xTimer)
{
    UNUSED_PARAMETER(xTimer);
    battery_level_update();
}


/**@brief Function for handling the Heart rate measurement timer time-out.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void heart_rate_meas_timeout_handler(TimerHandle_t xTimer)
{
    static uint32_t cnt = 0;
    uint32_t        err_code;
    uint16_t        heart_rate;

    UNUSED_PARAMETER(xTimer);

    heart_rate = (uint16_t)sensorsim_measure(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    cnt++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }

    // Disable RR Interval recording every third heart rate measurement.
    // NOTE: An application will normally not do this. It is done here just for testing generation
    // of messages without RR Interval measurements.
    m_rr_interval_enabled = ((cnt % 3) != 0);
}


/**@brief Function for handling the RR interval timer time-out.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void rr_interval_timeout_handler(TimerHandle_t xTimer)
{
    UNUSED_PARAMETER(xTimer);

    if (m_rr_interval_enabled)
    {
        uint16_t rr_interval;

        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
    }
}


/**@brief Function for handling the Sensor Contact Detected timer time-out.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void sensor_contact_detected_timeout_handler(TimerHandle_t xTimer)
{
    static bool sensor_contact_detected = false;

    UNUSED_PARAMETER(xTimer);

    sensor_contact_detected = !sensor_contact_detected;
    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    m_battery_timer = xTimerCreate("BATT",
                                   BATTERY_LEVEL_MEAS_INTERVAL,
                                   pdTRUE,
                                   NULL,
                                   battery_level_meas_timeout_handler);
    m_heart_rate_timer = xTimerCreate("HRT",
                                      HEART_RATE_MEAS_INTERVAL,
                                      pdTRUE,
                                      NULL,
                                      heart_rate_meas_timeout_handler);
    m_rr_interval_timer = xTimerCreate("RRT",
                                       RR_INTERVAL_INTERVAL,
                                       pdTRUE,
                                       NULL,
                                       rr_interval_timeout_handler);
    m_sensor_contact_timer = xTimerCreate("SCT",
                                          SENSOR_CONTACT_DETECTED_INTERVAL,
                                          pdTRUE,
                                          NULL,
                                          sensor_contact_detected_timeout_handler);

    /* Error checking */
    if ( (NULL == m_battery_timer)
         || (NULL == m_heart_rate_timer)
         || (NULL == m_rr_interval_timer)
         || (NULL == m_sensor_contact_timer) )
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_hrs_init_t hrs_init;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
    uint8_t        body_sensor_location;

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    m_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    m_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    m_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    m_heart_rate_sim_cfg.start_at_max = false;

    sensorsim_init(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    m_rr_interval_sim_cfg.start_at_max = false;

    sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    // Start application timers.
    if (pdPASS != xTimerStart(m_battery_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    if (pdPASS != xTimerStart(m_heart_rate_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    if (pdPASS != xTimerStart(m_rr_interval_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    if (pdPASS != xTimerStart(m_sensor_contact_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);

    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast Adverstising\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for receiving the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected\r\n");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION >= 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}

static void (*on_ble[10])(ble_evt_t * p_ble_evt) = {0};

int ble_on_event_process_reg(void (*p)(ble_evt_t * p_ble_evt))
{
		int i = 0;
		for(i = 0; i < sizeof(on_ble)/sizeof(on_ble[0]); i++){
				if(on_ble[i] == NULL){
						break;
				}
				
		}
		if(i >=  sizeof(on_ble)/sizeof(on_ble[0])){
				return -1;
		}
		if(p != NULL){
				on_ble[i] = p;
		}
		return 1;
}
/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
		for(int i = 0; i < sizeof(on_ble)/sizeof(on_ble[0]); i++){
				if(on_ble[i] != NULL){
						on_ble[i](p_ble_evt);
				}else{
						break;
				}
				
		}
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}


/**
 * @brief Event handler for new BLE events
 *
 * This function is called from the SoftDevice handler.
 * It is called from interrupt level.
 *
 * @return The returned value is checked in the softdevice_handler module,
 *         using the APP_ERROR_CHECK macro.
 */
static uint32_t ble_new_event_handler(void)
{
    BaseType_t yield_req = pdFALSE;
		
	if(task_handles != NULL && task_handles->ble_handle != NULL){
		 OS_TASK_NOTIFY_FROM_ISR(task_handles->ble_handle, BLE_APP_NOTIFY_SOFT_DEVICE_MASK, eSetBits);
	}
    // The returned value may be safely ignored, if error is returned it only means that
    // the semaphore is already given (raised).
    //UNUSED_VARIABLE(xSemaphoreGiveFromISR(m_ble_event_ready, &yield_req));
    portYIELD_FROM_ISR(yield_req);
    return NRF_SUCCESS;
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, ble_new_event_handler);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION >= 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}







/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);

    APP_ERROR_CHECK(err_code);
}
#include "app_twi.h"
#include "sdk_config.h"
#include "pinmap.h"
// TWI (with transaction manager) initialization.
// Set default configuration of LM75B - write 0 to Conf register.
#define CHARGING_I2C_ADDR (0X12>>1)
static uint8_t const default_config[] = { 0X01, 0X95, 0X0A, 0XBC, 0x0B, 0xbc, 0x0C, 0xbc,0x0D, 0x00 };
app_twi_transfer_t const lm75b_init_transfers[1] =
{
    APP_TWI_WRITE(CHARGING_I2C_ADDR, default_config, sizeof(default_config), 0)
};

#include "ble_gap_mgr.h"
void ble_peripheral_task(void *params)
{
        int8_t wdog_id;
				task_handles = (TASK_HANDLE_SETS*)(params);
        // in case services which do not use svc are all disabled, just surpress -Wunused-variable


       // ble_peripheral_task_handle = OS_GET_CURRENT_TASK();


//        task_handle_sets_t *p = (task_handle_sets_t*)params;
//        p_task_sets = p;
//        svc = cling_service_init(NULL, p);
//        ble_service_add(svc);
//        svc = clingpay_service_init(NULL, p);
//        ble_service_add(svc);

//        gap_mgr = GapMgr_get_instance();
//        gap_mgr->register_task(OS_GET_CURRENT_TASK());
//        gap_mgr->init_adv_data(gap_mgr);
//        gap_mgr->start_adv(gap_mgr);
//        size_t dev_id_length;
//        extern uint8_t *SYSTEM_set_dev_id(void);
//        uint8_t *t = (uint8_t*)SYSTEM_set_dev_id();
//        gap_mgr->get_dev_id_str(gap_mgr, t, &dev_id_length);

//        // Initialize application
//        btle_fsm_init();
		uint32_t err_code;
    bool     erase_bonds;

    UNUSED_PARAMETER(params);
		

				
    ble_stack_init();
    peer_manager_init(erase_bonds);
    //gap_params_init();
   // advertising_init();
    services_init();
    sensor_simulator_init();
    //conn_params_init();

		cling_service_init(NULL, task_handles);
		
		CLASS(GapMgr)* gap_mgr = GapMgr_get_instance();
    gap_mgr->register_task(OS_GET_CURRENT_TASK());
		gap_mgr->set_dev_name(gap_mgr, "RBAND");
		gap_mgr->switch_2_fast_conn_mode(gap_mgr);
    gap_mgr->init_adv_data(gap_mgr);
    gap_mgr->start_adv(gap_mgr);
    //APP_ERROR_CHECK(err_code);
        for (;;) {
         OS_BASE_TYPE ret  __attribute__((unused));
         uint32_t notif;
        /* Wait for event from SoftDevice */
//        while (pdFALSE == xSemaphoreTake(m_ble_event_ready, portMAX_DELAY))
//        {
//            // Just wait again in the case when INCLUDE_vTaskSuspend is not enabled
//        }
				/*
				 * Wait on any of the notification bits, then clear them all
				 */
        ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif,
                        OS_TASK_NOTIFY_FOREVER);
				/* Blocks forever waiting for task notification. The return value must be OS_OK */
				//OS_ASSERT(ret == OS_OK);
				if(notif & BLE_APP_NOTIFY_SOFT_DEVICE_MASK){
        // This function gets events from the SoftDevice and processes them by calling the function
        // registered by softdevice_ble_evt_handler_set during stack initialization.
        // In this code ble_evt_dispatch would be called for every event found.
        intern_softdevice_events_execute();				
				}



                if (notif & BLE_APP_NOTIFY_FSM_MASK) {
                        btle_fsm_process();
                }

                if(notif & BLE_APP_NOTIFY_CLING_STRAT_ADV_REQUEST_MASK){
                        gap_mgr->start_adv(gap_mgr);
                }

                if(notif & BLE_APP_NOTIFY_CLING_ADV_SWITCH_SPEED_REQUEST_MASK){
                        gap_mgr->adv_freq_swicth_process(gap_mgr);
                }
//                extern int clingpay_mac_fsm();
//                if (notif & BLE_APP_NOTIFY_FSM_MASK) {
//                        clingpay_mac_fsm();
//                }
//                extern int clingpay_cmd_fsm();
//                if (notif & BLE_APP_NOTIFY_CLING_PAY_CMD_MASK) {
//                        clingpay_cmd_fsm();
//                }
//                int get_balance_fsm();
//                if (notif & BLE_APP_NOTIFY_CLING_PAY_BALANCE_MASK) {
//                        get_balance_fsm();
//                }
//                extern int CLNGPAY_balance_callback(uint32_t balance);
//                if (notif & BLE_APP_NOTIFY_CLING_PAY_BALANCE_SYNC_MASK) {
//                        if (OS_TIMER_IS_ACTIVE(balance_sync_timer)) {
//                                OS_TIMER_STOP(balance_sync_timer, 0);
//                        }
//                        OS_TIMER_START(balance_sync_timer, 0);
//                }

//                if(notif & BLE_APP_NOTIFY_CLING_PAY_BALANCE_SYNC_TIMEOUT_MASK){
//                        if (OS_TIMER_IS_ACTIVE(balance_sync_timer)) {
//                                OS_TIMER_STOP(balance_sync_timer, 0);
//                        }
//                        extern int get_balance(int(*cb)(uint32_t balance));
//                        get_balance(CLNGPAY_balance_callback);
//                }


//#if CFG_CTS
//                if (notif & CTS_SET_TIME_NOTIF) {
//                        cts_notify_time_all(cts, &cts_time);
//                }
//#endif
        }
}
