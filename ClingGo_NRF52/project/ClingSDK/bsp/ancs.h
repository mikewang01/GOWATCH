/***************************************************************************//**
 * File ancs.h
 * 
 * Description: ancs service context.
 *
 *
 ******************************************************************************/
#ifndef _ANCS_H__
#define _ANCS_H__
#include "standard_types.h"
#include "ble.h"

#define _ENABLE_ANCS_

#ifndef _CLING_PC_SIMULATION_

/**@brief Category IDs for iOS notifications. */
typedef enum
{
  BLE_ANCS_CATEGORY_ID_OTHER,                /**< The iOS notification belongs to the "other" category.  */
  BLE_ANCS_CATEGORY_ID_INCOMING_CALL,        /**< The iOS notification belongs to the "Incoming Call" category. */
  BLE_ANCS_CATEGORY_ID_MISSED_CALL,          /**< The iOS notification belongs to the "Missed Call" category. */
  BLE_ANCS_CATEGORY_ID_VOICE_MAIL,           /**< The iOS notification belongs to the "Voice Mail" category. */
  BLE_ANCS_CATEGORY_ID_SOCIAL,               /**< The iOS notification belongs to the "Social" category. */
  BLE_ANCS_CATEGORY_ID_SCHEDULE,             /**< The iOS notification belongs to the "Schedule" category. */
  BLE_ANCS_CATEGORY_ID_EMAIL,                /**< The iOS notification belongs to the "E-mail" category. */
  BLE_ANCS_CATEGORY_ID_NEWS,                 /**< The iOS notification belongs to the "News" category. */
  BLE_ANCS_CATEGORY_ID_HEALTH_AND_FITNESS,   /**< The iOS notification belongs to the "Health and Fitness" category. */
  BLE_ANCS_CATEGORY_ID_BUSINESS_AND_FINANCE, /**< The iOS notification belongs to the "Buisness and Finance" category. */
  BLE_ANCS_CATEGORY_ID_LOCATION,             /**< The iOS notification belongs to the "Location" category. */
  BLE_ANCS_CATEGORY_ID_ENTERTAINMENT         /**< The iOS notification belongs to the "Entertainment" category. */
} ble_ancs_category_id_t;


#ifdef _ENABLE_ANCS_

#define ANCS_UUID_SERVICE                            0xF431  /**< 16-bit service UUID for the Apple Notification Center Service. */

#define ANCS_START_SERVICE_DISCOVERY_DELAY_TIME      1000    /*1s*/ 
#define ANCS_FILTERING_OLD_MSG_DELAY_TIME            10000   /*10s*/ 
#define ANCS_DISCOVERY_FAIL_DISCONNECT_DELAY_TIME    60000   /*60s*/  
#define ANCS_SUPPORT_MAX_TITLE_LEN                   64      /*64 byte*/
#define ANCS_SUPPORT_MAX_MESSAGE_LEN                 192     /*192 byte*/
#define ANCS_PARSE_NOTIF_ATTRIBUTE_TIMEOUT           15000   /*15s*/

#define ANCS_BOND_STATE_NUKNOW_FLAG                  0x01   
#define ANCS_BOND_STATE_SUCCESSED_FLAG               0x02                   
#define ANCS_BOND_STATE_FAIL_FLAG                    0x04
typedef enum
{
  PARSE_STAT_COMMAND_ID,
  PARSE_STAT_NOTIFICATION_UID_1,
  PARSE_STAT_NOTIFICATION_UID_2,
  PARSE_STAT_NOTIFICATION_UID_3,
  PARSE_STAT_NOTIFICATION_UID_4,
  PARSE_STAT_ATTRIBUTE_TITLE_ID,
  PARSE_STAT_ATTRIBUTE_TITLE_LEN1,
  PARSE_STAT_ATTRIBUTE_TITLE_LEN2,
  PARSE_STAT_ATTRIBUTE_TITLE_READY,
  PARSE_STAT_ATTRIBUTE_MESSAGE_ID,
  PARSE_STAT_ATTRIBUTE_MESSAGE_LEN1,
  PARSE_STAT_ATTRIBUTE_MESSAGE_LEN2,
  PARSE_STAT_ATTRIBUTE_MESSAGE_READY
} ANCS_PARSE_STATES;

/**@brief Event types that are passed from client to application on an event. */
typedef enum
{
  BLE_ANCS_EVT_NULL,  	
  BLE_ANCS_EVT_DISCOVER_COMPLETE,          
  BLE_ANCS_EVT_DISCOVER_FAILED,            
  BLE_ANCS_EVT_NOTIF_REQ,                                         
} ble_ancs_evt_type;

enum
{
  BLE_ANCS_STATE_IDLE,   
  BLE_ANCS_STATE_START_DISCOVER,  		
  BLE_ANCS_STATE_WAITING_PARSE_COMPLETE,                   
  BLE_ANCS_STATE_STORE_NOTIF_ATTRIBUTE        
};

typedef struct tagANCS_CONTEXT {
  // Global state
  I8U  state;	

  // Parse attrs state
  ANCS_PARSE_STATES	 parse_state;
	
  // Stored notific	data
  I8U  buf[256];	
	
  // The number of the notification
  I8U  message_total;		
	
  // Notify reminder switch by user set.
  I16U supported_categories;
	
  // Filtering old notify flag.
  I8U filtering_flag;
	
  // Record time flag to filtering old notifications.
  BOOLEAN start_record_time;	
	
	I32U parse_time;
	
	// Pair state
	BOOLEAN bond_flag;
	
	I32U dis_time;
} ANCS_CONTEXT;


/**@brief Function for handling the application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack that are of interest to the ANCS client.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
void  ANCS_on_ble_evt(const ble_evt_t * p_ble_evt);
void  ANCS_on_event_handling(ble_ancs_evt_type evt_type);
void  ANCS_service_add(void);
void  ANCS_nflash_store_one_message(I8U *data);
void  ANCS_state_machine(void);
#endif

#endif

#endif // _ANCS_H__

/** @} */
