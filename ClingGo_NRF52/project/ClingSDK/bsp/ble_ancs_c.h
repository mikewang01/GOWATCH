///* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
// *
// * The information contained herein is property of Nordic Semiconductor ASA.
// * Terms and conditions of usage are described in detail in NORDIC
// * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
// *
// * Licensees are granted free, non-transferable use of the information. NO
// * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
// * the file.
// */
//
///** @file
// *
// * @defgroup ble_sdk_srv_ancs_c Apple Notification Service client
// * @{
// * @ingroup ble_sdk_srv
// * @brief Apple Notification Center Service Client Module.
// *
// * @details Disclaimer: This client implementation of the Apple Notification Center Service can
// *          and will be changed at any time by Nordic Semiconductor ASA.
// *
// * Server implementations such as the ones found in iOS can be changed at any
// * time by Apple and may cause this client implementation to stop working.
// *
// * This module implements the Apple Notification Center Service (ANCS) client.
// * This client can be used as a Notification Consumer (NC) that receives data
// * notifications from a Notification Provider (NP). The NP is typically an iOS
// * device acting as a server. For terminology and up-to-date specs, see
// * http://developer.apple.com.
// *
// * The term "notification" is used in two different meanings:
// * - An <i>iOS notification</i> is the data received from the Notification Provider.
// * - A <i>GATTC notification</i> is a way to transfer data with <i>Bluetooth</i> Smart.
// * In this module, we receive iOS notifications using GATTC notifications.
// * We use the full term (iOS notification or GATTC notification) where required.
// *
// * @note The application must propagate BLE stack events to this module
// *       by calling ble_ancs_c_on_ble_evt() from the @ref softdevice_handler callback.
// */
//#ifndef BLE_ANCS_C_H__
//#define BLE_ANCS_C_H__
//
//#ifndef _CLING_PC_SIMULATION_
//#include "ble_types.h"
//#include "ble_srv_common.h"
//#include "device_manager.h"
//
//#define BLE_ANCS_NOTIF_EVT_ID_INDEX       0         /**< Index of the Event ID field when parsing notifications. */
//#define BLE_ANCS_NOTIF_FLAGS_INDEX        1         /**< Index of the Flags field when parsing notifications. */
//#define BLE_ANCS_NOTIF_CATEGORY_ID_INDEX  2         /**< Index of the Category ID field when parsing notifications. */
//#define BLE_ANCS_NOTIF_CATEGORY_CNT_INDEX 3         /**< Index of the Category Count field when parsing notifications. */
//#define BLE_ANCS_NOTIF_NOTIF_UID          4         /**< Index of the Notification UID field when patsin notifications. */
//
//#define START_HANDLE_DISCOVER             0x0001    /**< Value of start handle during discovery. */
//
//#define WRITE_MESSAGE_LENGTH              13        /**< Length of the write message for CCCD/control point. */
//#define BLE_CCCD_NOTIFY_BIT_MASK          0x0001    /**< Enable notification bit. */
//
//#define BLE_ANCS_NB_OF_CATEGORY_ID        12        /**< Number of iOS notification categories: Other, Incoming Call, Missed Call, Voice Mail, Social, Schedule, Email, News, Health And Fitness, Business And Finance, Location, Entertainment. */
//#define BLE_ANCS_NB_OF_EVT_ID             3         /**< Number of iOS notification events: Added, Modified, Removed.*/
//
///** @brief Length of the iOS notification data.
// *
// * @details 8 bytes:
// * Event ID |Event flags |Category ID |Category count|Notification UID
// * ---------|------------|------------|--------------|----------------
// * 1 byte   | 1 byte     | 1 byte     | 1 byte       | 4 bytes
// */
//#define BLE_ANCS_NOTIFICATION_DATA_LENGTH   8
//
//#define ANCS_UUID_CHAR_CONTROL_POINT        0xD8F3  /**< 16-bit control point UUID. */
//#define ANCS_UUID_CHAR_DATA_SOURCE          0xC6E9  /**< 16-bit data source UUID. */
//#define ANCS_UUID_CHAR_NOTIFICATION_SOURCE  0x120D  /**< 16-bit notification source UUID. */
//
//#define BLE_ANCS_EVENT_FLAG_SILENT          0       /**< 0b.......1 Silent: First (LSB) bit is set. All flags can be active at the same time.*/
//#define BLE_ANCS_EVENT_FLAG_IMPORTANT       1       /**< 0b......1. Important: Second (LSB) bit is set. All flags can be active at the same time.*/
//#define BLE_ANCS_EVENT_FLAG_PREEXISTING     2       /**< 0b.....1.. Pre-existing: Third (LSB) bit is set. All flags can be active at the same time.*/
//#define BLE_ANCS_EVENT_FLAG_POSITIVE_ACTION 3       /**< 0b....1... Positive action: Fourth (LSB) bit is set. All flags can be active at the same time.*/
//#define BLE_ANCS_EVENT_FLAG_NEGATIVE_ACTION 4       /**< 0b...1.... Negative action: Fifth (LSB) bit is set. All flags can be active at the same time. */
//
//
///**@brief Event IDs for iOS notifications. */
//typedef enum
//{
//	BLE_ANCS_EVENT_ID_NOTIFICATION_ADDED,     /**< The iOS notification was added. */
//	BLE_ANCS_EVENT_ID_NOTIFICATION_MODIFIED,  /**< The iOS notification was modified. */
//	BLE_ANCS_EVENT_ID_NOTIFICATION_REMOVED    /**< The iOS notification was removed. */
//} ble_ancs_evt_id_t;
//
//
///**@brief Control point command IDs that the Notification Consumer can send to the Notification Provider. */
//typedef enum
//{
//	BLE_ANCS_COMMAND_ID_GET_NOTIF_ATTRIBUTES,      /**< Requests attributes to be sent from the NP to the NC for a given notification. */
//	BLE_ANCS_COMMAND_ID_GET_APP_ATTRIBUTES,        /**< Requests attributes to be sent from the NP to the NC for a given iOS App. */
//	BLE_ANCS_COMMAND_ID_GET_PERFORM_NOTIF_ACTION,  /**< Requests an action to be performed on a given notification, for example dismiss an alarm. */
//} ble_ancs_c_command_id_values_t;
//
//
///**@brief IDs for iOS notification attributes. */
//typedef enum
//{
//	BLE_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER,         /**< Identifies that the attribute data is of an "App Identifier" type. */
//	BLE_ANCS_NOTIF_ATTR_ID_TITLE,                  /**< Identifies that the attribute data is a "Title". */
//	BLE_ANCS_NOTIF_ATTR_ID_SUBTITLE,               /**< Identifies that the attribute data is a "Subtitle". */
//	BLE_ANCS_NOTIF_ATTR_ID_MESSAGE,                /**< Identifies that the attribute data is a "Message". */
//	BLE_ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE,           /**< Identifies that the attribute data is a "Message Size". */
//	BLE_ANCS_NOTIF_ATTR_ID_DATE,                   /**< Identifies that the attribute data is a "Date". */
//	BLE_ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL,  /**< The notification has a "Positive action" that can be executed associated with it. */
//	BLE_ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL,  /**< The notification has a "Negative action" that can be executed associated with it. */
//} ble_ancs_notif_attr_id_t;
//
//
///**@brief Flags for iOS notifications. */
//typedef struct
//{
//	uint8_t silent          : 1;  /**< If this flag is set, the notification has a low priority. */
//	uint8_t important       : 1;  /**< If this flag is set, the notification has a high priority. */
//	uint8_t pre_existing    : 1;  /**< If this flag is set, the notification is pre-existing. */
//	uint8_t positive_action : 1;  /**< If this flag is set, the notification has a positive action that can be taken. */
//	uint8_t negative_action : 1;  /**< If this flag is set, the notification has a negative action that can be taken. */
//} ble_ancs_c_notif_flags_t;
//
//
///**@brief iOS notification structure. */
//typedef struct
//{
//	ble_ancs_evt_id_t     evt_id;          /**< Whether the notification was added, removed, or modified. */
//	uint8_t               evt_flags;       /**< Bitmask to signal if a special condition applies to the notification, for example, "Silent" or "Important". */
//	uint8_t               category_id;     /**< Classification of the notification type, for example, email or location. */
//	uint8_t               category_count;  /**< Current number of active notifications for this category ID. */
//	uint32_t              notif_uid;       /**< Notification UID. */
//} ble_ancs_notif_t;
//
//
///**@brief iOS notification attribute structure for incomming attributes. */
//typedef struct
//{
//	uint32_t                   uid;    /**< UID of the notification that the attribute belongs to.*/
//	uint16_t                   len;    /**< Length of the received attribute data. */
//	ble_ancs_notif_attr_id_t   id;     /**< Classification of the attribute type, for example, title or date. */
//} ble_ancs_notif_attr_t;
//
//
///**@brief iOS notification structure, which contains various status information for the client. */
//typedef struct
//{
//	ble_srv_error_handler_t  error_handler;   /**< Function to be called in case of an error. */
//	uint16_t                 conn_handle;     /**< Handle of the current connection (as provided by the BLE stack; BLE_CONN_HANDLE_INVALID if not in a connection). */
//	uint8_t                  central_handle;  /**< Handle of the currently connected peer (if we have a bond in the Device Manager). */
//	uint8_t                  service_handle;  /**< Handle of the service in the database to use for this instance. */
//} ble_ancs_c_t;
//
//
///**@brief Apple Notification Center Service UUIDs. */
//extern const ble_uuid128_t ble_ancs_base_uuid128;     /**< Service UUID. */
//extern const ble_uuid128_t ble_ancs_cp_base_uuid128;  /**< Control point UUID. */
//extern const ble_uuid128_t ble_ancs_ns_base_uuid128;  /**< Notification source UUID. */
//extern const ble_uuid128_t ble_ancs_ds_base_uuid128;  /**< Data source UUID. */
//
//
//
///**@brief Structure used for holding the characteristic found during the discovery process.
// */
//typedef struct
//{
//	ble_uuid_t            uuid;          /**< UUID identifying the characteristic. */
//	ble_gatt_char_props_t properties;    /**< Properties for the characteristic. */
//	uint16_t              handle_decl;   /**< Characteristic Declaration Handle for the characteristic. */
//	uint16_t              handle_value;  /**< Value Handle for the value provided in the characteristic. */
//	uint16_t              handle_cccd;   /**< CCCD Handle value for the characteristic. */
//} ble_ancs_c_characteristic_t;
//
//
///**@brief Structure used for holding the Apple Notification Center Service found during the discovery process.
// */
//typedef struct
//{
//	uint8_t                     handle;         /**< Handle of Apple Notification Center Service, which identifies to which peer this discovered service belongs. */
//	ble_gattc_service_t         service;        /**< The GATT Service holding the discovered Apple Notification Center Service. */
//	ble_ancs_c_characteristic_t control_point;  /**< Control Point Characteristic for the service. Allows interaction with the peer. */
//	ble_ancs_c_characteristic_t notif_source;   /**< Characteristic that keeps track of arrival, modification, and removal of notifications. */
//	ble_ancs_c_characteristic_t data_source;    /**< Characteristic where attribute data for the notifications is received from peer. */
//} ble_ancs_c_service_t;
//
//
///**@brief ANCS request types.
// */
//typedef enum
//{
//	ANCS_READ_REQ = 1,  /**< Type identifying that this tx_message is a read request. */
//	ANCS_WRITE_REQ      /**< Type identifying that this tx_message is a write request. */
//} ancs_tx_request_t;
//
///**@brief Structure for writing a message to the central, i.e. Control Point or CCCD.
// */
//typedef struct
//{
//	uint8_t                  gattc_value[WRITE_MESSAGE_LENGTH]; /**< The message to write. */
//	ble_gattc_write_params_t gattc_params;                      /**< GATTC parameters for this message. */
//} ancs_write_params_t;
//
//
///**@brief Structure for holding data to be transmitted to the connected master.
// */
//typedef struct
//{
//	uint16_t          conn_handle;  /**< Connection handle to be used when transmitting this message. */
//	ancs_tx_request_t type;         /**< Type of this message, i.e. read or write message. */
//	union
//	{
//			uint16_t       read_handle; /**< Read request message. */
//			ancs_write_params_t write_req;   /**< Write request message. */
//	} req;
//} ancs_tx_message_t;
//
//#endif
//
//#endif // BLE_ANCS_C_H__
//
///** @} */
//
