/***************************************************************************//**
 * File btle.h
 * 
 * Description: BLUETOOTH low energy header
 *
 *
 ******************************************************************************/

#ifndef __BTLE_HEADER__
#define __BTLE_HEADER__

#include "standards.h"

#define _BLE_ENABLED_
#include <string.h>
#define SEC_PARAM_TIMEOUT                    30                                         /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                       1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                         /**< Maximum encryption key size. */

typedef enum {
	AES_REQ_NONE,
	AES_REQ_PENDING,
	AES_REQ_RESPONDED,
	AES_REQ_TIMEOUT,
	AES_REQ_ERROR
} BTLE_AES_STATE_ENUM_LIST;

typedef enum {
	BTLE_SEND_PACKET_ACK,
	BTLE_SEND_PACKET_NACK_INVALID_FORMAT,
	BTLE_SEND_PACKET_NACK_INVALID_REG,
	BTLE_SEND_PACKET_NACK_INVALID_LEN,
	BTLE_CONNECTION_LOSS,
	BTLE_SEND_PACKET_TIMEOUT,
	BTLE_SEND_PACKET_NC_UNSOLICITED,
}BTLE_SEND_PACKET_RESPONSE;

typedef enum {
	BTLE_STATE_IDLE,
	BTLE_STATE_DISCONNECTED,
	BTLE_STATE_DISCONNECTING,
	BTLE_STATE_CONNECTING,
	BTLE_STATE_CONNECTED,
	BTLE_STATE_ADVERTISING
} BTLE_STATE_ENUM_LIST;

typedef enum {
	BTLE_SEND_NONE,
	BTLE_SEND_RDA_THREAD,
	BTLE_SEND_REG_THREAD
}BTLE_SEND_DATA_ENUM_LIST;

enum {
	BTLE_DISCONN_REASON_NONE,
	BTLE_DISCONN_REASON_REBOOT,
	BTLE_DISCONN_REASON_FACTORY_RESET,
	BTLE_DISCONN_REASON_FAST_CONN,
	BTLE_DISCONN_REASON_CP_DISCONN,
	BTLE_DISCONN_REASON_GATT_TIMEOUT,
	BTLE_DISCONN_REASON_USER_SLEEP,
	BTLE_DISCONN_REASON_SYSTEM_SHUTDOWN,
	BTLE_DISCONN_REASON_ANCS_DISC_FAIL,	
	BTLE_DISCONN_REASON_ANCS_DELETE_BOND	
};

#define BLE_DISCONN_EVT_FACTORY_RESET   0x0001
#define BLE_DISCONN_EVT_FAST_CONNECT		0x0002
#define BLE_DISCONN_EVT_POWER_OFF       0x0004
#define BLE_DISCONN_EVT_DISK_FORMAT			0x0008
#define BLE_DISCONN_EVT_SYS_BACKUP			0x0010
#define BLE_DISCONN_EVT_BONDMGR_ERROR		0x0020
#define BLE_DISCONN_EVT_UTC_MIDNIGHT		0x0040
#define BLE_DISCONN_EVT_REBOOT					0x0080

#define BLE_STREAMING_SECOND_MAX        10

/**@brief Advertisement states. */
typedef enum
{
    BLE_FAST_ADV,                                                                           /**< Fast advertising running. */
    BLE_SLOW_ADV,                                                                           /**< Slow advertising running. */
    BLE_ADV_SLEEP                                                                               /**< Go to system-off. */
} BTLE_ADVERTISING_MODE;

typedef struct tagBLE_CTX {
	I32U packet_received_ts;
	I32U streaming_ts;
	I32U streaming_file_addr;
	I16U streaming_minute_scratch_amount;
	I16U streaming_minute_scratch_index;
	I16U minute_file_entry_index;
	I16U service_handle;                                       // Handle of BLE Service (as provided by the BLE stack).
	I16U conn_handle;
	I16U disconnect_evt;
	I8U streaming_minute_file_amount;
	I8U streaming_minute_file_index;
	I8U streaming_second_count;
	I8U tx_buf_available;
	BTLE_STATE_ENUM_LIST	btle_State;
	BTLE_ADVERTISING_MODE adv_mode;
	BOOLEAN b_conn_params_updated;
} BLE_CTX;

void btle_init(void);
I32U BTLE_services_init(void);
void BTLE_aes_encrypt(I8U *key, I8U *in, I8U *out);
void BTLE_aes_decrypt(I8U *key, I8U *in, I8U *out);
BOOLEAN BTLE_tx_buffer_available(void);
void BTLE_disconnect(I8U reason);
BOOLEAN BTLE_Send_Packet(I8U *data, I8U len);
void BTLE_state_machine_update(void);
BOOLEAN BTLE_get_radio_software_version(I8U *radio_sw_ver);
BOOLEAN BTLE_is_connected(void);
BOOLEAN BTLE_is_advertising(void);
BOOLEAN BTLE_is_idle(void);
void BTLE_execute_adv(BOOLEAN b_fast);
BOOLEAN BTLE_is_streaming_enabled(void);
void BTLE_reset_streaming(void);
void BTLE_update_streaming_minute_entries(void);
BOOLEAN BTLE_streaming_authorized(void);
uint32_t get_flash_minute_activity_offset(void);
#endif // __BTLE_HEADER__
/** @} */
