/***************************************************************************//**
 * File cp.h
 * 
 * Description: Generic communication protocol header
 *
 *
 ******************************************************************************/

#ifndef __CP_API_HEADER__
#define __CP_API_HEADER__

#include "fs_memcfg.h"
#include "fs_file.h"

#define _NON_BLOCK_CP_

#define CP_UUID_PKT_SIZE	22
#define CP_PACKET_SIZE		20
#define CP_PAYLOAD_SIZE	16

#define UUID_ADV_SERVICE 0xffe0

#define UUID_TX_SP      0xffe1
#define UUID_TX_START   0xffe2
#define UUID_TX_MIDDLE  0xffe3
#define UUID_TX_END     0xffe4
#define UUID_RX_SP      0xffe5
#define UUID_RX_START   0xffe6
#define UUID_RX_MIDDLE  0xffe7
#define UUID_RX_END     0xffe8


// Activity streaming thresholds
#define ACTIVITY_STREAMING_RESET_TIME 60000 // 1 Minute time out for streaming

#define MSG_NFLASH_STORAGE_START ((I32U)0x78000)

typedef enum {
	CP_STORAGE_MEDIA_NFLASH,
	CP_STORAGE_MEDIA_RAM,
	CP_STORAGE_MEDIA_FILE_SYSTEM,
} CP_STORAGE_MEDIA_TYPE;

enum {
	GCIP_ACK_NORMAL,
	GCIP_ACK_BUSYEVT,
	GCIP_ACK_RETRANSMIT,
	GCIP_ACK_RETRANSMITLAST,
  GCIP_ACK_MAX = GCIP_ACK_RETRANSMITLAST
};

//Radio communication TX states 
enum {
	CP_MCU_STAT_IDLE,
	CP_MCU_STAT_TX_SENDING,
	CP_MCU_STAT_TX_COMPLETE,
	CP_MCU_STAT_TX_CANCEL,
	CP_MCU_STAT_RX_RECEIVING,
	CP_MCU_STAT_RX_COMPLETE,
	CP_MCU_STAT_INVALID,
};

// -----------------------------------------
// Message OPCODE ENUM list

enum {
	CP_MSG_TYPE_REGISTER_READ = 0,         // 0
	CP_MSG_TYPE_REGISTER_WRITE,
	CP_MSG_TYPE_FILE_LOAD_LIST,
	CP_MSG_TYPE_FILE_READ,
	CP_MSG_TYPE_FILE_WRITE,
	CP_MSG_TYPE_FILE_DELETE,               // 5
	CP_MSG_TYPE_LOAD_DEVICE_INFO,
	CP_MSG_TYPE_STREAMING_SECOND,
	CP_MSG_TYPE_STREAMING_MINUTE,
	CP_MSG_TYPE_STREAMING_MULTI_MINUTES,
	CP_MSG_TYPE_ACK,                      // 10
	CP_MSG_TYPE_AUTHENTICATIOIN, 
	CP_MSG_TYPE_REBOOT,
	CP_MSG_TYPE_START_OTA,
	CP_MSG_TYPE_WEATHER,
	CP_MSG_TYPE_USER_REMINDER,             // 15
	CP_MSG_TYPE_SIMULATION_CONFIG,
	CP_MSG_TYPE_BLE_DISCONNECT,
	CP_MSG_TYPE_LOAD_DAILY_ACTIVITY,
	CP_MSG_TYPE_SET_ANCS,
	CP_MSG_TYPE_DEBUG_MSG,                 // 20
	CP_MSG_TYPE_DEVICE_SETTING,
	CP_MSG_TYPE_STREAMING_DAILY,
	CP_MSG_TYPE_ANDROID_NOTIFY,
	CP_MSG_TYPE_SOS_MESSAGE,               
	CP_MSG_TYPE_SET_LANGUAGE,              // 25
	CP_MSG_TYPE_SET_USER_PROFILE, 
	CP_MSG_TYPE_PHONE_FINDER_MESSAGE,
	CP_MSG_TYPE_DAILY_ALARM,
};

typedef struct tagCP_PACKET {
	I8U uuid[2];
	I8U id;
	I8U len[3];
	I8U data[16];
} CP_PACKET;

enum {
	CP_TX_IDLE,
	CP_TX_PACKET_PENDING_SEND,
	CP_TX_PACKET_PENDING_ACK,
};

enum {
	HOST_TYPE_NONE,
	HOST_TYPE_IOS,
	HOST_TYPE_ANDROID
};

typedef struct tagCP_TX_STREAMING_CTX {
	I32U packet_tx_time;
	BOOLEAN pending;
	BOOLEAN packet_need_ack;
	BOOLEAN flag_entry_read;
	BOOLEAN packet_is_acked;
	I16U minutes_repeat_num;
	I8U pkt[CP_UUID_PKT_SIZE];
} CP_TX_STREAMING_CTX;

typedef struct tagCP_TX_CTX {
	BYTE state;  // A packet is sent
	I8U msg_type;
	I8U msg_id;
	CP_PACKET pkt;
	I8U msg[FAT_SECTOR_SIZE];
	I32U msg_pos;
	I8U msg_file_id;
	I8U msg_file_total;
	I32U msg_len;
	I32U msg_checksum;
	I8U msg_filling_offset;
	I8U msg_fetching_offset;
	I32U ack_ts;
	I8U retrans_cnt;
} CP_TX_CTX;

typedef struct tagCP_RX_CTX {
	I32U msg_len;
	I32U msg_file_len;
	I16U UUID;
	I8U msg_type;
	I8U msg_file_name_len;
	I8U rcving_offset;
	CLING_FILE f;
	BOOLEAN file_wr_active;
	I8U msg[FAT_SECTOR_SIZE];
	BOOLEAN b_reload_profile;
	BOOLEAN b_ota_start;
} CP_RX_CTX;

typedef struct tagCP_PENDING_CTX {
	I32U pending_len;
	I32U processed_len;
	BOOLEAN b_touched;
	I8U task_id;
	I8U msg[FAT_SECTOR_SIZE];
} CP_PENDING_CTX;

typedef struct tagCP_CTX {
	// Global state
	I8U state;
	
	// Local message id
	I8U msg_id;

	// TX context
	CP_TX_CTX tx;

	// Streaming ontext
	CP_TX_STREAMING_CTX streaming;

	// RX context
	CP_RX_CTX rx;

	// Pending message
	CP_PENDING_CTX pending;
	
	I8U pkt_buff[22];
	BOOLEAN b_new_pkt;
	
	// Host type
	I8U host_type;

} CP_CTX;

void CP_API_received_pkt(BYTE *data);
void CP_state_machine_update(void);
void CP_create_streaming_daily_msg( void );
BOOLEAN CP_create_streaming_minute_msg(I32U space_size);
BOOLEAN CP_create_streaming_file_minute_msg(I32U space_size);
void CP_create_register_rd_msg(void);
void CP_create_auth_stat_msg(void);
void CP_create_sos_msg(void);
void CP_create_phone_finder_msg(void);

#endif // __CP_API_HEADER__
/** @} */
