/***************************************************************************//**
 * File auth.h
 * 
 * Description: Pairing and authentication header
 *
 *
 ******************************************************************************/

#ifndef __LINK_HEADER__
#define __LINK_HEADER__

#define FILE_WAIT_TIMEOUT_IN_MS ((uint16_t)60000)
#define DEBUG_MILLISECONDS_BEFORE_AUTO_AUTH ((uint32_t)90000)
#define GENERATING_BLOB_TIMEOUT_IN_MS ((uint16_t)1400)
#define CHECKING_BLOB_TIMEOUT_IN_MS ((uint16_t)1400)
#define FLASH1_BASE_ADDRESS ((uint8_t *)0x4400)
#define FLASH1_END_ADDRESS ((uint16_t *)0xFF80)
#define TWO_BYTE_CHUNKS_IN_FLASH1 (32160)
#define FLASH2_BASE_ADDRESS ((uint8_t *)0x10000)
#define FLASH2_END_ADDRESS ((uint16_t *)0x1FFF0)
#define TWO_BYTE_CHUNKS_IN_FLASH2 (32767)

// independent defines first
#define MAX_ENCRYPT_DECRYPT_STRING_SIZE 100
#define DEVICE_ID_TOTAL_CHARACTERS 20
//#define LINK_VERSION_STRING ",1.1,"
#define LINK_VERSION_STRING_LENGTH 5
#define ENCRYPT_DECRYPT_BLOCK_SIZE 16
#define CHARACTERS_IN_FIRST_ENCRYPT_STRING 10

#define LINK_TOKEN 0x7fa5
#define FROM_SERVICE_FILE_SIZE 98 /* 3*32 (encryption blocks) + 2*1 (":") */

// derived defines next
#define ENCRYPT_STRING_SIZE (ENCRYPT_DECRYPT_BLOCK_SIZE << 1)
#define FIRST_AES_OUT_TEXT_POSITION (DEVICE_ID_TOTAL_CHARACTERS + LINK_VERSION_STRING_LENGTH)
#define VERSION_STRING_START_POSITION DEVICE_ID_TOTAL_CHARACTERS
#define CHARACTERS_IN_SECOND_ENCRYPT_STRING (DEVICE_ID_TOTAL_CHARACTERS - CHARACTERS_IN_FIRST_ENCRYPT_STRING)

#define SECOND_AES_OUT_TEXT_POSITION (FIRST_AES_OUT_TEXT_POSITION + (ENCRYPT_STRING_SIZE) + 1)
#define ENCRYPT_FILE_SIZE (20+1+3+1+32+1+32)  /* clingId + ":" + authVersion + ":" + encryptedToken1 + ":" + encryptedToken2 */

typedef enum {
	 LINK_S_IDLE = 0,		// default state after factory reset or reset for new user from ios
	 LINK_S_START_AUTHENTICATION, 		// state to give time from kicking it off to using the aes block
	 LINK_S_GENERATE_ENCRYPT_BLOCK1,	// generate the first encryption block
	 LINK_S_GENERATE_ENCRYPT_BLOCK2, 	// Generating the file to send to the service
	 LINK_S_WRITE_ENCRYPT_FILE,
	 LINK_S_WAIT_FOR_CHECK_FLAG,
	 LINK_S_WAITING_ON_RETURN_FILE,	// Waiting for return file from service
	 LINK_S_DECRYPT_BLOB1,
	 LINK_S_DECRYPT_BLOB2,
	 LINK_S_DECRYPT_BLOB3,
	 LINK_S_CHECK_FILE,				// Checking the contents of the return file
	 LINK_S_GENERATE_ERROR_FILE,		// generate error file
	 LINK_S_LINK_FAILED, 				// State used for displaying a message
	 LINK_S_WRITE_LINK_INFO, 			// Write the auth information to info C
	 LINK_S_AUTHORIZED,					// Successful authorization
	 LINK_S_WRITING_LINK_INFO,
	 LINK_S_REBOOT_DEVICE,
 } LINK_STATE_TYPE;

 typedef enum {
 	 NO_ERROR = 0,				// No error
 	 ERROR_ENCRYPT_BLOB1 = 1,		// Error encrypting first blob; only found if debugging
 	 ERROR_ENCRYPT_BLOB2 = 2, 		// Error encrypting second blob; only found if debugging
 	 ERROR_FILE_NOT_FOUND = 3,		// File not found when told to check for it
 	 ERROR_FILE_LENGTH = 4,			// File length doesn't match requirement
 	 ERROR_FILE_CONSTRUCTION = 5,	// Error with placement of colons
 	 ERROR_DECRYPT_BLOB1 = 6,		// Error decrypting first blob; only found if debugging
 	 ERROR_DECRYPT_BLOB2 = 7,		// Error decrypting second blob; only found if debugging
 	 ERROR_DECRYPT_BLOB3 = 8,		// Error decrypting third blob; only found if debugging
 	 ERROR_ID_MISMATCH = 9,			// Error with return ID not matching this device ID
 	 ERROR_BAD_DEVICE_ID = 10, 		// Error with device ID (not a number)
 	 ERROR_TIMEOUT_GENERATING_BLOB = 11, 	// Timeout generating the blob
 	 ERROR_TIMEOUT_CHECKING_BLOB = 12,		// Timeout checking the blob
 	 ERROR_TIMEOUT_FILE_RETURN = 13, 	// Timeout on the return file.
 	 ERROR_WRITE_FILE_MUTEX_HOLD = 14, // Error writing the to_service file due to mutex blocked
 	 ERROR_READ_FILE_MUTEX_HOLD = 15,	// Error reading the from_service file due to mutex blocked
 	 ERROR_WRITE_RX_BLOB_MUTEX_HOLD = 16,	// Error writing the file for the received blob due to mutex blocked
 	 ERROR_HIT_DEFAULT_STATE = 17,			// Error where we hit the default state in the FSM (which we should never)
	 ERROR_DECRYPT_NOT_SUPPORTED = 18
  } ERROR_CODE_TYPE;

 enum {
	 LINK_COMM_IDLE,
	 LINK_COMM_A1_DONE,
	 LINK_COMM_A2_DONE,
	 LINK_COMM_FA_DONE,
	 LINK_COMM_ERROR,
 };

typedef struct tagPAIRING_CTX {
	uint16_t authToken;
	uint16_t crc;
	// Local message id
	uint32_t userID;
	uint32_t epoch; /* registration time */
} PAIRING_CTX;

typedef struct tagLINK_CTX {
	PAIRING_CTX pairing;
	uint32_t userID;
	uint32_t link_ts;
	BOOLEAN b_authorizing;
	uint8_t trigger_cmd;
	uint8_t comm_state;
	uint8_t error_code;
	LINK_STATE_TYPE	auth_state;
} LINK_CTX;

void LINK_init(void);
int LINK_state_machine_update(void) ;
bool LINK_is_authorized(void);
BOOLEAN LINK_is_authorizing(void);
void LINK_start_authorizing(BOOLEAN b_state);
void LINK_deauthorize(void);
// the following routine stores the flash crc in the cling.link.crc variable
void LINK_GenerateFlashCRC(void);

#endif // __LINK_HEADER__
/** @} */
