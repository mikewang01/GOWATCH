/***************************************************************************/
/**
 * File: cp.c
 *
 * Description: Generic communication protocol.
 *
 * Created on Jan 3, 2014
 *
 ******************************************************************************/

#include <string.h>

#include "main.h"
#include "fs_root.h"
#include "ble_gap_mgr.h"
//#include "suota.h"
//#define _JACOB_TESTING_
#include "ota_hal.h"
#include "string.h"
static CLASS(HalOta) *ota_p = NULL;
static uint8_t is_app_package = false;
static uint8_t curent_sector_num   __attribute__((unused)) = 0;
static uint32_t dialog_upgrade_offset = 0;

#define  FLASH_BLOCK_SIZE 4096
#define _CLING_DEV_RF_MODEL_
#define  ONE_MORE_LOOP_FOR_FSM()  do{\
         OS_TASK_NOTIFY(OS_GET_CURRENT_TASK(), BLE_APP_NOTIFY_FSM_MASK, eSetBits);\
}while(0)

static void _sync_time_proc(I8U *data)
{
        DAY_TRACKING_CTX stored_day_total;
        BOOLEAN b_activity_consistency = TRUE; //

        // Record time difference
        cling.time.time_zone = data[0];

        cling.time.time_since_1970 = data[1];
        cling.time.time_since_1970 <<= 8;
        cling.time.time_since_1970 |= data[2];
        cling.time.time_since_1970 <<= 8;
        cling.time.time_since_1970 |= data[3];
        cling.time.time_since_1970 <<= 8;
        cling.time.time_since_1970 |= data[4];

        RTC_get_local_clock(cling.time.time_since_1970, &cling.time.local);

        // Reset minute streaming amount;
        BTLE_reset_streaming();

        // Make sure the minute file has correct dialog_upgrade_offset
#ifdef _ENABLE_UART_
        {
                I32U minute_flash_offset;
                // Make sure the minute file has correct dialog_upgrade_offset
                minute_flash_offset = TRACKING_get_daily_total(&stored_day_total);

                N_SPRINTF("[CP] time sync !! - %d (minute), %d, %d", minute_flash_offset, cling.time.time_zone, cling.time.time_since_1970);
        }
#else
        TRACKING_get_daily_total(&stored_day_total);
#endif
        // Check if we have consistent daily stored activity
        if (stored_day_total.calories != cling.activity.day_stored.calories) {
                b_activity_consistency = FALSE;
        }
        else if (stored_day_total.distance != cling.activity.day_stored.distance) {
                b_activity_consistency = FALSE;
        }
        else if (stored_day_total.running != cling.activity.day_stored.running) {
                b_activity_consistency = FALSE;
        }
        else if (stored_day_total.walking != cling.activity.day_stored.walking) {
                b_activity_consistency = FALSE;
        }

        if (b_activity_consistency) {
                N_SPRINTF("-- CP Tracking OK (%d): %d, %d, %d, %d",
                        cling.activity.tracking_flash_offset,
                        stored_day_total.calories,
                        stored_day_total.distance,
                        stored_day_total.running,
                        stored_day_total.walking);
                return;
        }

        // If not, take the flash as new startin point
        memcpy(&cling.activity.day, &stored_day_total, sizeof(DAY_TRACKING_CTX));
        memcpy(&cling.activity.day_stored, &stored_day_total, sizeof(DAY_TRACKING_CTX));

        // Reset day_stored structure too ...
        N_SPRINTF("-- Tracking reset (%d): %d, %d, %d, %d",
                cling.activity.tracking_flash_offset,
                stored_day_total.calories,
                stored_day_total.distance,
                stored_day_total.running,
                stored_day_total.walking);
}

static void _create_one_pkt_from_msg()
{
        CP_CTX *g = &cling.gcp;
        CP_TX_CTX *t = &g->tx;

        t->pkt.id = t->msg_id;

        // Get message length
        t->pkt.len[0] = (t->msg_pos >> 16) & 0xff;
        t->pkt.len[1] = (t->msg_pos >> 8) & 0xff;
        t->pkt.len[2] = t->msg_pos & 0xff;

        memcpy(t->pkt.data, t->msg + t->msg_fetching_offset, CP_PAYLOAD_SIZE);

        t->msg_pos += CP_PAYLOAD_SIZE;
        t->msg_fetching_offset += CP_PAYLOAD_SIZE;

        N_SPRINTF("_create_one_pkt_from_msg enter %d %d", t->msg_pos, t->msg_len);
        // check whether this is the last packet to send ...
        if (t->msg_pos >= t->msg_len) {
                t->pkt.uuid[0] = (UUID_TX_END >> 8) & 0xff;
                t->pkt.uuid[1] = (UUID_TX_END & 0xff);
                g->state = CP_MCU_STAT_TX_COMPLETE;
                N_SPRINTF("[CP] This is the last package");
                // Relase MUTEX as we get to the end of a file list
                SYSTEM_release_mutex(MUTEX_IOS_LOCK_VALUE);
        }
        else {
                N_SPRINTF("[CP] this  is in the midle of package");
                t->pkt.uuid[0] = (UUID_TX_MIDDLE >> 8) & 0xff;
                t->pkt.uuid[1] = (UUID_TX_MIDDLE & 0xff);
                ONE_MORE_LOOP_FOR_FSM()
                ;
        }
        N_SPRINTF("_create_one_pkt_from_msg leave");
}

static BOOLEAN _tx_streaming()
{
        CP_TX_STREAMING_CTX *s = &cling.gcp.streaming;
        I8U sent = BTLE_Send_Packet(s->pkt, CP_UUID_PKT_SIZE);

        if (sent)
                s->pending = FALSE;

        return sent;
}

static BOOLEAN _tx_process(I32U curr_ts)
{
        CP_CTX *g = &cling.gcp;
        CP_TX_CTX *t = &g->tx;
#ifdef _BLE_ENABLED_
        I8U sent = BTLE_Send_Packet((I8U *)(&t->pkt), CP_UUID_PKT_SIZE);

        if (sent) {

                t->state = CP_TX_IDLE;

                return TRUE;
        }
        else {
                return FALSE;
        }
#endif
        return TRUE;
}

static void _proc_pending_ctrl_wr()
{
#ifdef _BLE_ENABLED_
        CP_PENDING_CTX *p = &cling.gcp.pending;
        I8U idx;
        I8U *reg;

        // Ctrl register is the msg[0];
        idx = p->msg[0];
        reg = p->msg + 1;
        N_SPRINTF("[CP]: REGISTER_MCU_CTL index = 0x%04x", idx);
        cling.system.mcu_reg[REGISTER_MCU_CTL] |= idx;

        if (CTL_TS & idx) {
                _sync_time_proc(reg + 3);
        }
        else if (CTL_IM & idx) {
                cling.ble.streaming_second_count = 0; // Reset streaming second overall count
                ONE_MORE_LOOP_FOR_FSM()
                ;
        }
        else if (CTL_DF & idx) {
                N_SPRINTF("[CP]: System format disk index = 0x%04x", idx);
                // System format disk
                if ((I8U)(cling.link.pairing.crc & 0x00ff)
                        == cling.system.mcu_reg[REGISTER_MCU_PROTECT]) {
                        SYSTEM_format_disk(FALSE);
                        cling.system.mcu_reg[REGISTER_MCU_PROTECT] = 0;
                }
                else {
                        N_SPRINTF("[CP] disk format failed: %02x, 0x%02x", (cling.link.pairing.crc & 0x00ff), cling.system.mcu_reg[REGISTER_MCU_PROTECT]);
                }
        }
        else if (CTL_A0 & idx) {
                N_SPRINTF("[CP]: Authorization starts = 0x%04x", idx);
                // Authorization starts.
                LINK_start_authorizing(TRUE);
        }
        else if (CTL_RFNU & idx) {

                if ((I8U)(cling.link.pairing.crc & 0x00ff)
                        == cling.system.mcu_reg[REGISTER_MCU_PROTECT]) {
                        N_SPRINTF("[CP] factory reset succeed");
                        // When user confirms a factory reset, we switch to reset vector
                        SYSTEM_factory_reset();
                }
                else {
                        N_SPRINTF("[CP] factory reset failed: %02x, 0x%02x", (cling.link.pairing.crc & 0x00ff), cling.system.mcu_reg[REGISTER_MCU_PROTECT]);
                }
        }
#endif
}

static void _create_file_list_msg()
{
        I16U i;
        CP_CTX *g = &cling.gcp;
        CP_TX_CTX *t = &g->tx;

        // Creat a new message
        t->msg_id++;

        // Get overall file amount and message length
        t->msg_file_total = FILE_GetFileNum(&t->msg_len);

        t->pkt.uuid[0] = (UUID_TX_START >> 8) & 0xff;
        t->pkt.uuid[1] = (UUID_TX_START & 0xff);
        t->pkt.id = t->msg_id;

        // Get message length
        t->msg_len += 7;  // 7 bytes: msg type (1 byte) + total files (2 bytes) + checkSum (4 bytes)
        t->pkt.len[0] = (t->msg_len >> 16) & 0xff;
        t->pkt.len[1] = (t->msg_len >> 8) & 0xff;
        t->pkt.len[2] = t->msg_len & 0xff;

        // Initialize message position
        t->msg_pos = 0;
        t->msg_fetching_offset = 0;
        t->msg_checksum = 0;
        t->msg_file_id = 0;

        // Filling up the message buffer
        t->msg_filling_offset = 0;
        t->msg[t->msg_filling_offset++] = CP_MSG_TYPE_FILE_LOAD_LIST;
        t->msg[t->msg_filling_offset++] = (t->msg_file_total >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = t->msg_file_total & 0xff;

        // Create packet payload
        // Pending message delivery
        t->msg_type = CP_MSG_TYPE_FILE_LOAD_LIST;
        t->state = CP_TX_PACKET_PENDING_SEND;
        g->state = CP_MCU_STAT_TX_SENDING;

        if (!t->msg_file_total) {
                // Add checksum to the end
                for (i = 0; i < t->msg_filling_offset; i++)
                        t->msg_checksum += t->msg[i];
                t->msg[t->msg_filling_offset++] = (t->msg_checksum >> 24) & 0xff;
                t->msg[t->msg_filling_offset++] = (t->msg_checksum >> 16) & 0xff;
                t->msg[t->msg_filling_offset++] = (t->msg_checksum >> 8) & 0xff;
                t->msg[t->msg_filling_offset++] = t->msg_checksum & 0xff;

                memcpy(t->pkt.data, t->msg, t->msg_filling_offset);

                t->msg_fetching_offset += CP_PAYLOAD_SIZE;
                t->msg_pos += CP_PAYLOAD_SIZE;
                // Sending is completed
                g->state = CP_MCU_STAT_TX_COMPLETE;

                // Relase MUTEX as we get to the end of a file list
                SYSTEM_release_mutex(MUTEX_IOS_LOCK_VALUE);
                return;
        }
        else {
                for (i = 0; i < t->msg_file_total; i++) {
                        if (t->msg_filling_offset >= 64) {
                                break;
                        }
                        t->msg_filling_offset = FILE_getFileInfo(i, t->msg, t->msg_filling_offset);
                }

                t->msg_file_id = i;

                // Add checksum to the end
                for (i = 0; i < t->msg_filling_offset; i++)
                        t->msg_checksum += t->msg[i];


                uint8_t *p =  (uint8_t*)&dialog_upgrade_offset;
                for(i= 0; i < sizeof(dialog_upgrade_offset); i++){
                        t->msg_checksum+=p[i];
                }
                N_SPRINTF("app.bin found fw lenth = %d\r\n", dialog_upgrade_offset);

                if (t->msg_file_id == t->msg_file_total) {
                        t->msg[t->msg_filling_offset++] = (t->msg_checksum >> 24) & 0xff;
                        t->msg[t->msg_filling_offset++] = (t->msg_checksum >> 16) & 0xff;
                        t->msg[t->msg_filling_offset++] = (t->msg_checksum >> 8) & 0xff;
                        t->msg[t->msg_filling_offset++] = t->msg_checksum & 0xff;

                        // Relase MUTEX as we get to the end of a file list
                        SYSTEM_release_mutex(MUTEX_IOS_LOCK_VALUE);

                }

                // Create the payload
                memcpy(t->pkt.data, t->msg, CP_PAYLOAD_SIZE);

                t->msg_pos += CP_PAYLOAD_SIZE;
                t->msg_fetching_offset += CP_PAYLOAD_SIZE;
        }
}

static void _create_dev_info_msg()
{
#define FAKE_MAC_ADRESS {0X01, 0X04, 0X05, 0X06, 0X08, 0X0A}
#ifdef _BLE_ENABLED_
        CP_CTX *g = &cling.gcp;
        CP_TX_CTX *t = &g->tx;
        I32U free;
        I16S file_available;
        I8U dev_data[GLOBAL_DEVICE_ID_LEN]   __attribute__((unused));
        I8U key_user_info[32];
        I8U i   __attribute__((unused));
        PAIRING_CTX *pPairing;
        uint8_t mac_addr_t[] = FAKE_MAC_ADRESS;

        // Get free space in current file system
        FAT_Init();

        free = FAT_CheckSpace();
        free += 200 * 1024;
        // Get available files in the CLING
        file_available = N_ROOTDIR - ROOT_ListFiles();

        if (file_available < 0)
                file_available = 0;

        N_SPRINTF("[CP] @@@@ file available: %d, mem: %d", file_available, free);

        // Creat a new message
        t->msg_id++;

        // Get message length
        // 49 bytes:
        //          type: 1 B
        // touch version: 1 B
        //   mcu-hw-info: 1 B
        // battery level: 1 B
        //  mcu-sw-major: 1 B
        //  mcu-sw-minor: 1 B
        //  radio-sw-ver: 1 B
        //    space free: 4 B
        //     file free: 2 B
        //         token: 2 B
        //           crc: 2 B
        //       user id: 4 B
        //         epoch: 4 B
        //        dev id: 20 B
        //   reset count: 4 B
        //  Model Number: 6 B
        // Minute file number: 1 B
        // MAC valid flag: 1 B
        //   MAC address: 6 B
        //
        t->pkt.uuid[0] = (UUID_TX_START >> 8) & 0xff;
        t->pkt.uuid[1] = (UUID_TX_START & 0xff);
        t->pkt.id = t->msg_id;

        // Filling up the message buffer
        t->msg_filling_offset = 0;
        t->msg_pos = 0;
        t->msg_fetching_offset = 0;

        // Type
        t->msg[t->msg_filling_offset++] = CP_MSG_TYPE_LOAD_DEVICE_INFO;
        // Touch panel version
        t->msg[t->msg_filling_offset++] = cling.whoami.touch_ver[2];
        // HW INFO
        t->msg[t->msg_filling_offset++] = 1;
        // BATTERY LEVEL
        t->msg[t->msg_filling_offset++] = cling.system.mcu_reg[REGISTER_MCU_BATTERY];
        // Software version number
        t->msg[t->msg_filling_offset++] = cling.system.mcu_reg[REGISTER_MCU_REVH];  // SW VER (HI)
        t->msg[t->msg_filling_offset++] = cling.system.mcu_reg[REGISTER_MCU_REVL];  // SW VER (LO)
        // Radio S/W version number
        BTLE_get_radio_software_version(t->msg + t->msg_filling_offset);
        t->msg_filling_offset++;
        // File system free space
        t->msg[t->msg_filling_offset++] = (free >> 24) & 0xff;
        t->msg[t->msg_filling_offset++] = (free >> 16) & 0xff;
        t->msg[t->msg_filling_offset++] = (free >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = free & 0xff;
        // Available files in the CLING
        t->msg[t->msg_filling_offset++] = (file_available >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = file_available & 0xff;
        // key user info 16 bytes

        N_SPRINTF("[CP] ++dev info: %d, %d, %d", cling.link.pairing.userID, cling.link.pairing.crc, cling.link.pairing.authToken);

        // System checks dev info after authentication is done, but auth info
        // is yet to be written into flash. So, rather than loading auth from flash
        // we do it directly from auth pairing info, so the app proceed as a successful
        // authentication.
        if (cling.link.pairing.authToken == LINK_TOKEN) {
                memcpy(key_user_info, &cling.link.pairing, sizeof(PAIRING_CTX));
        }
        else {
                FLASH_Read_App(SYSTEM_LINK_SPACE_START, key_user_info, 32);
        }

        pPairing = (PAIRING_CTX *)key_user_info;

        // Token: 2 B
        t->msg[t->msg_filling_offset++] = (pPairing->authToken >> 8) & 0xFF;
        t->msg[t->msg_filling_offset++] = pPairing->authToken & 0xFF;

        // CRC 2B
        t->msg[t->msg_filling_offset++] = (pPairing->crc >> 8) & 0xFF;
        t->msg[t->msg_filling_offset++] = pPairing->crc & 0xFF;
        // USER ID: 4 B
        t->msg[t->msg_filling_offset++] = (pPairing->userID >> 24) & 0xff;
        t->msg[t->msg_filling_offset++] = (pPairing->userID >> 16) & 0xff;
        t->msg[t->msg_filling_offset++] = (pPairing->userID >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = pPairing->userID & 0xff;
        // EPOCH: 4 B
        t->msg[t->msg_filling_offset++] = (pPairing->epoch >> 24) & 0xff;
        t->msg[t->msg_filling_offset++] = (pPairing->epoch >> 16) & 0xff;
        t->msg[t->msg_filling_offset++] = (pPairing->epoch >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = pPairing->epoch & 0xff;

        // Device info 20 bytes
        size_t length = 0;
        CLASS(GapMgr) *gap_mgr = GapMgr_get_instance();
				N_SPRINTF("[cp]gap_mgr->get_dev_id_strd");
        gap_mgr->get_dev_id_str(gap_mgr, &t->msg[t->msg_filling_offset], &length);
//        SYSTEM_get_dev_id(dev_data);
        t->msg_filling_offset += length;
//        for (i = 0; i < GLOBAL_DEVICE_ID_LEN; i++)
//                t->msg[t->msg_filling_offset++] = dev_data[i];

        // Add system reset count
        t->msg[t->msg_filling_offset++] = 0;
        t->msg[t->msg_filling_offset++] = 0;
        t->msg[t->msg_filling_offset++] = (cling.system.reset_count >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = cling.system.reset_count & 0xff;
				
				gap_mgr->get_model_id(gap_mgr, &t->msg[t->msg_filling_offset], &length);
				t->msg_filling_offset += length;

        // Amount of minute streaming files
        t->msg[t->msg_filling_offset++] = FILE_exists_with_prefix((I8U *)"epoch", 5);

        // Valid mac address
//		sd_ble_gap_address_get(&mac_addr_t);
				N_SPRINTF("[cp] get mac adress");
        t->msg[t->msg_filling_offset++] = 0x69;
				gap_mgr->get_model_id(gap_mgr, &t->msg[t->msg_filling_offset], &length);
				t->msg_filling_offset += length;


        t->msg_len = t->msg_filling_offset;

        // Get message length (fixed length: 2 bytes)
        t->pkt.len[0] = 0;
        t->pkt.len[1] = 0;
        t->pkt.len[2] = t->msg_filling_offset;

        // Create packet payload
        // Pending message delivery
        t->msg_type = CP_MSG_TYPE_LOAD_DEVICE_INFO;
        t->state = CP_TX_PACKET_PENDING_SEND;
        g->state = CP_MCU_STAT_TX_SENDING;

        memcpy(&t->pkt.data, t->msg, CP_PAYLOAD_SIZE);

        t->msg_pos += CP_PAYLOAD_SIZE;
        t->msg_fetching_offset += CP_PAYLOAD_SIZE;
#endif
}

static void _create_daily_activity_info_msg()
{
        CP_CTX *g = &cling.gcp;
        CP_TX_CTX *t = &g->tx;
        DAY_STREAMING_CTX day_streaming;

        TRACKING_get_daily_streaming_stat(&day_streaming);
        TRACKING_get_daily_streaming_sleep(&day_streaming);

        // Creat a new message
        t->msg_id++;

        // Get message length
        //                 type: 1 B
        //                steps: 4 B
        //             distance: 4 B
        //     calories(active): 2 B
        //       calories(idle): 2 B
        //         wakeup times: 1 B
        //  sleep_light_seconds: 2 B
        //  sleep_sound_seconds: 2 B
        //    sleep_rem_seconds: 2 B
        //   average heart rate: 1 B
        //  average temperature: 2 B
        //             uv index: 1B
        //
        t->pkt.uuid[0] = (UUID_TX_START >> 8) & 0xff;
        t->pkt.uuid[1] = (UUID_TX_START & 0xff);
        t->pkt.id = t->msg_id;

        // Filling up the message buffer
        t->msg_filling_offset = 0;
        t->msg_pos = 0;
        t->msg_fetching_offset = 0;

        // Type
        t->msg[t->msg_filling_offset++] = CP_MSG_TYPE_LOAD_DAILY_ACTIVITY;
        // Steps
        t->msg[t->msg_filling_offset++] = (day_streaming.steps >> 24) & 0xff;
        t->msg[t->msg_filling_offset++] = (day_streaming.steps >> 16) & 0xff;
        t->msg[t->msg_filling_offset++] = (day_streaming.steps >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = (day_streaming.steps & 0xff);
        // distance
        t->msg[t->msg_filling_offset++] = (day_streaming.distance >> 24) & 0xff;
        t->msg[t->msg_filling_offset++] = (day_streaming.distance >> 16) & 0xff;
        t->msg[t->msg_filling_offset++] = (day_streaming.distance >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = (day_streaming.distance & 0xff);
        // calorie active
        t->msg[t->msg_filling_offset++] = (day_streaming.calories_active >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = (day_streaming.calories_active & 0xff);
        // calorie idle
        t->msg[t->msg_filling_offset++] = (day_streaming.calories_idle >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = (day_streaming.calories_idle & 0xff);
        // wake up times
        t->msg[t->msg_filling_offset++] = day_streaming.wake_up_time;
        // sleep light
        t->msg[t->msg_filling_offset++] = (day_streaming.sleep_light >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = (day_streaming.sleep_light & 0xff);
        // sleep sound
        t->msg[t->msg_filling_offset++] = (day_streaming.sleep_sound >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = (day_streaming.sleep_sound & 0xff);
        // sleep rem
        t->msg[t->msg_filling_offset++] = (day_streaming.sleep_rem >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = (day_streaming.sleep_rem & 0xff);
        // heart rate
        t->msg[t->msg_filling_offset++] = day_streaming.heart_rate;
        // skin temperature
        t->msg[t->msg_filling_offset++] = (day_streaming.temperature >> 8) & 0xff;
        t->msg[t->msg_filling_offset++] = (day_streaming.temperature & 0xff);
        // UV index
#ifdef _CLINGBAND_UV_MODEL_
        t->msg[t->msg_filling_offset++] = (cling.uv.max_uv + 5) / 10;
#else
        t->msg[t->msg_filling_offset++] = 0;
#endif
        t->msg_len = t->msg_filling_offset;

        // Get message length (fixed length: 2 bytes)
        t->pkt.len[0] = 0;
        t->pkt.len[1] = 0;
        t->pkt.len[2] = t->msg_filling_offset;

        // Create packet payload
        // Pending message delivery
        t->msg_type = CP_MSG_TYPE_LOAD_DAILY_ACTIVITY;
        t->state = CP_TX_PACKET_PENDING_SEND;
        g->state = CP_MCU_STAT_TX_SENDING;

        memcpy(&t->pkt.data, t->msg, CP_PAYLOAD_SIZE);

        t->msg_pos += CP_PAYLOAD_SIZE;
        t->msg_fetching_offset += CP_PAYLOAD_SIZE;
}

static void _delete_file_from_fs(BYTE *data)
{
        I8U *name = data + 1;

        FILE_delete(name);

        FILE_init();
}

static void _file_read_prepare_first_pkt(BYTE *f_name)
{
        CP_CTX *g = &cling.gcp;
        CP_TX_CTX *t = &g->tx;
        CP_RX_CTX *r = &g->rx;

        // Get overall file amount and message length
        if ((r->f.fc = FILE_fopen(f_name, FILE_IO_READ)) == 0) {
                // Reading file wrong, no packets will be sent

                // Relase MUTEX as we get to the end of a file
                SYSTEM_release_mutex(MUTEX_IOS_LOCK_VALUE);
                return;
        }

        t->msg_len = r->f.fc->size;
        t->msg_len++;

        // Creat a new message
        t->msg_id++;

        t->pkt.uuid[0] = (UUID_TX_START >> 8) & 0xff;
        t->pkt.uuid[1] = (UUID_TX_START & 0xff);
        t->pkt.id = t->msg_id;

        // Get message length
        t->pkt.len[0] = (t->msg_len >> 16) & 0xff;
        t->pkt.len[1] = (t->msg_len >> 8) & 0xff;
        t->pkt.len[2] = t->msg_len & 0xff;

        // Initialize message position
        t->msg_pos = 0;

        // Filling up the message buffer
        t->msg_filling_offset = 0;
        t->msg_fetching_offset = 0;
        t->msg_pos = 0;
        t->msg[t->msg_filling_offset++] = CP_MSG_TYPE_FILE_READ;

        // Create packet payload
        // Pending message delivery
        t->msg_type = CP_MSG_TYPE_FILE_READ;
        t->state = CP_TX_PACKET_PENDING_SEND;
        g->state = CP_MCU_STAT_TX_SENDING;

        if (t->msg_len > 64) {
                FILE_fread(r->f.fc, t->msg + t->msg_filling_offset, 64);
                t->msg_filling_offset += 64;
        }
        else {
                FILE_fread(r->f.fc, t->msg + t->msg_filling_offset, t->msg_len - 1);
                t->msg_filling_offset += t->msg_len - 1;
        }
        memcpy(t->pkt.data, t->msg, CP_PAYLOAD_SIZE);

        t->msg_pos += CP_PAYLOAD_SIZE;
        t->msg_fetching_offset += CP_PAYLOAD_SIZE;

        if (!FILE_feof(r->f.fc)) {
                FILE_fclose(r->f.fc);

                // Relase MUTEX as we get to the end of a file
                SYSTEM_release_mutex(MUTEX_IOS_LOCK_VALUE);
        }

        if (t->msg_pos >= t->msg_len) {
                g->state = CP_MCU_STAT_TX_COMPLETE;
        }
}

static void _read_file_from_fs(BYTE *data)
{
        CP_CTX *g = &cling.gcp;
        CP_RX_CTX *r = &g->rx;

        // Get file name length
        r->msg_file_name_len = data[0];

        // start compose a list of packets to read out file from CLING
        _file_read_prepare_first_pkt(data + 1);
}

static void _write_file_to_fs_rest(I8U *data)
{
        CP_CTX *g = &cling.gcp;
        CP_RX_CTX *r = &g->rx;

        // If a file is already written into file system, and we got some extra checksum data,
        // then, we shouldn't perform file write ever again.
        if (!r->msg_file_len) {
                // Relase MUTEX as we write the end of a file
                SYSTEM_release_mutex(MUTEX_IOS_LOCK_VALUE);
                OTA_set_state(FALSE);
                return;
        }

        // Move onto the next sector (128 bytes)
        if (r->msg_file_len > FAT_SECTOR_SIZE) {
                r->msg_file_len -= FAT_SECTOR_SIZE;
                uint16_t data_num_processed   __attribute__((unused)) = 0;
                if (is_app_package == true) {
                        /**/
                        uint8_t *data_p = data;
                        ota_p = HalOta_get_instance();
                        if (dialog_upgrade_offset % FAT_SECTOR_SIZE) {
                                uint16_t page_bytes_lft = FAT_SECTOR_SIZE - (dialog_upgrade_offset % FAT_SECTOR_SIZE);
                                ota_p->page_program(ota_p, dialog_upgrade_offset, page_bytes_lft,
                                        data_p);
                                dialog_upgrade_offset += page_bytes_lft;
                                data_num_processed = page_bytes_lft;
                                data_p += page_bytes_lft;
                                if (dialog_upgrade_offset % FLASH_BLOCK_SIZE == 0) {
                                        N_SPRINTF("[CP] dialog ota erase 4k");
                                        ota_p->erase_one_block_4k(ota_p, dialog_upgrade_offset);
                                }
                                page_bytes_lft = FAT_SECTOR_SIZE - (page_bytes_lft);
                                ota_p->page_program(ota_p, dialog_upgrade_offset, page_bytes_lft,
                                        data_p);
                                dialog_upgrade_offset += page_bytes_lft;
                        }
                        else {
                                N_SPRINTF("[CP] dialog file fs rest, file offset: %d, current: FAT_SECTOR_SIZE data0=0x%02x, data1 = 0x%02x", dialog_upgrade_offset, data[0], data[1]);
                                if (dialog_upgrade_offset % FLASH_BLOCK_SIZE == 0) {
                                        N_SPRINTF("[CP] dialog ota erase 4k");
                                        ota_p->erase_one_block_4k(ota_p, dialog_upgrade_offset);
                                }
                                ota_p->page_program(ota_p, dialog_upgrade_offset, FAT_SECTOR_SIZE,
                                        data);
                                dialog_upgrade_offset += FAT_SECTOR_SIZE;
                                data_num_processed = (FAT_SECTOR_SIZE);
//                        ota_p->write_enable(ota_p);
                        }
                        r->f.fc->crc = SYSTEM_CRC(data, FAT_SECTOR_SIZE, r->f.fc->crc);
//                        r->f.fc->tell += FAT_SECTOR_SIZE;
//                        app_fw_size += FAT_SECTOR_SIZE;

                }
                else {
                        N_SPRINTF("[CP] file fs rest, file len: %d, current: FAT_SECTOR_SIZE", r->msg_file_len);
                        FILE_fwrite(r->f.fc, data, FAT_SECTOR_SIZE);
                        data_num_processed = (FAT_SECTOR_SIZE);
                }

                if (OTA_if_enabled()) {
                        if (cling.ota.file_len == 0) {
                                cling.ota.file_len = r->msg_file_len;
                                N_SPRINTF("[CP] OTA file len re-init: %d", r->msg_file_len);
                        }
                        else {
                                cling.ota.percent = (I8U)(((cling.ota.file_len - r->msg_file_len)
                                        * 100) / cling.ota.file_len);
                                N_SPRINTF("[CP] ota: %d curr: %d, all: %d", cling.ota.percent, r->msg_file_len, cling.ota.file_len);
                        }
                }

        }
        else {

                if (is_app_package == true) {
                        ota_p = HalOta_get_instance();
                        N_SPRINTF("[CP] dialog file fs rest, file offset: %d, current: FAT_SECTOR_SIZE data0=0x%02x, data1 = 0x%02x", dialog_upgrade_offset, data[0], data[1]);
                        if (dialog_upgrade_offset % FLASH_BLOCK_SIZE == 0) {
                                N_SPRINTF("[CP] dialog ota erase 4k");
                                ota_p->erase_one_block_4k(ota_p, dialog_upgrade_offset);
                        }
                        ota_p->page_program(ota_p, dialog_upgrade_offset, r->msg_file_len, data);
                        dialog_upgrade_offset += r->msg_file_len;
                        r->f.fc->crc = SYSTEM_CRC(data, r->msg_file_len, r->f.fc->crc);
                        //  ota_p->write_enable(ota_p);
                }
                else {

                        // This is the final 128 bytes of the file.
                        FILE_fwrite(r->f.fc, data, r->msg_file_len);
                }
                printf("crc = %x/r/n", r->f.fc->crc);
                FILE_fclose(r->f.fc);
                N_SPRINTF("[CP] file closed (rest)- len: %d", r->msg_file_len);
                // Initialize user profile (name, weight, height, stride length)
                FILE_init();
                r->msg_file_len = 0;

                r->b_reload_profile = FALSE;

//                dialog_upgrade_offset = 0;

                // Relase MUTEX as we write the end of a file
                SYSTEM_release_mutex(MUTEX_IOS_LOCK_VALUE);
                OTA_set_state(FALSE);
                /*update firmware*/
//                SYSTEM_restart_from_reset_vector();

        }

}

static void _write_file_to_fs_head(BYTE *data)
{
        CP_CTX *g = &cling.gcp;
        CP_RX_CTX *r = &g->rx;
        I32U len;
        I8U *pd = data;
        I8U f_buf[128];

        // 4 BYTES - file length
        r->msg_file_len = *pd++;
        r->msg_file_len <<= 8;
        r->msg_file_len += *pd++;
        r->msg_file_len <<= 8;
        r->msg_file_len += *pd++;
        r->msg_file_len <<= 8;
        r->msg_file_len += *pd++;

        // If OTA is enabled, go ahead to initalize the percentage
        if (OTA_if_enabled()) {
                cling.ota.file_len = r->msg_file_len;
                cling.ota.percent = 0;
        }

        // 1 byte - File name length
        r->msg_file_name_len = *pd++;

        // Write file name first
        memcpy(f_buf, pd, r->msg_file_name_len);
        r->f.fc = FILE_fopen(f_buf, FILE_IO_WRITE);
        N_SPRINTF("[CP] fopen a file: %s", f_buf);

        if (r->f.fc == 0) {
                // Relase MUTEX as we get to the end of a file
                SYSTEM_release_mutex(MUTEX_IOS_LOCK_VALUE);
                OTA_set_state(FALSE);
                return;
        }

        if (!strcmp((char *)"profile.txt", (char *)f_buf)) {
                r->b_reload_profile = TRUE;
        }
        else if (!strcmp((char *)"ota_start.txt", (char *)f_buf)) {
                r->b_ota_start = TRUE;
                r->b_reload_profile = FALSE;
                /*clear dialog firmware upgrade offset address*/
//                dialog_upgrade_offset = 0;
//                is_app_package = true;
                N_SPRINTF("[CP] OTA start text file received");
        }
        else if (!strcmp((char *)"app.bin", (char *)f_buf)) {
                // App central start to download 'app.bin', which is a clear indicattor of OTA
                OTA_set_state(TRUE);
                //
                N_SPRINTF("[CP] Start download app.bin, set OTA flag");

                if (cling.ota.file_len == 0) {
                        cling.ota.file_len = r->msg_file_len;
                        N_SPRINTF("[CP] Initialize OTA file len: %d", r->msg_file_len);
                }
                /*clear dialog firmware upgrade offset address*/
                dialog_upgrade_offset = 0;
                is_app_package = true;
        }
        else {
                r->b_reload_profile = FALSE;
        }

        pd += r->msg_file_name_len;

        // If there is any data left, write it into file
        len = FAT_SECTOR_SIZE - r->msg_file_name_len - 1 /*type*/- 4 /*file length*/
        - 1 /*file name length*/;
        if (r->msg_file_len <= len) {

                FILE_fwrite(r->f.fc, pd, r->msg_file_len);

                FILE_fclose(r->f.fc);
                FILE_init();
                N_SPRINTF("[CP] file closed (head)- len: %d, file len: %d", len, r->msg_file_len);

                r->msg_file_len = 0;
                r->b_reload_profile = FALSE;

                // Relase MUTEX as we write the end of a file
                SYSTEM_release_mutex(MUTEX_IOS_LOCK_VALUE);
                OTA_set_state(FALSE);

                if (r->b_ota_start) {
                        // Over-the-air update check
                        OTA_main();
                }
        }
        else {
                if (is_app_package == true) {
                        ota_p = HalOta_get_instance();
                        /*erase first sector*/
                        ota_p->erase_one_block_4k(ota_p, dialog_upgrade_offset);
                        N_SPRINTF("[CP] dialog file fs head, file offset: %d, current: FAT_SECTOR_SIZE data0=0x%02x, data1 = 0x%02x", dialog_upgrade_offset, data[0], data[1]);
                        if ((dialog_upgrade_offset / FLASH_BLOCK_SIZE) < ((dialog_upgrade_offset + len)/ FLASH_BLOCK_SIZE)) {
                                /*if data is long engough to cover next sector we need to erase it ahead*/
                                N_SPRINTF("[CP] dialog ota erase 4k");
                                ota_p->erase_one_block_4k(ota_p, ((dialog_upgrade_offset/FLASH_BLOCK_SIZE) + 1)*FLASH_BLOCK_SIZE);
                        }
                        ota_p->page_program(ota_p, dialog_upgrade_offset, len, pd);
                        dialog_upgrade_offset += len;
                        r->f.fc->crc = 0xffff;
                        r->f.fc->crc = SYSTEM_CRC(pd, len, r->f.fc->crc);
                        //  ota_p->write_enable(ota_p);
                        r->f.fc->tell = 0;
                }
                else {
                        N_SPRINTF("[CP] file write (head), file len: %d, len: %d", r->msg_file_len, len);
                        // If this is the case, all the remaining data should be read out of Nor Flash
                        FILE_fwrite(r->f.fc, pd, len);
                }
                r->msg_file_len -= len;

        }
}

static void _create_ack_pkt()
{
        CP_CTX *g = &cling.gcp;
        CP_TX_CTX *t = &g->tx;

        // Creat a new message
        t->msg_id++;

        // ACK message length: 1B
        //
        t->msg_len = 2;

        t->pkt.uuid[0] = (UUID_TX_START >> 8) & 0xff;
        t->pkt.uuid[1] = (UUID_TX_START & 0xff);
        t->pkt.id = t->msg_id;

        // Get message length (fixed length: 2 bytes)
        t->pkt.len[0] = 0;
        t->pkt.len[1] = 0;
        t->pkt.len[2] = 2;

        // Filling up the message buffer
        t->pkt.data[0] = CP_MSG_TYPE_ACK;
        t->pkt.data[1] = GCIP_ACK_NORMAL;

        // Create packet payload
        // Pending message delivery
        t->msg_type = CP_MSG_TYPE_ACK;
        t->state = CP_TX_PACKET_PENDING_SEND;
        g->state = CP_MCU_STAT_TX_COMPLETE;
}

static void _pending_process()
{
        CP_PENDING_CTX *p = &cling.gcp.pending;
        N_SPRINTF("[CP] pending process");
        N_SPRINTF("[CP] pending - register is: %d, %d, %d", p->msg[0], p->task_id, p->b_touched);

#ifndef _CLING_PC_SIMULATION_

        switch (p->task_id) {
        case CP_MSG_TYPE_REGISTER_WRITE: {
                N_SPRINTF("[CP] write reg");
                _proc_pending_ctrl_wr();
                break;
        }
        case CP_MSG_TYPE_FILE_LOAD_LIST: {
                N_SPRINTF("[CP] load file list");
                // When iOS get MUTEX, it also rely on iOS to release MUTEX, or MCU release it when turning off the radio
                if (!SYSTEM_get_mutex(MUTEX_IOS_LOCK_VALUE))
                        return;
                _create_file_list_msg();
                break;
        }
        case CP_MSG_TYPE_LOAD_DEVICE_INFO: {
#ifdef _BLE_ENABLED_
                cling.gcp.host_type = p->msg[1];
                N_SPRINTF("[CP] load dev info: %d", cling.gcp.host_type);

//            HAL_disconnect_for_fast_connection(SWITCH_SPEED_FOR_DATA_SYNC);
                _create_dev_info_msg();
                BTLE_reset_streaming(); // Reset streaming as the App is trying to figure whether it is an authorized device
#endif
                break;
        }
        case CP_MSG_TYPE_LOAD_DAILY_ACTIVITY: {
                N_SPRINTF("[CP] load daily activity stat");
                _create_daily_activity_info_msg();
                BTLE_reset_streaming(); //
                break;
        }
        case CP_MSG_TYPE_DEBUG_MSG: {
                N_SPRINTF("[CP] debug msg");
                break;
        }
        case CP_MSG_TYPE_WEATHER: {
                N_SPRINTF("[CP] set weather");
                WEATHER_set_weather(p->msg + 1);
                break;
        }
        case CP_MSG_TYPE_USER_REMINDER: {
                N_SPRINTF("[CP] Reminder setup - all week");
                REMINDER_setup(p->msg + 1, FALSE);
                break;
        }
        case CP_MSG_TYPE_DAILY_ALARM: {
                N_SPRINTF("[CP] Reminder setup - daily");
                REMINDER_setup(p->msg + 1, TRUE);
                break;
        }
        case CP_MSG_TYPE_START_OTA: {
#ifdef _BLE_ENABLED_
                N_SPRINTF("[CP] << OTA enabled >>");
//            HAL_disconnect_for_fast_connection(SWITCH_SPEED_FOR_FW_UPGRADE);

                // Enable over the air update flag, and exit low power mode
                OTA_set_state(TRUE);
#endif
                break;
        }
        case CP_MSG_TYPE_REBOOT: {
                N_SPRINTF("[CP] reboot");
                SYSTEM_reboot();
                break;
        }
        case CP_MSG_TYPE_FILE_WRITE: {
                if (p->pending_len > p->processed_len) {

                        N_SPRINTF("[CP] file write: %d, %d", p->pending_len, p->processed_len);

                        if (!SYSTEM_get_mutex(MUTEX_IOS_LOCK_VALUE))
                                return;

                        if (p->pending_len <= FAT_SECTOR_SIZE) {
                                _write_file_to_fs_head(p->msg + 1);
                        }
                        else {
                                _write_file_to_fs_rest(p->msg);
                        }

                        _create_ack_pkt();

                        p->processed_len = p->pending_len;
                }
                break;
        }
        case CP_MSG_TYPE_FILE_DELETE: {
                if (!SYSTEM_get_mutex(MUTEX_IOS_LOCK_VALUE))
                        return;

                _delete_file_from_fs(p->msg + 1);

                // Relase MUTEX as we get to the end of a file list
                SYSTEM_release_mutex(MUTEX_IOS_LOCK_VALUE);
                break;
        }
        case CP_MSG_TYPE_FILE_READ: {
                if (!SYSTEM_get_mutex(MUTEX_IOS_LOCK_VALUE))
                        return;
                _read_file_from_fs(p->msg + 1);
                break;
        }
        case CP_MSG_TYPE_SIMULATION_CONFIG: {
                break;
        }
        case CP_MSG_TYPE_BLE_DISCONNECT: {
#ifdef _BLE_ENABLED_
                // Disconnect BLE service
                N_SPRINTF("[CP] BLE disconnecting ...");
                if (BTLE_is_connected()) {

                        CLASS(GapMgr) *gap_mgr = GapMgr_get_instance();
                        gap_mgr->disconnect(gap_mgr);
                }
                /*for debug purpose*/
//               BTLE_disconnect(BTLE_DISCONN_REASON_CP_DISCONN);
#endif
                break;
        }
        case CP_MSG_TYPE_SET_ANCS: {
                N_SPRINTF("[CP] SET CP_MSG_TYPE_SET_ANCS");
#ifdef _ENABLE_ANCS_
                if (p->msg[1]) {
                        N_SPRINTF("[CP] ANCS mode: enabled, %02x, %02x, %02x", p->msg[1], p->msg[2], p->msg[3]);
                        cling.ancs.supported_categories = p->msg[2];
                        cling.ancs.supported_categories <<= 8;
                        cling.ancs.supported_categories |= p->msg[3];
                }
#endif
                break;
        }
        case CP_MSG_TYPE_SET_LANGUAGE: {
                N_SPRINTF("[CP] SET LANGUAGE font type = %d", p->msg[1]);
                // Display fonts type.
                //      cling.ui.fonts_type = p->msg[1];
                break;
        }
        case CP_MSG_TYPE_SET_USER_PROFILE: {
                USER_setup_profile(p->msg + 1);
                break;
        }
        case CP_MSG_TYPE_ANDROID_NOTIFY: {
                N_SPRINTF("[CP] android notif received");
#ifdef _ENABLE_ANCS_
                NOTIFIC_smart_phone_notify(p->msg + 1);
#endif
                break;
        }
        case CP_MSG_TYPE_DEVICE_SETTING: {
                USER_setup_device(p->msg + 1, cling.user_data.setting_len - 1);
#ifndef _USE_HW_MOTION_DETECTION_
                // Configure accelerometer with the latest change
                LIS3DH_normal_FIFO();
#endif
                break;
        }
        default:
                break;
        }
#endif
        p->b_touched = FALSE; // Indicate that we have something pending
        N_SPRINTF("[CP] set b_touched to FALSE");

}

static void _filling_msg_tx_buf()
{
        CP_CTX *g = &cling.gcp;
        CP_RX_CTX *r = &g->rx;
        CP_TX_CTX *t = &g->tx;
        I8U *p1, *p2;
        I16U i, len, pos;

        if (t->msg_filling_offset >= (t->msg_fetching_offset + CP_PAYLOAD_SIZE))
                return;
#if 0
        if ((t->msg_type == CP_MSG_TYPE_FILE_LOAD_LIST) ||
                (t->msg_type == CP_MSG_TYPE_FILE_READ)) {
                // If we running out data, fill the buffer before we send out a packet.
                if (!SYSTEM_get_mutex(MUTEX_IOS_LOCK_VALUE))
                return;
        }
#endif

        if (t->msg_fetching_offset) {
                // Moving memory content
                len = t->msg_filling_offset - t->msg_fetching_offset;
                t->msg_filling_offset = len;
                p1 = t->msg;
                p2 = t->msg + t->msg_fetching_offset;
                if (len) {
                        while (len--) {
                                *p1++ = *p2++;
                        }
                }
        }

        // Reset fetching dialog_upgrade_offset to 0.
        t->msg_fetching_offset = 0;

        // Re-filling up the memory
        if (t->msg_type == CP_MSG_TYPE_FILE_LOAD_LIST) {
                if (t->msg_file_total > t->msg_file_id) {

                        // Checksum starting position
                        pos = t->msg_filling_offset;
                        for (i = t->msg_file_id; i < t->msg_file_total; i++) {
                                if (t->msg_filling_offset >= 64) {
                                        break;
                                }
                                t->msg_filling_offset = FILE_getFileInfo(i, t->msg,
                                        t->msg_filling_offset);
                        }
                        t->msg_file_id = i;

                        for (i = pos; i < t->msg_filling_offset; i++) {
                                t->msg_checksum += t->msg[i];
                        }

                        if (t->msg_file_id == t->msg_file_total) {
                                t->msg[t->msg_filling_offset++] = (t->msg_checksum >> 24) & 0xff;
                                t->msg[t->msg_filling_offset++] = (t->msg_checksum >> 16) & 0xff;
                                t->msg[t->msg_filling_offset++] = (t->msg_checksum >> 8) & 0xff;
                                t->msg[t->msg_filling_offset++] = t->msg_checksum & 0xff;

                                // Relase MUTEX as we get to the end of a file
                                SYSTEM_release_mutex(MUTEX_IOS_LOCK_VALUE);
                        }
                }
        }
        else if (t->msg_type == CP_MSG_TYPE_FILE_READ) {

                if (FILE_feof(r->f.fc)) {
                        // Read out maximum 64 bytes
                        len = FILE_fread(r->f.fc, t->msg + t->msg_filling_offset, 64);
                        t->msg_filling_offset += len;

                        if (!FILE_feof(r->f.fc)) {
                                FILE_fclose(r->f.fc);

                                // Relase MUTEX as we get to the end of a file
                                SYSTEM_release_mutex(MUTEX_IOS_LOCK_VALUE);
                        }
                }
        }
}

static BOOLEAN _core_cp_packet_handling()
{
#if 1
        CP_CTX *g = &cling.gcp;
        CP_TX_CTX *t = &g->tx;
        I32U curr_ts = CLK_get_system_time();
        CP_PENDING_CTX *p = &cling.gcp.pending;
        CP_TX_STREAMING_CTX *s = &cling.gcp.streaming;
//        N_SPRINTF("[cp]_core_cp_packet_handling");
        // 0. process this packet if a new one is received
        if (cling.gcp.b_new_pkt) {

                N_SPRINTF("[CP] receive a new packet");

                cling.gcp.b_new_pkt = FALSE;

                // Send packet data to C.P. for processing
                CP_API_received_pkt(cling.gcp.pkt_buff);
        }

        // 1. Process pending packets
        if (p->b_touched) {
                _pending_process();
        }

        // 2. check if we are running of buffer
        if (!BTLE_tx_buffer_available()) {
                N_SPRINTF("TX buffer NOT available");
                return FALSE;
        }

        // 3. Deliver a packet if needed, then update TX state after delivery
        if (t->state == CP_TX_PACKET_PENDING_SEND) {
                if (_tx_process(curr_ts)) {
                        N_SPRINTF("[CP] 15");
                        return TRUE;
                }
                else {
                        N_SPRINTF("[CP] 11");
                        return FALSE;
                }
        }

        // 4. Deliver an activity streaming packet if it is a pending
        if (s->pending) {
                if (_tx_streaming()) {
                        return TRUE;
                }
                else {
                        return FALSE;
                }
        }

        // In case that an ack packet is needed, go return, no further process
        if (t->state != CP_TX_IDLE) {
                N_SPRINTF("[cp]:  t->state != CP_TX_IDLE");
                return TRUE;
        }

        // Check whether we have any packet to deliver
        if (g->state != CP_MCU_STAT_TX_SENDING) {
                N_SPRINTF("[cp]: g->state != CP_MCU_STAT_TX_SENDING");
                return TRUE;
        }

        if (t->msg_filling_offset < t->msg_fetching_offset) {
                N_SPRINTF("[cp]: t->msg_filling_offset < t->msg_fetching_offse");
                return TRUE;
        }

        if (t->msg_len <= t->msg_pos) {
                N_SPRINTF("[cp]:t->msg_len <= t->msg_pos");
                return TRUE;
        }

        _filling_msg_tx_buf();

        _create_one_pkt_from_msg();

        // start sending next packet
        t->state = CP_TX_PACKET_PENDING_SEND;

        if (_tx_process(curr_ts)) {
                return TRUE;
        }
        else {
                return FALSE;
        }
#endif
}

void CP_state_machine_update()
{
        I8U i;

        for (i = 0; i < 5; i++) {
//                N_SPRINTF("[cp] i = %d", i);
                // Send up to 5 packets continuously
                if (!_core_cp_packet_handling()) {
                        N_SPRINTF("[cp] _core_cp_packet_handling return false");
                        break;

                }
        }
        N_SPRINTF("[cp] CP_state_machine_update end");
}

static void _ack_pkt_rcvr()
{
        CP_CTX *g = &cling.gcp;
        CP_TX_CTX *t = &g->tx;
        CP_TX_STREAMING_CTX *s = &cling.gcp.streaming;

        s->packet_is_acked = TRUE;

        if (t->state == CP_TX_PACKET_PENDING_ACK) {
                t->state = CP_TX_IDLE;
        }

        if (s->packet_need_ack) {
                s->flag_entry_read = TRUE;

                // Clear flag
                s->packet_need_ack = FALSE;
        }
        N_SPRINTF("[CP] ack packet received");

}

void CP_create_auth_stat_msg()
{
        CP_CTX *g = &cling.gcp;
        CP_TX_CTX *t = &g->tx;

        // Create a new message
        t->msg_id++;

        t->pkt.uuid[0] = (UUID_TX_START >> 8) & 0xff;
        t->pkt.uuid[1] = (UUID_TX_START & 0xff);
        t->pkt.id = t->msg_id;

        // Get message length
        t->msg_len = 6;  // 6 bytes - payload size, the whole authentication structure
        t->msg_len += 1;  // 1 bytes: msg type (1 byte)
        t->pkt.len[0] = (t->msg_len >> 16) & 0xff;
        t->pkt.len[1] = (t->msg_len >> 8) & 0xff;
        t->pkt.len[2] = t->msg_len & 0xff;

        // Initialize message position
        t->msg_pos = 0;
        t->msg_fetching_offset = 0;
        t->msg_checksum = 0;  // No check-sum on authentication packet
        t->msg_file_id = 0;

        // Filling up the message buffer
        t->msg_filling_offset = 0;
        t->msg[t->msg_filling_offset++] = CP_MSG_TYPE_AUTHENTICATIOIN;

        // Create packet payload
        // Pending message delivery
        t->msg_type = CP_MSG_TYPE_AUTHENTICATIOIN;
        t->state = CP_TX_PACKET_PENDING_SEND;
        g->state = CP_MCU_STAT_TX_SENDING;

        // Add all auth communication info
        t->msg[t->msg_filling_offset++] = cling.link.comm_state;
        t->msg[t->msg_filling_offset++] = cling.link.error_code;
        memcpy(t->msg + t->msg_filling_offset, (I8U *)(&cling.link.userID), 4);
        t->msg_filling_offset += 4;

        memcpy(t->pkt.data, t->msg, CP_PAYLOAD_SIZE);

        t->msg_fetching_offset += CP_PAYLOAD_SIZE;
        t->msg_pos += CP_PAYLOAD_SIZE;
        // Sending is completed
        g->state = CP_MCU_STAT_TX_COMPLETE;
}

void CP_create_sos_msg()
{
        CP_CTX *g = &cling.gcp;
        CP_TX_CTX *t = &g->tx;

        // Creat a new message
        t->msg_id++;

        // Get message length
        t->msg_len = CP_PAYLOAD_SIZE;

        // Single packet messagef
        t->pkt.uuid[0] = (UUID_TX_SP >> 8) & 0xff;
        t->pkt.uuid[1] = UUID_TX_SP & 0xff;

        // Filling up the message buffer
        t->msg_filling_offset = 0;
        t->msg[t->msg_filling_offset++] = CP_MSG_TYPE_SOS_MESSAGE;

        // Create packet payload
        // Pending message delivery
        t->msg_type = CP_MSG_TYPE_SOS_MESSAGE;
        t->state = CP_TX_PACKET_PENDING_SEND;
        g->state = CP_MCU_STAT_TX_COMPLETE;

        memcpy(&t->pkt.id, t->msg, t->msg_filling_offset);

}

void CP_create_phone_finder_msg()
{
        CP_CTX *g = &cling.gcp;
        CP_TX_CTX *t = &g->tx;

        // Creat a new message
        t->msg_id++;

        // Get message length
        t->msg_len = CP_PAYLOAD_SIZE;

        // Single packet messagef
        t->pkt.uuid[0] = (UUID_TX_SP >> 8) & 0xff;
        t->pkt.uuid[1] = UUID_TX_SP & 0xff;

        // Filling up the message buffer
        t->msg_filling_offset = 0;
        t->msg[t->msg_filling_offset++] = CP_MSG_TYPE_PHONE_FINDER_MESSAGE;

        // Create packet payload
        // Pending message delivery
        t->msg_type = CP_MSG_TYPE_PHONE_FINDER_MESSAGE;
        t->state = CP_TX_PACKET_PENDING_SEND;
        g->state = CP_MCU_STAT_TX_COMPLETE;

        memcpy(&t->pkt.id, t->msg, t->msg_filling_offset);

}

void CP_create_register_rd_msg()
{
        CP_CTX *g = &cling.gcp;
        CP_TX_CTX *t = &g->tx;

        // Creat a new message
        t->msg_id++;

        // Get message length
        t->msg_len = CP_PAYLOAD_SIZE;

        // Single packet messagef
        t->pkt.uuid[0] = (UUID_TX_SP >> 8) & 0xff;
        t->pkt.uuid[1] = UUID_TX_SP & 0xff;

        // Filling up the message buffer
        t->msg_filling_offset = 0;
        t->msg[t->msg_filling_offset++] = CP_MSG_TYPE_REGISTER_READ;

        // Return with all MCU register values
        memcpy(t->msg + t->msg_filling_offset, cling.system.mcu_reg, SYSTEM_REGISTER_MAX);

        t->msg_filling_offset += SYSTEM_REGISTER_MAX;

        // Create packet payload
        // Pending message delivery
        t->msg_type = CP_MSG_TYPE_REGISTER_READ;
        t->state = CP_TX_PACKET_PENDING_SEND;
        g->state = CP_MCU_STAT_TX_COMPLETE;

        memcpy(&t->pkt.id, t->msg, t->msg_filling_offset);

}

static void _fillup_streaming_packet(I8U *pkt, MINUTE_TRACKING_CTX *pminute, I8U mode,
        I16U same_entry_num)
{
#ifdef _BLE_ENABLED_
        // Filling up the single packet message
        SYSTIME_CTX local;
        I8U filling_offset = 0;
        I32U pkt_index = cling.ble.streaming_minute_file_index;

        pkt_index <<= 8; // Each file contains 256 entries;
        pkt_index += cling.ble.streaming_minute_scratch_index;

        if (pkt_index > 65535) {
                pkt_index = 65535;
        }

        //
        // Streaming packet (20 B):
        //
        //             UUID: 2 B
        //             type: 1 B
        //            index: 2 B
        //            epoch: 4 B
        // skin temperature: 2 B
        //          Walking: 1 B
        //          Running: 1 B
        //         Calories: 1 B
        //         distance: 1 B
        //     Sleep second: 1 B
        //       heart rate: 1 B
        //       skin touch: 1 B
        //   Activity count: 2 B
        //    workouts & uv: 1 B
        //
        // Characteristcs -
        pkt[filling_offset++] = (UUID_TX_SP >> 8) & 0xff;
        pkt[filling_offset++] = UUID_TX_SP & 0xff;

        // Type (1B)
        pkt[filling_offset++] = mode;

        if (mode == CP_MSG_TYPE_STREAMING_MULTI_MINUTES) {
                // Entry number (2B)
                pkt[filling_offset++] = (same_entry_num >> 8) & 0xff;
                pkt[filling_offset++] = same_entry_num & 0xff;
        }
        else {
                // Index (2B)
                pkt[filling_offset++] = (I8U)((pkt_index >> 8) & 0xff);
                pkt[filling_offset++] = (I8U)(pkt_index & 0xff);
        }

        // Epoch (4B)
        pkt[filling_offset++] = (pminute->epoch >> 24) & 0xff;
        pkt[filling_offset++] = (pminute->epoch >> 16) & 0xff;
        pkt[filling_offset++] = (pminute->epoch >> 8) & 0xff;
        pkt[filling_offset++] = pminute->epoch & 0xff;

        RTC_get_local_clock(pminute->epoch, &local);

        N_SPRINTF("[CP] time (idx:%d): %d, %d, %d", cling.ble.streaming_minute_scratch_index, local.day, local.hour, local.minute);

        // skin temperature (2B)
        pkt[filling_offset++] = (pminute->skin_temperature >> 8) & 0xff;
        pkt[filling_offset++] = pminute->skin_temperature & 0xff;

        // Walking (1B)
        pkt[filling_offset++] = pminute->walking;

        // Running (1B)
        pkt[filling_offset++] = pminute->running;

        N_SPRINTF("[CP] time: %d, %d, %d(%d, %d)", local.day, local.hour, local.minute, pminute->walking, pminute->running);

        // Calories (1B)
        pkt[filling_offset++] = pminute->calories;

        // distance (1B)
        pkt[filling_offset++] = pminute->distance;

        // sleep seconds (1B)
        pkt[filling_offset++] = pminute->sleep_state;

        // heart rate (1B)
        pkt[filling_offset++] = pminute->heart_rate;

        // skin touch (1B)
        pkt[filling_offset++] = pminute->skin_touch_pads;

        N_SPRINTF("[CP] Sleep state: %02x, skin touch: %02x", pminute->sleep_state, pminute->skin_touch_pads);

        // Activity count (2B)
        pkt[filling_offset++] = (pminute->activity_count >> 8) & 0xff;
        pkt[filling_offset++] = pminute->activity_count & 0xff;

        N_SPRINTF("[CP] activity counts: %04x", pminute->activity_count);

        // Workouts and UV
        pkt[filling_offset++] = pminute->uv_and_activity_type;

        N_SPRINTF("[CP] streaming UV: %02x", pminute->uv_and_activity_type);
#endif
}

BOOLEAN CP_create_streaming_file_minute_msg(I32U space_size)
{
#ifdef _BLE_ENABLED_
        CP_TX_STREAMING_CTX *s = &cling.gcp.streaming;
        I32U minute_data_1[4];
        I32U minute_data_2[4];
        I8U *pbuf_1 = (I8U *)minute_data_1;
        I8U *pbuf_2 = (I8U *)minute_data_2;
        MINUTE_TRACKING_CTX *pminute_1 = (MINUTE_TRACKING_CTX *)minute_data_1;
        MINUTE_TRACKING_CTX *pminute_2 = (MINUTE_TRACKING_CTX *)minute_data_2;
        I32U dialog_upgrade_offset, offset2, i;

        // If there is already a pending activity streaming packet,
        // ignore further streaming request until current one is sent out.
        //
        if (s->pending)
                return TRUE;

        // If last packet is not yet acknowledged, go back return;
        if (s->packet_need_ack)
                return TRUE;

        if (s->flag_entry_read) {
                N_SPRINTF("[CP] flag entry read is set: %d, %d", s->minutes_repeat_num, cling.ble.minute_file_entry_index);
                for (i = 0; i < s->minutes_repeat_num; i++) {

                        // Flag this entry to be uploaded
                        dialog_upgrade_offset = cling.ble.minute_file_entry_index << 4;
                        N_SPRINTF("[CP] flag read: %d, %d, %d", dialog_upgrade_offset, s->minutes_repeat_num, i);
                        if (dialog_upgrade_offset >= space_size) {
                                break;
                        }
                        FLASH_Read_Flash(dialog_upgrade_offset + cling.ble.streaming_file_addr,
                                (I8U *)minute_data_1, 4);
                        N_SPRINTF("[CP] flag minute entry: %d, %d", dialog_upgrade_offset >> 4, pminute_1->epoch);
                        pminute_1->epoch &= 0x7fffffff;
                        FLASH_Write_Flash(dialog_upgrade_offset + cling.ble.streaming_file_addr,
                                (I8U *)minute_data_1, 4);
                        cling.ble.minute_file_entry_index++;
                }
                s->flag_entry_read = FALSE;
        }

        // Get current ofset
        dialog_upgrade_offset = cling.ble.minute_file_entry_index << 4;

        // If we get to the end of scrach pad, go return;
        if (dialog_upgrade_offset >= space_size)
                return FALSE;

        N_SPRINTF("Create one streaming message: %d", cling.ble.minute_file_entry_index);

        // Find a valid entry (the first)
        s->minutes_repeat_num = 1;
        while (dialog_upgrade_offset < space_size) {
                FLASH_Read_Flash(dialog_upgrade_offset + cling.ble.streaming_file_addr, pbuf_1, 16);

                if ((pbuf_1[0] != 0xff) && (pminute_1->epoch & 0x80000000)) {

                        // Flag this entry to be read
                        pminute_1->epoch &= 0x7fffffff;
                        N_SPRINTF("[CP] found new entry (%d): %d", s->minutes_repeat_num, dialog_upgrade_offset >> 4);
                        break;
                }
                cling.ble.minute_file_entry_index++;
                dialog_upgrade_offset = cling.ble.minute_file_entry_index << 4;
        }

        // Find the repeated entries
        //
        if (dialog_upgrade_offset < space_size) {
                offset2 = (cling.ble.minute_file_entry_index + 1) << 4;
                N_SPRINTF("[CP] find repeat entries: %d, %d, %d", dialog_upgrade_offset, offset2, cling.ble.minute_file_entry_index);
                while (offset2 < space_size) {
                        FLASH_Read_Flash(offset2 + cling.ble.streaming_file_addr, pbuf_2, 16);
                        // Invalid entry, go return
                        if (pbuf_2[0] == 0xff) {
                                break;
                        }

                        // The entry is already uploaded, go return
                        if (!(pminute_2->epoch & 0x80000000)) {
                                break;
                        }

                        // Check epoch
                        pminute_2->epoch &= 0x7fffffff;

                        // New entry has to have a larger epoch
                        if (pminute_2->epoch <= pminute_1->epoch) {
                                N_SPRINTF("[CP] different epoch: %d, %d", pminute_2->epoch, pminute_1->epoch);
                                break;
                        }

                        if (pminute_2->walking != pminute_1->walking) {
                                N_SPRINTF("[CP] different walking: %d, %d", pminute_2->walking, pminute_1->walking);
                                break;
                        }
                        if (pminute_2->running != pminute_1->running) {
                                N_SPRINTF("[CP] different running: %d, %d", pminute_2->running, pminute_1->running);
                                break;
                        }
                        if (pminute_2->calories != pminute_1->calories) {
                                N_SPRINTF("[CP] different c: %d, %d", pminute_2->calories, pminute_1->calories);
                                break;
                        }
                        if (pminute_2->distance != pminute_1->distance) {
                                N_SPRINTF("[CP] different d: %d, %d", pminute_2->distance, pminute_1->distance);
                                break;
                        }
                        if (pminute_2->sleep_state != pminute_1->sleep_state) {
                                N_SPRINTF("[CP] different sleep: %d, %d", pminute_2->sleep_state, pminute_1->sleep_state);
                                break;
                        }
                        if (pminute_2->activity_count != pminute_1->activity_count) {
                                N_SPRINTF("[CP] different activity: %d, %d", pminute_2->activity_count, pminute_1->activity_count);
                                break;
                        }
                        if (pminute_2->heart_rate != pminute_1->heart_rate) {
                                N_SPRINTF("[CP] different h: %d, %d", pminute_2->heart_rate, pminute_1->heart_rate);
                                break;
                        }
                        if (pminute_2->skin_touch_pads != pminute_1->skin_touch_pads) {
                                N_SPRINTF("[CP] different skin touch: %d, %d", pminute_2->skin_touch_pads, pminute_1->skin_touch_pads);
                                break;
                        }

                        // If we can come down here, pretty much this entry has the same content as previous one, go increase the repeat number
                        s->minutes_repeat_num++;
                        N_SPRINTF("[CP] found repeated entry (%d): %d, %d", (offset2 >> 4), s->minutes_repeat_num, pminute_2->epoch);
                        offset2 += 16;
                }
        }
        N_SPRINTF("Streaming entry (%d):%d,  %d", s->minutes_repeat_num, cling.ble.minute_file_entry_index, pminute_1->epoch);

        // If we get to the end of scrach pad, still send out a packet to let the APP
        // know that we are done;

        // Filling up the single packet message
        if (s->minutes_repeat_num > 1) {
                _fillup_streaming_packet(s->pkt, pminute_1, CP_MSG_TYPE_STREAMING_MULTI_MINUTES,
                        s->minutes_repeat_num);
                N_SPRINTF("Streaming multi-entry (%d): %d, %d", s->minutes_repeat_num, cling.ble.minute_file_entry_index, pminute_1->epoch);
        }
        else {
                _fillup_streaming_packet(s->pkt, pminute_1, CP_MSG_TYPE_STREAMING_MINUTE, 0);
                N_SPRINTF("Streaming single-entry (%d): %d, %d", s->minutes_repeat_num, cling.ble.minute_file_entry_index, pminute_1->epoch);
        }

        // It is a pending message, and also need ACK
        s->pending = TRUE;
        s->packet_need_ack = TRUE;
#endif
        return TRUE;
}

BOOLEAN CP_create_streaming_minute_msg(I32U space_size)
{
        CP_TX_STREAMING_CTX *s = &cling.gcp.streaming;
        I32U minute_data_1[4];
        I32U minute_data_2[4];
        I8U *pbuf_1 = (I8U *)minute_data_1;
        I8U *pbuf_2 = (I8U *)minute_data_2;
        MINUTE_TRACKING_CTX *pminute_1 = (MINUTE_TRACKING_CTX *)minute_data_1;
        MINUTE_TRACKING_CTX *pminute_2 = (MINUTE_TRACKING_CTX *)minute_data_2;
        I32U dialog_upgrade_offset, offset2, i;

        // If there is already a pending activity streaming packet,
        // ignore further streaming request until current one is sent out.
        //
        if (s->pending)
                return TRUE;

        // If last packet is not yet acknowledged, go back return;
        if (s->packet_need_ack)
                return TRUE;

        if (s->flag_entry_read) {
                N_SPRINTF("[CP] flag entry read is set: %d, %d", s->minutes_repeat_num, cling.ble.streaming_minute_scratch_index);
                for (i = 0; i < s->minutes_repeat_num; i++) {

                        // Flag this entry to be uploaded
                        dialog_upgrade_offset = cling.ble.streaming_minute_scratch_index << 4;
                        N_SPRINTF("[CP] flag read: %d, %d, %d", dialog_upgrade_offset, s->minutes_repeat_num, i);
                        if (dialog_upgrade_offset >= space_size) {
                                break;
                        }
                        FLASH_Read_App(dialog_upgrade_offset + SYSTEM_TRACKING_SPACE_START,
                                (I8U *)minute_data_1,
                                4);
                        N_SPRINTF("[CP] flag minute entry: %d, %d", dialog_upgrade_offset >> 4, pminute_1->epoch);
                        pminute_1->epoch &= 0x7fffffff;
                        FLASH_Write_App(dialog_upgrade_offset + SYSTEM_TRACKING_SPACE_START,
                                (I8U *)minute_data_1,
                                4);
                        cling.ble.streaming_minute_scratch_index++;
                }
                s->flag_entry_read = FALSE;
        }

        // Get current ofset
        dialog_upgrade_offset = cling.ble.streaming_minute_scratch_index << 4;

        // If we get to the end of scrach pad, go return;
        if (dialog_upgrade_offset >= space_size)
                return FALSE;

        N_SPRINTF("Create one streaming message: %d", cling.ble.streaming_minute_scratch_index);

        // Find a valid entry (the first)
        s->minutes_repeat_num = 1;
        while (dialog_upgrade_offset < space_size) {
                FLASH_Read_App(dialog_upgrade_offset + SYSTEM_TRACKING_SPACE_START, pbuf_1, 16);
#ifdef _JACOB_TESTING_
                if ((pbuf_1[0] != 0xff) /*&& (pminute_1->epoch & 0x80000000)*/) {
#else
                if ((pbuf_1[0] != 0xff) && (pminute_1->epoch & 0x80000000)) {
#endif

                        // Flag this entry to be read
                        pminute_1->epoch &= 0x7fffffff;
                        N_SPRINTF("[CP] found new entry (%d): %d", s->minutes_repeat_num, dialog_upgrade_offset >> 4);

                        break;
                }
#ifdef _BLE_ENABLED_
                cling.ble.streaming_minute_scratch_index++;
                dialog_upgrade_offset = cling.ble.streaming_minute_scratch_index << 4;
#endif
        }

        // Find the repeated entries
        if (dialog_upgrade_offset < space_size) {
#ifdef _BLE_ENABLED_
                offset2 = (cling.ble.streaming_minute_scratch_index + 1) << 4;
#endif
                N_SPRINTF("[CP] find repeat entries: %d, %d, %d", dialog_upgrade_offset, offset2, cling.ble.streaming_minute_scratch_index);
                while (offset2 < space_size) {
                        FLASH_Read_App(offset2 + SYSTEM_TRACKING_SPACE_START, pbuf_2, 16);
                        // Invalid entry, go return
                        if (pbuf_2[0] == 0xff) {
                                break;
                        }
#ifndef _JACOB_TESTING_
                        // The entry is already uploaded, go return
                        if (!(pminute_2->epoch & 0x80000000)) {
                                break;
                        }
#endif
                        // Check epoch
                        pminute_2->epoch &= 0x7fffffff;

                        // New entry has to have a larger epoch
                        if (pminute_2->epoch <= pminute_1->epoch) {
                                N_SPRINTF("[CP] different epoch: %d, %d", pminute_2->epoch, pminute_1->epoch);
                                break;
                        }

                        if (pminute_2->walking != pminute_1->walking) {
                                N_SPRINTF("[CP] different walking: %d, %d", pminute_2->walking, pminute_1->walking);
                                break;
                        }
                        if (pminute_2->running != pminute_1->running) {
                                N_SPRINTF("[CP] different running: %d, %d", pminute_2->running, pminute_1->running);
                                break;
                        }
                        if (pminute_2->calories != pminute_1->calories) {
                                N_SPRINTF("[CP] different c: %d, %d", pminute_2->calories, pminute_1->calories);
                                break;
                        }
                        if (pminute_2->distance != pminute_1->distance) {
                                N_SPRINTF("[CP] different d: %d, %d", pminute_2->distance, pminute_1->distance);
                                break;
                        }
                        if (pminute_2->sleep_state != pminute_1->sleep_state) {
                                N_SPRINTF("[CP] different sleep: %d, %d", pminute_2->sleep_state, pminute_1->sleep_state);
                                break;
                        }
                        if (pminute_2->activity_count != pminute_1->activity_count) {
                                N_SPRINTF("[CP] different activity: %d, %d", pminute_2->activity_count, pminute_1->activity_count);
                                break;
                        }
                        if (pminute_2->heart_rate != pminute_1->heart_rate) {
                                N_SPRINTF("[CP] different h: %d, %d", pminute_2->heart_rate, pminute_1->heart_rate);
                                break;
                        }
                        if (pminute_2->skin_touch_pads != pminute_1->skin_touch_pads) {
                                N_SPRINTF("[CP] different skin touch: %d, %d", pminute_2->skin_touch_pads, pminute_1->skin_touch_pads);
                                break;
                        }

                        // If we can come down here, pretty much this entry has the same content as previous one, go increase the repeat number
                        s->minutes_repeat_num++;
                        N_SPRINTF("[CP] found repeated entry (%d): %d, %d", (offset2 >> 4), s->minutes_repeat_num, pminute_2->epoch);
                        offset2 += 16;
                }
        }
        N_SPRINTF("Streaming entry (%d):%d,  %d", s->minutes_repeat_num, cling.ble.streaming_minute_scratch_index, pminute_1->epoch);

        // If we get to the end of scrach pad, still send out a packet to let the APP
        // know that we are done;

        // Filling up the single packet message
        if (s->minutes_repeat_num > 1) {
                _fillup_streaming_packet(s->pkt, pminute_1, CP_MSG_TYPE_STREAMING_MULTI_MINUTES,
                        s->minutes_repeat_num);
                N_SPRINTF("Streaming multi-entry (%d): %d, %d", s->minutes_repeat_num, cling.ble.streaming_minute_scratch_index, pminute_1->epoch);
        }
        else {
                _fillup_streaming_packet(s->pkt, pminute_1, CP_MSG_TYPE_STREAMING_MINUTE, 0);
                N_SPRINTF("Streaming single-entry (%d): %d, %d", s->minutes_repeat_num, cling.ble.streaming_minute_scratch_index, pminute_1->epoch);
        }

        // It is a pending message, and also need ACK
        s->pending = TRUE;
        s->packet_need_ack = TRUE;
        return TRUE;
}

void CP_create_streaming_daily_msg(void)
{
        CP_TX_STREAMING_CTX *s = &cling.gcp.streaming;
        I8U filling_offset = 0;
        I8U *pkt = s->pkt;
        I32U steps, calories, distance;

        // If there is already a pending activity streaming packet,
        // ignore further streaming request until current one is sent out.
        //
        if (s->pending)
                return;

        // Filling up the single packet message

        //
        // Streaming daly packet (17 B):
        //
        //             type: 1 B
        //            epoch: 4 B
        //            Steps: 4 B
        // skin temperature: 2 B
        //         Calories: 2 B
        //         distance: 2 B
        //       heart rate: 1 B
        //       skin touch: 1 B
        //

        // Characteristcs -
        pkt[filling_offset++] = (UUID_TX_SP >> 8) & 0xff;
        pkt[filling_offset++] = UUID_TX_SP & 0xff;

        // Type (1B)
        pkt[filling_offset++] = CP_MSG_TYPE_STREAMING_DAILY;

        // Epoch (4B)
        pkt[filling_offset++] = (cling.time.time_since_1970 >> 24) & 0xff;
        pkt[filling_offset++] = (cling.time.time_since_1970 >> 16) & 0xff;
        pkt[filling_offset++] = (cling.time.time_since_1970 >> 8) & 0xff;
        pkt[filling_offset++] = (cling.time.time_since_1970 & 0xff);

        // Steps (4B)
        steps = cling.activity.day.running + cling.activity.day.walking;
        pkt[filling_offset++] = (steps >> 24) & 0xff;
        pkt[filling_offset++] = (steps >> 16) & 0xff;
        pkt[filling_offset++] = (steps >> 8) & 0xff;
        pkt[filling_offset++] = (steps & 0xff);

        // skin temperature (2B)
        pkt[filling_offset++] = 0; //(cling.therm.current_temperature >> 8) & 0xff;
        pkt[filling_offset++] = 0; //(cling.therm.current_temperature & 0xff);

        // Calories (2B)
        calories = cling.activity.day.calories >> 4;
        pkt[filling_offset++] = (calories >> 8) & 0xff;
        pkt[filling_offset++] = (calories & 0xff);

        // distance (2B)
        distance = cling.activity.day.distance >> 4;
        pkt[filling_offset++] = (distance >> 8) & 0xff;
        pkt[filling_offset++] = (distance & 0xff);

        // heart rate (1B)
        pkt[filling_offset++] = 0; //cling.hr.current_rate;

        // skin touch (1B)
        pkt[filling_offset++] = 0; //TOUCH_is_skin_touched();
#ifdef _CLINGBAND_UV_MODEL_
                // UV index
                pkt[filling_offset++] = (cling.uv.max_uv + 5) / 10;
#else
        pkt[filling_offset++] = 0;
#endif

        s->pending = TRUE;
}

void _write_ctrl_reg(I8U *reg)
{
        I8U i, v;
        I8U idx = 0x80;
        CP_PENDING_CTX *p = &cling.gcp.pending;

        // check whether it is a command to enable a system mode
        if (reg[REGISTER_MCU_CTL]) {
                v = reg[REGISTER_MCU_CTL];
                for (i = 0; i < 8; i++) {
                        if (v & idx) {
                                p->task_id = CP_MSG_TYPE_REGISTER_WRITE;
                                p->msg[0] = idx;
                                memcpy(p->msg + 1, reg, CP_PACKET_SIZE);
                                cling.system.mcu_reg[REGISTER_MCU_PROTECT] =
                                        reg[REGISTER_MCU_PROTECT];
                                p->b_touched = TRUE; // Indicate that we have something pending
                                break;
                        }
                        idx >>= 1;
                }
        }
        else if (reg[REGISTER_MCU_CLEAR]) {
                // Disable certain system mode
                v = reg[REGISTER_MCU_CLEAR];
                for (i = 0; i < 8; i++) {
                        if (v & idx) {
                                cling.system.mcu_reg[REGISTER_MCU_CTL] &= ~idx;
                                cling.system.mcu_reg[REGISTER_MCU_CLEAR] |= idx;
                                break;
                        }
                        idx >>= 1;

                }
        }
}

// Single packet message process
static void _rx_sp_msg_process(I8U *data)
{
        CP_CTX *g = &cling.gcp;
        CP_RX_CTX *r = &g->rx;

        switch (r->msg_type) {
        case CP_MSG_TYPE_REGISTER_READ:
                N_SPRINTF("[CP] msg read");
                CP_create_register_rd_msg();
                break;
        case CP_MSG_TYPE_REGISTER_WRITE:
                N_SPRINTF("[CP] msg write");
                _write_ctrl_reg(data);
                CP_create_register_rd_msg();
                break;
        default:
                break;
        }
}

void _store_rx_msg(CP_RX_CTX *r, I8U *data, I32U pos)
{
        CP_PENDING_CTX *p = &cling.gcp.pending;
        I32U dialog_upgrade_offset = pos & 0x7f;
        I32U len = pos + CP_PAYLOAD_SIZE;

        // Copy it to a temporairy buffer
        memcpy(r->msg + dialog_upgrade_offset, data, CP_PAYLOAD_SIZE);

        // Update the position
        dialog_upgrade_offset += CP_PAYLOAD_SIZE;

        N_SPRINTF("[CP] we receive a packet: %d, %d, %d", pos, r->msg_len, len);

        // If message length is greater than 128
        if (r->msg_len > (I8U)FAT_SECTOR_SIZE) {
                if ((len >= r->msg_len) || (dialog_upgrade_offset >= (I8U)FAT_SECTOR_SIZE)) {
                        // We have received either 128 bytes or the last packet, it is time to process it
                        p->b_touched = TRUE;
                        p->task_id = r->msg_type;
                        p->pending_len = len;
                        memcpy(p->msg, r->msg, FAT_SECTOR_SIZE);
                }
        }
        else {
                // Check whether we have received the whole message
                // if so, go process the message
                if (dialog_upgrade_offset < r->msg_len)
                        return;

                p->task_id = r->msg[0];

                if (p->task_id == CP_MSG_TYPE_ACK) {
                        _ack_pkt_rcvr();
                        return;
                }

                // We have pending command that needs to be processed
                p->b_touched = TRUE;

                N_SPRINTF("Message received: %d", r->msg_len);

                if (p->task_id == CP_MSG_TYPE_DEVICE_SETTING) {
                        cling.user_data.setting_len = r->msg_len;
                        N_SPRINTF("DEVICE SETTING: %d", cling.user_data.setting_len);
                }
                p->pending_len = dialog_upgrade_offset;

                memcpy(p->msg, r->msg, FAT_SECTOR_SIZE);
        }
}

void CP_API_received_pkt(I8U *data_in)
{
        CP_PENDING_CTX *p = &cling.gcp.pending;
        CP_CTX *g = &cling.gcp;
        CP_PACKET *pkt = (CP_PACKET *)data_in;
        CP_RX_CTX *r = &g->rx;
        I32U pos;

        r->UUID = pkt->uuid[0];
        r->UUID <<= 8;
        r->UUID += pkt->uuid[1];
        N_SPRINTF("[cp]: uuid =  %d",r->UUID);
        // Single packet message
        if (r->UUID == UUID_RX_SP) {
                // Reset pending status
                p->pending_len = 0;
                p->processed_len = 0;

                // Load message type
                r->msg_type = pkt->id;

                // Process the single packet message
                _rx_sp_msg_process(pkt->len);

        }
        else if (r->UUID == UUID_RX_START) {	// Multi-packet message

                // Reset pending status
                p->pending_len = 0;
                p->processed_len = 0;

                // The start packet of a multi-packet message
                //r->msg_id = pkt->id;
                r->msg_len = (I8U)pkt->len[0];
                r->msg_len <<= 8;
                r->msg_len |= (I8U)pkt->len[1];
                r->msg_len <<= 8;
                r->msg_len |= (I8U)pkt->len[2];        //

                N_SPRINTF("msg length: %d", r->msg_len);

                // The first byte of payload in the starting packet indicates the message type
                r->msg_type = pkt->data[0];

                // Store this message, and process it
                _store_rx_msg(r, pkt->data, 0);

        }
        else if ((r->UUID == UUID_RX_MIDDLE) || (r->UUID == UUID_RX_END)) {

                // Get message dialog_upgrade_offset
                pos = (I8U)pkt->len[0];
                pos <<= 8;
                pos |= (I8U)pkt->len[1];
                pos <<= 8;
                pos |= (I8U)pkt->len[2];

                _store_rx_msg(r, pkt->data, pos);
        }
}

