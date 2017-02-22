/***************************************************************************/
/**
 * File: system.c
 *
 * Description: system layer implementation
 *
 * Created on Jan 26, 2014
 *
 ******************************************************************************/
#include "main.h"

#include "fs_file.h"
#include "ble_gap_mgr.h"
//#include "sysflash_rw.h"
static TASK_HANDLE_SETS *task_handles = NULL;
static const unsigned short crc_table[256] = {

    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5,
    0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b,
    0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
    0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c,
    0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b,
    0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
    0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
    0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5,
    0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969,
    0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96,
    0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03,
    0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6,
    0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb,
    0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1,
    0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c,
    0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2,
    0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
    0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447,
    0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2,
    0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827,
    0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c,
    0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0,
    0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d,
    0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
    0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba,
    0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

I16U CRCCCITT(I8U *data, I32U length, unsigned short seed, unsigned short final)
{
    I16U count;
    I16U crc = seed;
    I16U temp;

    for (count = 0; count < length; ++count) {
        temp = (*data++ ^ (crc >> 8)) & 0xff;
        crc = crc_table[temp] ^ (crc << 8);
    }

    return (crc ^ final);
}

I16U SYSTEM_CRC(I8U *data, I32U len, I16U seed)
{
    return CRCCCITT(data, len, seed, 0x0000);
}

void SYSTEM_get_ble_code(I8U *code)
{
    I8U dev_id[20];
    SYSTEM_get_dev_id(dev_id);

    code[0] = dev_id[16];
    code[1] = dev_id[17];
    code[2] = dev_id[18];
    code[3] = dev_id[19];
}

I8U SYSTEM_get_ble_device_name(I8U *device_name)
{
    I8U dev_id[20];
    I8U len;
    SYSTEM_get_dev_id(dev_id);

#ifdef _CLINGBAND_RAINFLOWER_MODEL_
    len = sprintf((char *)device_name, "RBAND ");
#endif

    device_name[len++] = dev_id[16];
    device_name[len++] = dev_id[17];
    device_name[len++] = dev_id[18];
    device_name[len++] = dev_id[19];
    device_name[len++] = 0;

    return len;
}

static I8U g_TwentyCharDevID[32];

I8U *SYSTEM_set_dev_id()
{
    return g_TwentyCharDevID;
}

void SYSTEM_get_dev_id(I8U *twentyCharDevID)
{
#if 1
    memcpy(twentyCharDevID, g_TwentyCharDevID, 20);
#else
    I8U v8;
    I16U i;
    I8U *dev_data;
    I32U dev_info[2];
#ifndef _CLING_PC_SIMULATION_
    dev_info[0] = 0x55667788;
    dev_info[1] = 0x11223344;
#endif
    // for the 20 character device ID, we use the first 2 bytes, then skip 4B, then take the next 8B
    dev_data = (I8U *)dev_info;

    twentyCharDevID[0] = 'H'; // Hi
    twentyCharDevID[1] = 'I'; // Hi
    twentyCharDevID[2] = 'C'; // Cling
#ifdef _CLINGBAND_RAINFLOWER_MODEL_
    twentyCharDevID[3] = 'R'; // UV Band
#endif

    for (i = 0; i < 8; i++) {
        v8 = (dev_data[i] >> 4) & 0x0f;
        v8 += (v8 > 9) ? 'A' - 0x0a : '0';
        twentyCharDevID[((i) << 1) + 4] = v8;

        v8 = (dev_data[i]) & 0x0f;
        v8 += (v8 > 9) ? 'A' - 0x0a : '0';
        twentyCharDevID[(((i) << 1) + 1) + 4] = v8;
    }
#endif
}
#ifdef _ENABLE_ANCS_

static void _notification_msg_init()
{
    I32U dw_buf[16];
    I8U *p_byte_addr;
    I32U offset = 0;

    // If this device is unauthorized, wipe out everything in the critical info section
    if (!LINK_is_authorized()) {
        cling.ancs.message_total = 0;
#ifdef _FLASH_ENABLED_
        FLASH_erase_App(SYSTEM_NOTIFICATION_SPACE_START);
#endif
        Y_SPRINTF("[SYSTEM] message: 0 (unauthorized)");
        return;
    }

    p_byte_addr = (I8U *)dw_buf;
    cling.ancs.message_total = 0;
    while (offset < FLASH_ERASE_BLK_SIZE) {
#ifdef _FLASH_ENABLED_
        // Read out the first 4 bytes -
        FLASH_Read_App(SYSTEM_NOTIFICATION_SPACE_START+offset, p_byte_addr, 4);
#endif
        // Every message, we store 256 bytes, which includes message title and length info.
        if (p_byte_addr[0] == 0xFF) {
            break;
        }

        if (p_byte_addr[0] >= 128) {
            Y_SPRINTF("[SYSTEM] message (%d) wrong info - erased: %08x!", offset, dw_buf[0]);
            // something wrong
            cling.ancs.message_total = 0;
#ifdef _FLASH_ENABLED_
            FLASH_erase_App(SYSTEM_NOTIFICATION_SPACE_START);
#endif
            break;
        }
        offset += 256;
        cling.ancs.message_total ++;

        N_SPRINTF("[SYSTEM] message buffer update(%d) - %d, %d",
                  cling.ancs.message_total, p_byte_addr[0], p_byte_addr[1]);

    }

    // If nothing gets restored from system
    if (cling.ancs.message_total == 16) {
        Y_SPRINTF("[SYSTEM] message buffer full - erased!");
        cling.ancs.message_total = 0;
#ifdef _FLASH_ENABLED_
        FLASH_erase_App(SYSTEM_NOTIFICATION_SPACE_START);
#endif
    }

    Y_SPRINTF("[SYSTEM] message OVERALL: %d", cling.ancs.message_total);

}
#endif

static BOOLEAN _critical_info_restored()
{
#if 0
    return TRUE;
#else
    // We should add system restoration for critical info
    CLING_TIME_CTX *t = &cling.time;
    TRACKING_CTX *a = &cling.activity;
    I32U dw_buf[16];
    I8U *p_byte_addr;
    I32U offset = 0;

    // If this device is unauthorized, wipe out everything in the critical info section
    if (!LINK_is_authorized()) {

        // Make sure the minute file has correct offset
        a->tracking_flash_offset = 0;
        a->day_stored.calories = a->day.calories;
        a->day_stored.distance = a->day.distance;
        a->day_stored.running = a->day.running;
        a->day_stored.walking = a->day.walking;

        // Get sleep by noon from flash
        a->sleep_by_noon = 0;
        a->sleep_stored_by_noon = 0;
        return FALSE;
    }

    p_byte_addr = (I8U *)dw_buf;

    while (offset < FLASH_ERASE_BLK_SIZE) {
#ifdef _FLASH_ENABLED_
        // Read out the first 4 bytes -
        FLASH_Read_App(SYSTEM_INFORMATION_SPACE_START+offset, p_byte_addr, 4);
#endif
        // Every minute, we store 64 bytes, each 64 bytes is seperated as a block,
        // which saves info, including UTC time, time zone, activity info.
        if (p_byte_addr[0] == 0xFF) {
            break;
        }
        offset += 64;
    }

    // If nothing gets restored from system
    if (offset == 0) {
        cling.batt.b_no_batt_restored = TRUE;
        Y_SPRINTF("[SYSTEM] Nothing gets restored");
        return FALSE;
    }

    // If we do have something restored from system, good, initialize all the parameters that should be initialized.
    offset -= 64;
#ifdef _FLASH_ENABLED_
    // Read out the first 4 bytes -
    FLASH_Read_App(SYSTEM_INFORMATION_SPACE_START+offset, p_byte_addr, 64);
#endif
    // Epoch time
    t->time_since_1970 = dw_buf[0];

    // add 30 seconds to correct bias of time when system rebooting..
    t->time_since_1970 += 30;

    // Bytes: 5 - 49: device related parameters,
    // length: p_byte_addr[4]
    //
    if ((p_byte_addr[4] >= 22) && (p_byte_addr[4] <= 39)) {
        Y_SPRINTF("[SYSTEM] critical restored device info: %d", p_byte_addr[4]);
        USER_setup_device(p_byte_addr+5, p_byte_addr[4]);
    }

    // Restore weather info
    cling.weather[0].day = p_byte_addr[44];
    cling.weather[0].month = p_byte_addr[45];
    cling.weather[0].type = p_byte_addr[46];
    cling.weather[0].high_temperature = p_byte_addr[47];
    cling.weather[0].low_temperature = p_byte_addr[48];

    // Restoring the time zone info
    t->time_zone = p_byte_addr[49];

    RTC_get_local_clock(cling.time.time_since_1970, &cling.time.local);

    // Get current local minute
    cling.time.local_day = cling.time.local.day;
    cling.time.local_minute = cling.time.local.minute;

    // Make sure the minute file has correct offset
    a->tracking_flash_offset = TRACKING_get_daily_total(&a->day);

    Y_SPRINTF("-- tracking offset (critical) ---: %d @ %d", a->tracking_flash_offset, cling.time.time_since_1970);

    // Update stored total
    a->day_stored.walking = a->day.walking;
    a->day_stored.running = a->day.running;
    a->day_stored.distance = a->day.distance;
    a->day_stored.calories = a->day.calories;

    // Get sleep seconds by noon
    a->sleep_by_noon = TRACKING_get_sleep_by_noon(FALSE);
    a->sleep_stored_by_noon = TRACKING_get_sleep_by_noon(TRUE);

    // 50 reserve

    // Restoring amount of reminders
    cling.reminder.total = p_byte_addr[51];
#ifdef _ENABLE_ANCS_
    // Restore ancs supported set.
    cling.ancs.supported_categories = p_byte_addr[52];
    cling.ancs.supported_categories = (cling.ancs.supported_categories << 8) | p_byte_addr[53];

    Y_SPRINTF("[SYSTEM] restored ANCS categories: %04x", cling.ancs.supported_categories);
#endif
#ifdef _ENABLE_ANCS_
    // Restore Skin touch
    cling.touch.b_skin_touch = p_byte_addr[54];

    // 55: clock orientation.
    cling.ui.clock_orientation = p_byte_addr[55];
#endif
    // Restore battery level
    if (p_byte_addr[56] == 0)
        cling.batt.b_no_batt_restored = TRUE;
    else if (p_byte_addr[56] > 100)
        cling.batt.b_no_batt_restored = TRUE;
    else
        cling.system.mcu_reg[REGISTER_MCU_BATTERY] = p_byte_addr[56];

    Y_SPRINTF("[SYSTEM] restore battery at: %d (%d)", p_byte_addr[56], cling.time.time_since_1970);

    cling.batt.non_charging_accumulated_active_sec = p_byte_addr[57];
    cling.batt.non_charging_accumulated_steps = p_byte_addr[58];
    Y_SPRINTF("[SYSTEM] restore charging param: %d, %d", p_byte_addr[57], p_byte_addr[58]);
#ifdef _ENABLE_ANCS_
    cling.ui.fonts_type = p_byte_addr[59];
#endif
    cling.gcp.host_type = p_byte_addr[60];
#ifdef _ENABLE_ANCS_
    // Restore sleep state
    cling.sleep.state = (SLEEP_STATUSCODE)p_byte_addr[61];
#endif
    // Minute file critical timing info
    cling.system.reset_count = p_byte_addr[62];
    cling.system.reset_count = (cling.system.reset_count << 8) | p_byte_addr[63];

    if (cling.system.reset_count & 0x8000) {
        cling.system.reset_count = 0;
    } else {
        cling.system.reset_count ++;
    }

    // Always erase this segment
    FLASH_erase_App(SYSTEM_INFORMATION_SPACE_START);

    // Add latency right before backup critical info (Erasure latency: 50 ms)
    BASE_delay_msec(50);

    // Initialize INFO-D section with the new time
    SYSTEM_backup_critical();

//        if(cling.system.reset_count%2 == 0){
//                hw_cpm_dcdc_on();
//                REG_CLR_BIT(CRG_TOP, LDO_CTRL2_REG, LDO_3V3_ON);
//                hw_cpm_reboot_system();
//        }
    return TRUE;
#endif
}

#ifdef _ENABLE_UART_
static void _print_out_dev_name()
{
    char dev_name[21];
    memset(dev_name, 0, 21);
    SYSTEM_get_dev_id(dev_name);
    Y_SPRINTF("####  system reboot: %s", dev_name);
}

static void _print_out_dev_version()
{
    I16U major;
    I16U minor;

    major = cling.system.mcu_reg[REGISTER_MCU_REVH] >> 4;
    minor = cling.system.mcu_reg[REGISTER_MCU_REVH] & 0x0f;
    minor <<= 8;
    minor |= cling.system.mcu_reg[REGISTER_MCU_REVL];
    Y_SPRINTF("#### ver: %d.%d ", major, minor);

}
#endif
static void _startup_logging()
{
#ifdef _ENABLE_UART_
    Y_SPRINTF("  ");
    Y_SPRINTF("####  ");
    _print_out_dev_name();
    Y_SPRINTF("####  ");
    _print_out_dev_version();
    Y_SPRINTF("####  ");
    Y_SPRINTF("[MAIN] WHO AM I -------- ");
    Y_SPRINTF("acc: 0x%02x, ", cling.whoami.accelerometer);
    Y_SPRINTF("issp: 0x%02x, ", cling.whoami.hssp);
    Y_SPRINTF("nor flash:0x%02x%02x, ", cling.whoami.nor[0], cling.whoami.nor[1]);
    Y_SPRINTF("Touch dev: %d.%d.%d", cling.whoami.touch_ver[0], cling.whoami.touch_ver[1], cling.whoami.touch_ver[2]);
    Y_SPRINTF("-----------------------");
    Y_SPRINTF("[MAIN] SYSTEM init ...");
#endif
}

extern void SFE_touch_key_init(void);
extern void CLING_global_init(void);
#include "ble_gap_mgr.h"
void SYSTEM_CLING_init(TASK_HANDLE_SETS *sets)
{
#if 0 // Enable it if wiping off everything on the Flash
    FLASH_erase_all(FALSE);
#endif
//    task_handles =sets;

//    CLASS(GapMgr)* gap_mgr = GapMgr_get_instance();
//    size_t dev_id_length;
//    gap_mgr->get_dev_id_str(gap_mgr, SYSTEM_set_dev_id(), &dev_id_length);
//void pin_map_init(void);
    CLING_global_init();
		pin_map_init();
		BATT_charging_init();
    RTC_Init();

//    // Accelerometer init
    SENSOR_init();

//    // Touch key init
//    HOMEKEY_click_init();

//    // GPIO interrupt init
    SFE_touch_key_init();
		NFLASH_init();
    // Logging when system boots up
    _startup_logging();
    

    // Check whether this is a authenticated device
    LINK_init();

    // If our FAT file system is NOT mounted, please erase NOR flash, and create it
    if (!FAT_chk_mounted()) {
        // Erase Nor Flash if device is not authorized or File system undetected
        if (LINK_is_authorized()) {
            FLASH_erase_all(FALSE);
            // Print out the amount of page that gets erased
            Y_SPRINTF("[MAIN] No FAT, With Auth ");
        } else {
            FLASH_erase_all(TRUE);

            // Print out the amount of page that gets erased
            Y_SPRINTF("[MAIN] No FAT, No Auth");
        }

        // Extra latency before initializing file system (Erasure latency: 50 ms)
        BASE_delay_msec(50);

        // File system initialization
        // clear FAT and root directory
        FAT_clear_fat_and_root_dir();

    } else if (!LINK_is_authorized()) {
        FLASH_erase_application_data(TRUE);
        // Extra latency before initializing file system (Erasure latency: 50 ms)
        BASE_delay_msec(50);
        // Print out the amount of page that gets erased
        Y_SPRINTF("[MAIN] YES FAT, No AUTH");
    } else {
        Y_SPRINTF("[MAIN] YES FAT, YES AUTH");
    }

    // Init the file system
    FILE_init();

    // Initialize user profile (name, weight, height, stride length)
    USER_data_init();

    // Restore critical system information, rtc and others
    // If nothing got stored before, this is an unauthorized device, let's initialize time
    //
    _critical_info_restored();

#ifdef _ENABLE_ANCS_
    // Initialize smart notification messages
    _notification_msg_init();
#endif

    // Over-the-air update check
//        OTA_main();
#ifndef _CLING_PC_SIMULATION_

    // Watchdog init, and disabled for debugging purpose (Rainflower has
    // Watchdog_Init();
#endif

    // Initialize alarm
    //REMINDER_alarm_init();

    // Start first battery measurement
    //
    BATT_init();

    //TRACKING_initialization();
    PEDO_set_step_detection_sensitivity(TRUE);

    // Initialize  UI rendering state machine
    UI_init();

    N_SPRINTF("[ClingInit]: task initialization is done!");

}

BOOLEAN SYSTEM_get_mutex(I8U value)
{
    BOOLEAN b_mutex_obtained = FALSE;
    if (MUTEX_NOLOCK_VALUE == cling.system.mcu_reg[REGISTER_MCU_MUTEX]) {
        cling.system.mcu_reg[REGISTER_MCU_MUTEX] = value;
        b_mutex_obtained = TRUE;
    } else {
        if (value == cling.system.mcu_reg[REGISTER_MCU_MUTEX]) {
            b_mutex_obtained = TRUE;
        }
    }
    return b_mutex_obtained;
}

BOOLEAN SYSTEM_backup_critical()
{
    // We should add system restoration for critical info
    CLING_TIME_CTX *t = &cling.time;
    I32U cbuf[16];
    I8U *critical = (I8U *)cbuf;
    I32U offset = 0;

    while (offset < FLASH_ERASE_BLK_SIZE) {
        // Read out the first 4 bytes -
        FLASH_Read_App(SYSTEM_INFORMATION_SPACE_START + offset, critical, 4);

        // Every minute, we store 64 bytes, each 64 bytes is seperated as a block,
        // which saves info, including UTC time, time zone, activity info.
        if (critical[0] == 0xFF) {
            break;
        }
        offset += 64;
    }

    if (offset >= FLASH_ERASE_BLK_SIZE) {
        N_SPRINTF("[FLASH] erase critical space as it gets full");

        FLASH_erase_App(SYSTEM_INFORMATION_SPACE_START);
        // add latency before backing up critical info (Erasure latency: 50 ms)
//                BASE_delay_msec(50);
        offset = 0;
    }

    // Reset critical buffer
    memset(critical, 0, 64);

    // Store time
    cbuf[0] = t->time_since_1970;

    // Backup device parameters
    USER_store_device_param(critical + 4);

    // Store weather info
    critical[44] = cling.weather[0].day;
    critical[45] = cling.weather[0].month;
    critical[46] = cling.weather[0].type;
    critical[47] = cling.weather[0].high_temperature;
    critical[48] = cling.weather[0].low_temperature;

    // Store time zone info to prevent unexpected day rollover
    critical[49] = t->time_zone;

    // 50 reserve

    // Store total reminders
    critical[51] = cling.reminder.total;
#ifdef _ENABLE_ANCS_
    // Store ancs supported set.
    critical[52] = (I8U)((cling.ancs.supported_categories>>8) & 0xFF);
    critical[53] = (I8U)(cling.ancs.supported_categories & 0xFF);
#endif

#ifdef REPLANT_SUCESS
    // Store skin touch type
    critical[54] = cling.touch.b_skin_touch;

    // 55: Clock orientation
    critical[55] = cling.ui.clock_orientation;
#endif
    // Store battery level
    critical[56] = cling.system.mcu_reg[REGISTER_MCU_BATTERY];
    if (critical[56] == 0) {
        critical[56] = 1;
    }

    Y_SPRINTF("[SYSTEM] backup battery at: %d", critical[56]);

    critical[57] = cling.batt.non_charging_accumulated_active_sec;
    critical[58] = cling.batt.non_charging_accumulated_steps;
#ifdef REPLANT_SUCESS
    critical[59] = cling.ui.fonts_type;
#endif
    N_SPRINTF("[SYSTEM] Backup charging param: %d, %d", cling.batt.non_charging_accumulated_active_sec,
              cling.batt.non_charging_accumulated_steps);

    // Store the  phone type
    critical[60] = cling.gcp.host_type;
#ifdef REPLANT_SUCESS
    critical[61] = cling.sleep.state;
#endif
    // Store the reset count
    critical[62] = (I8U)((cling.system.reset_count >> 8) & 0xFF);
    critical[63] = (I8U)((cling.system.reset_count) & 0xFF);

    // Put it in info flash area
    FLASH_Write_App(SYSTEM_INFORMATION_SPACE_START + offset, critical, 64);

    //
    N_SPRINTF("[SYSTEM] system backedup (%d)", cling.time.time_since_1970);

    return TRUE;
}

// System check sume
void SYSTEM_checksum(void *pdata, I32U size, I8U *checksum)
{
    I32U sum = 0;

    I8U *ptr = (I8U *)pdata;

    while (size-- > 0)
        sum += *ptr++;

    checksum[0] = (sum >> 24) & 0xff;
    checksum[1] = (sum >> 16) & 0xff;
    checksum[2] = (sum >> 8) & 0xff;
    checksum[3] = sum & 0xff;
}

void SYSTEM_factory_reset()
{
#if 1
    BLE_CTX *r = &cling.ble;

    Y_SPRINTF("[SYSTEM] factory reset - BLE disconnect");

    // Disconnect BLE service
    if (BTLE_is_connected()) {
        CLASS(GapMgr) *gap_mgr = GapMgr_get_instance();
        gap_mgr->disconnect(gap_mgr);
    }
//        BTLE_disconnect(BTLE_DISCONN_REASON_FACTORY_RESET);

    // Enable factory reset
    r->disconnect_evt |= BLE_DISCONN_EVT_FACTORY_RESET;
#endif

    Y_SPRINTF("[BTLE] disconn event: factory reset");

    // De-authorize
    LINK_deauthorize();

    // Add latency before storing messages (Erasure latency: 50 ms)
    BASE_delay_msec(50);

    // System format disk along with all data erasure
    SYSTEM_format_disk(TRUE);

    // Wipe out all user data
    memset(&cling.user_data, 0, sizeof(USER_DATA));

    // Jumpt to reset vector
    // SYSTEM_restart_from_reset_vector();

    // Let's turn on device to signal instead
    Y_SPRINTF("SYSTEM:  format & restart (not yet) ----");
    // Turn on OLED panel
    OLED_set_panel_on();
    cling.ui.true_display = TRUE;
    // Update touch event time
    cling.ui.touch_time_stamp = CLK_get_system_time();
    // UI initial state, a glance of current clock
    UI_switch_state(UI_STATE_CLOCK_GLANCE, 0);
    if(task_handles != NULL) {
        if(task_handles->ble_handle != NULL) {
            OS_TASK_NOTIFY(task_handles->ble_handle, BLE_APP_NOTIFY_FSM_MASK, eSetBits);
        }
    }
    // clear factory reset flag
//                r->disconnect_evt &= ~BLE_DISCONN_EVT_FACTORY_RESET;

}

void SYSTEM_reboot()
{
    Y_SPRINTF("[SYSTEM] rebooting ...");
    CLASS(GapMgr) *gap_mgr = GapMgr_get_instance();
    gap_mgr->reboot_on_disconnect(gap_mgr);
}

void SYSTEM_format_disk(BOOLEAN b_erase_data)
{
#if 1
    Y_SPRINTF("[FS] system format disk (all FLASH) ....");
    I16U page_erased   __attribute__((unused)) = FLASH_erase_all(b_erase_data);

    Y_SPRINTF("[FS] Clear fat and root dir ....");

    // File system initialization
    // clear FAT and root directory
    FAT_clear_fat_and_root_dir();

    // Before formating disk, adding some latency (Erasure latency: 50 ms)
    BASE_delay_msec(50);

    Y_SPRINTF("[SYSTEM] Initialize FILE sys");

    // Re-initialize file system and user profile & milestone
    FILE_init();

#endif
}

void SYSTEM_restart_from_reset_vector()
{
#if 1
    Y_SPRINTF("[SYSTEM] restarting system");

    if (LINK_is_authorized()) {
        // For an authorized device
        // Backup system critical information before reboot
        SYSTEM_backup_critical();
    }
#ifndef _CLING_PC_SIMULATION_

    // Reboot system.
//        sd_nvic_SystemReset();
    SYSTEM_reboot();
#endif
#endif
}

void SYSTEM_release_mutex(I8U value)
{
    if (value == cling.system.mcu_reg[REGISTER_MCU_MUTEX]) {
        cling.system.mcu_reg[REGISTER_MCU_MUTEX] = MUTEX_NOLOCK_VALUE;
    }
}

