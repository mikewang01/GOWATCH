//
//  File: main.h
//  
//  Description: the main header contains release info, and global variables
//
//  Created on Feb 26, 2014
//
#ifndef __MAIN_HEADER__
#define __MAIN_HEADER__

/* Generic macros (do not change across platforms) */
#define LOOP_FOREVER    1         /* makes while() statements more legible */

//#define _CLINGBAND_NFC_MODEL_

#define _CLINGBAND_RAINFLOWER_MODEL_

// 
// Global device id address
//
#define GLOBAL_DEVICE_ID_LEN 20

#include "stdint.h"
#include "stdbool.h"
//
//#include "standards.h"
#include "dbg.h"    // For dbg_warning function
#include "pin_names.h"
#include "pinmap.h"
#include "gpio_api.h"
//#include "gpio.h"
//#include "uart_api.h"
//#include "uart.h"
//#include "spi_master_api.h"
#include "system.h"
#include "base.h"
#include "lis3dh.h"
#include "RTC.h"
#include "sys_reg.h"
#include "ota.h"
//#include "nflash_spi.h"
//#include "systick.h"
#include "link.h"
//#include "hal.h"
#include "sensor.h"
#include "ppg.h"
#include "btle.h"
#include "touch.h"
#include "ui.h"
#include "oled.h"
#include "release_number.h"
#include "sysclk.h"
#include "batt.h"
//#include "butterworth.h"
//#if 0
#include "weather.h"
#include "sleep.h"
//#endif
#include "reminder.h"
#include "notific.h"
//#if 0
//#include "ancs.h"
//#endif
#include "homekey.h"
#include "tracking.h"
#include "fs_memcfg.h"
#include "fs_nflash_rw.h"
#include "fs_flash.h"
#include "fs_fat.h"
//#include "user.h"
#include "cp.h"
////#include "arch_console.h"
//// Add Dialog system header functions
//#include "adc.h"
#include "ble_common.h"
#include "aes_hal.h"

/*************freertos_realated*************/
#include "osal.h"
#include "clingtask.h"

enum {
        MUTEX_NOLOCK_VALUE = 0,
        MUTEX_IOS_LOCK_VALUE,
        MUTEX_MCU_LOCK_VALUE,
};

enum {
        PSYS_ERR_OK,
        PSYS_ERR_FS_OPEN_FAIL,
        PSYS_ERR_FS_CREATE_FILE_FAIL,
        PSYS_ERR_FS_WRONG_DATA_FAT,
        PSYS_ERR_FS_WRONG_NAME_FAT,
        PSYS_ERR_FS_WRONG_FCLOSE,
        PSYS_ERR_ALTIMETER_INIT_FAIL,
};

typedef struct tagCLING_TIME_CTX {
        // Time difference
        uint32_t time_since_1970;
        // Tick count
        uint32_t tick_count;
        I8S time_zone;
        uint8_t local_day;
        uint8_t local_minute;
        uint8_t local_hour;
        SYSTIME_CTX local;

        // System clock
        uint32_t system_clock_in_sec;	// Sourced from RTC

        // System clock interval
        bool operation_clk_enabled;
        uint32_t operation_clk_start_in_ms;

        //
        bool local_minute_updated;
        bool local_noon_updated;
        bool local_day_updated;
        bool local_hour_updated;

} CLING_TIME_CTX;

typedef struct tagLOW_POWER_STATIONARY_CTX {
        uint32_t ts;
        bool b_low_power_mode;
        bool b_accelerometer_fifo_mode;
        uint8_t int_count; // Interrupt counter
} LOW_POWER_STATIONARY_CTX;

typedef struct tagWHOAMI_CONTEXT {
        uint8_t accelerometer;
        uint8_t hssp;
        uint8_t nor[2];
        uint8_t touch_ver[3];
} WHOAMI_CTX;

typedef struct tagSYSTEM_CTX {
        // Connection parameter update
        uint32_t conn_params_update_ts;

        // Card balance
        uint32_t balance;

        // Reset count - overall reset time since the factory reset
        // The count only works for authorized device.
        uint16_t reset_count;

        // System power status
        bool b_powered_up;

        /// MCU Registers
        uint8_t mcu_reg[SYSTEM_REGISTER_MAX];

        // MCU peripheral requirements
        bool b_spi_ON;  // SPI 0 (dedicated for OLED, accelerometer, flash)
        bool b_twi_1_ON;  // TWI 1 (dedicated for UV, , TOUCH)
} SYSTEM_CTX;

typedef struct tagCLING_MAIN_CTX {

        // Cling system related
        SYSTEM_CTX system;

        // weather context (5 days)
        WEATHER_CTX weather[MAX_WEATHER_DAYS];

        // System diagonostic variables
        WHOAMI_CTX whoami;

        // Low power stationary mode
        LOW_POWER_STATIONARY_CTX lps;

        // Activity information including all daily total/minute, and on-going activity status.
        TRACKING_CTX activity;

        // Timing info
        CLING_TIME_CTX time;

        // Battery context
        BATT_CTX batt;

        // User related data
        USER_DATA user_data;

#if defined(_ENABLE_BLE_DEBUG_) || defined(_ENABLE_UART_)
        // Debug context
        DEBUG_CTX dbg;
#endif

        // Generic communication protocol
        CP_CTX gcp;

        // Pairing and authentication
        LINK_CTX link;

  // HEART RATE state machine
  HEARTRATE_CTX hr;
	
        // Touch panel context
        TOUCH_CTX touch;

        // Radio BLE state
        BLE_CTX ble;

#if 0
#ifdef _ENABLE_ANCS_
        // ANCS (apple notification center service)
        ANCS_CONTEXT ancs;
#endif
#endif
        // UI state
        UI_ANIMATION_CTX ui;

        // LED state machine
	CLING_OLED_CTX oled;

        // Over the air update
        OTA_CTX ota;

        // Sleep monitoring state context
        SLEEP_CTX sleep;

        // Reminder
        REMINDER_CTX reminder;
        // Notific
        NOTIFIC_CTX notific;

        // Homekey state
        HOMEKEY_CLICK_STAT key;

} CLING_MAIN_CTX;

// CLING main context
extern CLING_MAIN_CTX cling;

#endif
