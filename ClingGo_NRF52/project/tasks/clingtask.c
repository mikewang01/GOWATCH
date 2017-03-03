/******************************************************************************
 * Copyright 2013-2016 hicling Systems (MikeWang)
 *
 * FileName: clingtask.c
 *
 * Description: cling task in  freertos.
 *
 * Modification history:
 *     2016/7/23, v1.0 create this file mike.
 *******************************************************************************/
#include "main.h"
#include "osal.h"
#include "RTC.h"
#include "flash_hal.h"
#include "clingtask.h"
#include "btle_api.h"
#include "flatform_dev.h"
#include "nrf_nvmc.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "pinmap.h"
/*********************************************************************
 * MACROS
 */
/* BLE peripheral task. */
#define BLE_PERIPHERAL_TASK_STACKSIZE                 ( 512 * OS_STACK_WORD_SIZE )
#define BLE_PERIPHERAL_TASK_PRIORITY                  ( OS_TASK_PRIORITY_LOWEST  )

/* Sensor Front End task
 *
 */
#define INPUT_TASK_STACKSIZE                          ( 200 * OS_STACK_WORD_SIZE )
#define INPUT_TASK_PRIORITY                           ( OS_TASK_PRIORITY_NORMAL  )

/* core fsm alert task */
#define CORE_FSM_TASK_STACKSIZE                       ( 400 * OS_STACK_WORD_SIZE )
#define CORE_FSM_TASK_PRIORITY                        ( OS_TASK_PRIORITY_HIGHEST  )

/* Cling Output task */
#define OUTPUT_TASK_STACKSIZE                         ( 400 * OS_STACK_WORD_SIZE )
#define OUTPUT_TASK_PRIORITY                          ( OS_TASK_PRIORITY_NORMAL  )

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */
CLING_MAIN_CTX cling;

/*********************************************************************
 * LOCAL VARIABLES
 */
static TASK_HANDLE_SETS task_handles;
/*********************************************************************
 * FUNCTION DECALARATION
 */
void ble_peripheral_task(void *params);

// all local static tasks
static void cling_task_core_fsm(void *pvParameters);
extern void cling_task_input(void *pvParameters);
extern void cling_task_output(void *pvParameters);

/*
 * @brief cling Initialization and creation of the BLE task
 * this is the entry task to all cling app

 */

void compare_task(uint32_t *data)
{
		char *p;
	  if(data == task_handles.ble_handle){
				Y_SPRINTF("ble task");
		}else if(data == task_handles.cling_core_fsm_handle){
					Y_SPRINTF("cling_core_fsm_handle task");
		}else if(data == task_handles.input_handle){
					Y_SPRINTF("task_handles.input_handle");
		}else if(data == task_handles.output_handle){
						Y_SPRINTF("task_handles.output_handle");
		}

}
void  oled_test();
void BATT_charging_init();
void nfc_test();
int fm151_test(void);
void cling_init_task(void *pvParameters)
{
    uint32_t nfcpins = (*(uint32_t *)0x1000120C);
    if (nfcpins & 1) {
        nrf_nvmc_write_word(0x1000120C, 0xFFFFFFFE);
        NVIC_SystemReset();
    } 
//		void PPG_init();
	//BATT_charging_init();
	 // nfc_test();
//	fm151_test();
//		pin_map_init();
//	PPG_init();
//		
		//oled_test();
	//	BATT_monitor_process();
 //_nor_flash_spi_test2();
    /*get cling task handle*/
    Y_SPRINTF("[ClingInit]: create BLE Peripheral task");

        // Sensor Front End - data acquisition
        // Accelerometer, Key, Touch panel, Battery, Optical sensor, Skin Temperature sensor, UV sensor
        Y_SPRINTF("[ClingInit]: Create Cling Input task");
        OS_TASK_CREATE("Cling Input Task", cling_task_input, &task_handles,
                INPUT_TASK_STACKSIZE, INPUT_TASK_PRIORITY, task_handles.input_handle); /* The task handle. */
        OS_ASSERT(task_handles.input_handle);
				
        // BLE peripheral task -
        Y_SPRINTF("[ClingInit]: Create Cling main task");
        OS_TASK_CREATE("Core Fsm", cling_task_core_fsm, &task_handles,
                CORE_FSM_TASK_STACKSIZE, CORE_FSM_TASK_PRIORITY, task_handles.cling_core_fsm_handle); /* The task handle. */
        OS_ASSERT(task_handles.cling_core_fsm_handle);

        Y_SPRINTF("[ClingInit]: Create Cling output Task");
        OS_TASK_CREATE("ClingOutput Task", cling_task_output, &task_handles,
                OUTPUT_TASK_STACKSIZE, OUTPUT_TASK_PRIORITY, task_handles.output_handle);
        OS_ASSERT(task_handles.output_handle);

    /* Start the BLE Peripheral application task. */
    OS_TASK_CREATE("BLEPeripheral", /* The text name assigned to the task, for debug only; not used by the kernel. */
                   ble_peripheral_task, /* The function that implements the task. */
                   &task_handles, /* The parameter passed to the task. */
                   BLE_PERIPHERAL_TASK_STACKSIZE, /* The number of bytes to allocate to the stack of the task. */
                   BLE_PERIPHERAL_TASK_PRIORITY,/* The priority assigned to the task. */
                   task_handles.ble_handle); /* The task handle. */
    OS_ASSERT(task_handles.ble_handle);
    SYSTEM_CLING_init(&task_handles);
    /* the work of the clingInit task is done */
    OS_TASK_DELETE(OS_GET_CURRENT_TASK());
//			while(1){
//					OS_DELAY(2000);
//			}
}

void CLING_global_init()
{
    // Clean the cling global structure
    memset(&cling, 0, sizeof(CLING_MAIN_CTX));

    // Initialize the low power variables
    cling.lps.b_low_power_mode = FALSE;

    // Version initialization
    cling.system.mcu_reg[REGISTER_MCU_HW_REV] = 0; // Board version is yet to be supported, initialized to 0.

    // BATTERY LEVEL
    cling.system.mcu_reg[REGISTER_MCU_BATTERY] = 50;

    // Software version number
    cling.system.mcu_reg[REGISTER_MCU_REVH] = BUILD_MAJOR_RELEASE_NUMBER;  // SW VER (HI)
    cling.system.mcu_reg[REGISTER_MCU_REVH] <<= 4;
    cling.system.mcu_reg[REGISTER_MCU_REVH] |= (BUILD_MINOR_RELEASE_NUMBER >> 8) & 0x0f;
    cling.system.mcu_reg[REGISTER_MCU_REVL] = (I8U)(BUILD_MINOR_RELEASE_NUMBER & 0xff); // SW VER (LO)

    // Restoring the time zone info
    // Shanghai: UTC +8 hours (units in 15 minutes) -> 8*(60/15) = 32 minutes
    cling.time.time_zone = 32;
    cling.time.time_since_1970 = 1400666400;
    RTC_get_local_clock(cling.time.time_since_1970, &cling.time.local);

    // Get current local minute
    cling.time.local_day = cling.time.local.day;
    cling.time.local_minute = cling.time.local.minute;

}

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL DECLARATION
 */
void _nor_flash_spi_test();

void CLING_test_notify()
{
    OS_BASE_TYPE xResult   __attribute__((unused));

    xResult = OS_TASK_NOTIFY(task_handles.input_handle, OS_TASK_SFE_KEY_INT_BITS, OS_NOTIFY_SET_BITS);
}

void CLING_task_rtc_notify()
{
    OS_TASK_NOTIFY(task_handles.cling_core_fsm_handle, OS_TASK_FSM_MINUTE_UPDATE_BITS, eSetBits);
}

void BLE_task_rtc_notify()
{
    OS_TASK_NOTIFY(task_handles.ble_handle, BLE_APP_NOTIFY_FSM_MASK, eSetBits);
}

/*
 * @brief System Initialization and creation of the BLE task

 */
static void cling_task_core_fsm(void *pvParameters)
{
    uint32_t notified_value = 0;
    OS_BASE_TYPE xResult   __attribute__((unused));

    while (1) {
        // Lets suspend the task as state machine runs in idle
        xResult = OS_TASK_NOTIFY_WAIT(0x0, OS_TASK_NOTIFY_ALL_BITS, &notified_value,
                                      OS_TASK_NOTIFY_FOREVER);

        /* Guaranteed to succeed since we're waiting forever for the notification */
        OS_ASSERT(xResult == OS_OK);

        Y_SPRINTF("CLING TASK: event: 0x%08x @ %d", notified_value, CLK_get_system_time());

        if (notified_value & OS_TASK_FSM_ACC_DATA_INT_BITS) {
            // It is a ACC data interrupt ... go ahead to perform sensor processing
            N_SPRINTF("CLING TASK:  fsm acc event");
        }

        // Process sensor data
         SENSOR_accel_processing();

        if (notified_value & OS_TASK_FSM_MINUTE_UPDATE_BITS) {

            if (RTC_timer_process()) {
                // For testing purpose -
                BATT_monitor_process();
			
                // User data save and store
                TRACKING_data_logging();
#if 0
                // Idle alert state update
                USER_idle_state_machine_update();

                // Check on reminder event
                if (REMINDER_check_reminder()) {
                    Y_SPRINTF("[REMINDER] reminder is hit @ %d:%d", cling.time.local.hour, cling.time.local.minute);
                    REMINDER_fire_up_alert();
                    OS_TASK_NOTIFY(task_handles.output_handle, OS_TASK_ALERT_REMINDER_BITS, OS_NOTIFY_SET_BITS);
                }
#endif
                // If device is unauthorized, we might shut it down in a period of time
                BATT_device_unauthorized_shut_down();
            }
        }

    }

    /* the work of the SysInit task is done */
    OS_TASK_DELETE(OS_GET_CURRENT_TASK());
}

#ifdef __cplusplus
}
#endif
