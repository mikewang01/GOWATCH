#include <battmgr_hal.h>
#include <btle_api.h>
/******************************************************************************
 * Copyright 2013-2016 hicling Systems (MikeWang)
 *
 * FileName: rtc_hal.c
 *
 * Description: abstract layer between cling sdk and freertos.
 *
 * Modification history:
 *     2016/7/22, v1.0 create this file mike.
 *******************************************************************************/
#include "oop_hal.h"
//#include "ad_flash.h"
#include "rtc_hal.h"
#include "osal.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "btle_api.h"
#include "flash_hal.h"
#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL DECLARATION
 */
/******************************************************************************
 * FunctionName :btle_fsm_init
 * Description  : init fsm state
 * Parameters   : write: write call back function
 * Returns          : 0: sucess
 -1: error
 *******************************************************************************/

void usb_charger_event_observer(usb_charger_event_t evt)
{
        N_SPRINTF("[batt mgr] test event %d arrived", (uint8_t) evt);
}


void CLING_global_init();


int btle_fsm_init(void)
{

        /*put system init down here for the pupose of getting all task handles*/
        /* Initialize Cling global variables */
        Y_SPRINTF("[ClingInit]: initialize system before waking up task loop");
        // Cling global initialization


        // System init for cling device
//        SYSTEM_CLING_init();

        //
        // Algorithms (core pedometer)
        //
        // -- activity detection and classification
        //
        // -- High sensitivity when system boots up
        //
        //
//        TRACKING_initialization();
//        PEDO_set_step_detection_sensitivity(TRUE);
//
//        // Initialize  UI rendering state machine
//        UI_init();
//
//        N_SPRINTF("[ClingInit]: task initialization is done!");
        return 0;
}

/******************************************************************************
 * FunctionName :btle_fsm_process
 * Description  : fsm
 * Parameters   : write: write call back function
 * Returns          : 0: sucess
 -1: error
 *******************************************************************************/
int btle_fsm_process()
{
        Y_SPRINTF("btle_fsm_process entered");
        BTLE_state_machine_update();

        // Communication Protocol (CP) update
        CP_state_machine_update();

        // Device authorization state update
        LINK_state_machine_update();

        return 0;
}

#ifdef __cplusplus
}
#endif
