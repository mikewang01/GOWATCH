/***************************************************************************/
/**
* File:    ota.c
*
* Description: Over-the-air update
*
******************************************************************************/

#include "main.h"
#include "sysflash_rw.h"
#include "ota_hal.h"
#ifndef _CLING_PC_SIMULATION_
//#include "crc_abs.h"


static void _start_OTA()
{
    SYSTEM_restart_from_reset_vector();
}
#endif

void OTA_main()
{

    _start_OTA();

}

BOOLEAN OTA_if_enabled()
{
    if (cling.ota.b_update) {
#if 0
        // Switch to fast connection mode to exchange data (no need, just
        if (HAL_set_conn_params(TRUE)) {
            N_SPRINTF("OTA update with FAST Connection interval");
        }
#endif
        return TRUE;
    } else {
        return FALSE;
    }
}

void OTA_set_state(BOOLEAN b_enable)
{
#if 1
    // Set OTA flag
    cling.ota.b_update = b_enable;

    // Turn on OLED
    if (b_enable) {

        if (cling.system.mcu_reg[REGISTER_MCU_BATTERY] > 90) {

            // Turn on screen
            UI_turn_on_display(UI_STATE_TOUCH_SENSING, 0);
        }
    }
#endif		
}



