/**
 ****************************************************************************************
 *
 * @file sys_power_mgr.c
 *
 * @brief Power Manager
 *
 * Copyright (C) 2015. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd. All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

/**
 \addtogroup BSP
 \{
 \addtogroup SYSTEM
 \{
 \addtogroup Clock-Power_Manager
 \{
 */

#include <stdint.h>
#include <string.h>
#include "stdio.h"
#include "sys_power_mgr.h"
#include "osal.h"
/* 
 * Global and / or retained variables
 */
static periph_init_cb periph_init = NULL;
static adapter_call_backs_t *pm_adapters_cb[dg_configPM_MAX_ADAPTERS_CNT];

void pm_system_init(periph_init_cb peripherals_initialization)
{
    int i __attribute__((unused));
    uint32_t rtc __attribute__((unused));
//        if (dg_configUSE_WDOG == 0) {
//                hw_watchdog_freeze();                   // Stop watchdog
//        }


		memset(pm_adapters_cb, 0, sizeof(pm_adapters_cb));
    if (periph_init != NULL) {
        periph_init();                          // Power on peripherals and GPIOs
    }


//        if (dg_configUSE_WDOG == 1) {
//                hw_watchdog_unfreeze();                 // Start watchdog
//        }
}


pm_id_t pm_register_adapter(const adapter_call_backs_t *cb)
{
    pm_id_t ret = -1;
    int i = 0;

    ASSERT_WARNING(cb != NULL);

    while ((i < dg_configPM_MAX_ADAPTERS_CNT) && (pm_adapters_cb[i] != NULL)) {
        i++;
    }

    if (i < dg_configPM_MAX_ADAPTERS_CNT) {
        pm_adapters_cb[i] = (adapter_call_backs_t *)cb;
        ret = i;
    }

    ASSERT_WARNING(ret != -1);                      // Increase dg_configPM_MAX_ADAPTERS_CNT
    return ret;
}

void pm_unregister_adapter(pm_id_t id)
{

    ASSERT_WARNING((id >= 0) && (id < dg_configPM_MAX_ADAPTERS_CNT));     // Is id valid?
    ASSERT_WARNING(pm_adapters_cb[id] != NULL);     // Is it registered?
    pm_adapters_cb[id] = NULL;
}


static void pm_init_wake_up(void)
{
    int i;
    uint32_t rtc __attribute__((unused));
    uint32_t iser __attribute__((unused));
    adapter_call_backs_t *p_Ad;
    // Inform Adapters
    for (i = 0; i < dg_configPM_MAX_ADAPTERS_CNT; i++) {
        p_Ad = pm_adapters_cb[i];
        if ((p_Ad != NULL) && (p_Ad->ad_wake_up_ind != NULL)) {
            p_Ad->ad_wake_up_ind(false);
        }
    }

    if (periph_init != NULL) {
        periph_init();                          // Power on peripherals and GPIOs
    }

}

int pm_sleep_enter()
{
	
	bool allow_entering_sleep = true;
    do {
        int i;
        adapter_call_backs_t *p_Ad;
        // 5. Calculate the overhead added from the Adapters.
            for (i = 0; i < dg_configPM_MAX_ADAPTERS_CNT; i++) {
                p_Ad = pm_adapters_cb[i];
                if (p_Ad != NULL) {
                   if(p_Ad->ad_prepare_for_sleep != NULL){
												if(p_Ad->ad_prepare_for_sleep() == false){
														/*if one preh refuse to enter sleep , reinitilize is needed here*/
														 pm_init_wake_up();
														 allow_entering_sleep = false;
															break;
												}
									 }
                }
            }
    } while (0);
		return allow_entering_sleep;
}

/**
 * \brief Initialize system after a wake-up.
 *
 * \return void
 *
 */
void pm_system_wake_up()
{
		pm_init_wake_up();
}


/**
 \}
 \}
 \}
 */
