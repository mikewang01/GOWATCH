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
#ifdef __cplusplus
extern "C" {
#endif
#include "pinmap.h"
/*********************************************************************
 * MACROS
 */

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

/*********************************************************************
 * LOCAL DECLARATION
 */
/*
 * @brief System Initialization and creation of the BLE task

 */
void cling_task_ui(void *pvParameters)
{

        struct task_handle_sets *p   __attribute__((unused)) = (struct task_handle_sets *)(pvParameters);
        uint32_t notif_value   __attribute__((unused)) = 0;
        while (1) {
//                Bits in this RTOS task's notification value are set by the notifying
//                 tasks and interrupts to indicate which events have occurred. */
//                OS_TASK_NOTIFY_WAIT(0x00, 0xffffffff, notif_value, OS_TASK_NOTIFY_FOREVER); /* Block indefinitely. */
                Y_SPRINTF("hello hicling");
//                GPIO_UART2_TX
                pin_toggle(GPIO_TEST_PIN);
                OS_DELAY(OS_MS_2_TICKS(5000));
        }
        /* the work of the SysInit task is done */
        OS_TASK_DELETE(OS_GET_CURRENT_TASK());
}

#ifdef __cplusplus
}
#endif
