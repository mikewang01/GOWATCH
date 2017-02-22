/**
 ****************************************************************************************
 *
 * @file clingtask.h
 *
 * @brief hicling task service
 *
 * Copyright (C) 2016. hicling elec Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#ifndef CLING_TASK_H
#define CLING_TASK_H


#include <stdint.h>
#include "standards.h"
/**
 * Register cling TASK instance
 *
 *
 * \param [in] pvParameters   parameter passed to this task
 *
 * \return null
 *
 */

// Cling main task event bits
#define OS_TASK_FSM_MINUTE_UPDATE_BITS         0x0002
#define OS_TASK_FSM_ACC_DATA_INT_BITS          0x0010
#define OS_TASK_FSM_TEST_BITS                  0x0800

// Cling input (Sensor Front End) task
#define OS_TASK_SFE_KEY_INT_BITS               0x0001
#define OS_TASK_SFE_KEY_LONG_PRESSED_BITS      0x0002
#define OS_TASK_SFE_ACC_WATERMARK_INT_BITS     0x0010
#define OS_TASK_SFE_ACC_TIMER_INT_BITS         0x0020
#define OS_TASK_SFE_BATT_EVT_BITS              0x0100

//
// Cling output task related
//
// Cling alert task
//
#define OS_TASK_ALERT_INIT_BITS                0x0001
#define OS_TASK_ALERT_REMINDER_BITS            0x0002
#define OS_TASK_ALERT_NOTIFIC_BITS             0x0004
// UI rendering task
//
#define OS_TASK_UI_RENDER_FRAME_BITS           0x0010
#define OS_TASK_UI_OLED_PANEL_BITS             0x0020
// Payment (Z32) task
//
#define OS_TASK_PAYMENT_COMM_SEND_BITS         0x0100


#endif /* CLING_TASK_H */
