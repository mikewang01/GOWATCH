/**
 ****************************************************************************************
 *
 * @file ble_cus.h
 *
 * @brief hicling service implementation API
 *
 * Copyright (C) 2016. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#ifndef _BLE_CLING_H_

#define _BLE_CLING_H_

#include "ble_common.h"
#include <stdint.h>
//#include "ble_service.h"
#include <btle_api.h>
#define ble_service_t int
#define ble_service_config_t int 
/**
 * Register cling Service instance
 *
 *
 * \param [in] config   service configuration
 *
 * \return service instance
 *
 */
int *cling_service_init(const ble_service_config_t *cfg, TASK_HANDLE_SETS *sets);

/**
 * Register cling Service write call back function
 *
 *
 * \param [in] p: callback funtion pointer passed from user
 *
 * \return ssucess failed
 *
 */
int cling_service_reg_write_callback(int (*p)(ble_service_t *svc, uint16_t conn_idx, uint16_t uuid_num, uint8_t *s, size_t size));
/**
 * Register cling Service data send function
 *
 *
 * \param [in] p: svc: service pointer conn_idx:connection id uuid_num uuid
 *
 * \return ssucess failed
 *
 */
int cling_notify_send(uint16_t conn_idx, uint16_t uuid_num, uint8_t *t, size_t size);

extern int cling_service_reg_disconn_callback(int (*p)(void));
extern int cling_service_reg_conn_callback(int (*p)(ble_service_t *svc, uint16_t conn_idx));
#endif /* _BLE_CLING_H_ */
