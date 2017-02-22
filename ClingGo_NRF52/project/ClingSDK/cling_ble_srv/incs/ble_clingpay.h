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

#ifndef _BLE_CLING_PAY_H_

#define _BLE_CLING_PAY_H_

#include <stdint.h>
#include "ble_service.h"
#include <btle_api.h>
/**
 * Register cling Service instance
 *
 *
 * \param [in] config   service configuration
 *
 * \return service instance
 *
 */
ble_service_t *clingpay_service_init(const ble_service_config_t *cfg, TASK_HANDLE_SETS *sets);

/**
 * Register cling Service write call back function
 *
 *
 * \param [in] p: callback funtion pointer passed from user
 *
 * \return ssucess failed
 *
 */
int clingpay_service_reg_write_callback(int (*p)(ble_service_t *svc, uint16_t conn_idx, uint16_t uuid_num, uint8_t *s, size_t size));
/**
 * Register cling Service data send function
 *
 *
 * \param [in] p: svc: service pointer conn_idx:connection id uuid_num uuid
 *
 * \return ssucess failed
 *
 */
ble_error_t clingpay_cus_notify_send(ble_service_t *svc, uint16_t conn_idx, uint16_t uuid_num, uint8_t *t, size_t size);

int clingpay_service_reg_disconn_callback(int (*p)());
int clingpay_service_reg_conn_callback(int (*p)(ble_service_t*, uint16_t));
#endif /* _BLE_CLING_PAY_H_ */
