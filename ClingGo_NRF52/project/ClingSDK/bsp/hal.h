//
//  File: hal.h
//  
//  Description: Hardware abstruct layer header
//
//  Created on Feb 26, 2014
//
#ifndef __HAL_HEADER__
#define __HAL_HEADER__

#include "standard_types.h"

#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */

#define FLASH_PAGE_SYS_ATTR                 (BLE_FLASH_PAGE_END - 3)                    /**< Flash page used for bond manager system attribute information. */
#define FLASH_PAGE_BOND                     (BLE_FLASH_PAGE_END - 1)                    /**< Flash page used for bond manager bonding information. */

#define DEAD_BEEF                            0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

void HAL_init(void);
void HAL_device_manager_init(BOOLEAN b_delete);
void HAL_advertising_start(void);
BOOLEAN HAL_set_conn_params(BOOLEAN b_fast);
BOOLEAN HAL_set_slow_conn_params(const uint8_t swith_purpose);
static inline void HAL_disconnect_for_fast_connection(const uint8_t swith_purpose){

}
void HAL_start_ancs_service_discovery(void);
#endif
