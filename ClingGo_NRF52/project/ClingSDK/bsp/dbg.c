/***************************************************************************/
/**
 * File: dbg.c
 * 
 * Description: All debugging code comes in here
 *
 * Created on Feb 26, 2014
 *
 ******************************************************************************/
#include "main.h"


void DEBUG_create_dbg_msg(I8U *msg, I8U len)
{

}

enum {
	ENABLE_OTA_DEBUG,
	TOUCH_OTA_READ,
	TOUCH_OTA_WRITE,
};

#if defined(_ENABLE_BLE_DEBUG_) || defined(_ENABLE_UART_)

void DBG_event_processing()
{

}

#endif
