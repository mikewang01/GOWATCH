/***************************************************************************//**
 * 
 * File ota.h
 *
 * The header of over-the-air update
 *
 ******************************************************************************/

#ifndef __OTA_H__
#define __OTA_H__

#include "standards.h"

typedef struct tagOTA_CTX {
	// OTA file length
	I32U file_len;
	
	// Over the air update
	BOOLEAN b_update;
	
	// Update the percentage
	I8U percent;
	
} OTA_CTX;

void OTA_main(void);
BOOLEAN OTA_if_enabled(void);
void OTA_set_state(BOOLEAN b_enable);

#endif // __PPG_H__
/** @} */
