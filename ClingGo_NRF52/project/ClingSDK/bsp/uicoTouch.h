
#ifndef __UICOTOUCH_H__
#define __UICOTOUCH_H__

#include "standards.h"


typedef enum {
	UICO_POWER_FAST_SCAN,
	UICO_POWER_SLOW_SCAN,
	UICO_POWER_DEEP_SLEEP,
} UICO_POWER_MODE;

typedef struct tagUICOTOUCH_RESPONSE_CTX {
	//
	I8U type;
	I8U x;
	I32U ts;
} UICOTOUCH_RESPONSE_CTX;

// Time difference
void UICO_init(void);
I8U  UICO_main(void);
void UICO_set_power_mode(UICO_POWER_MODE mode);
void UICO_dbg_write_read(I8U *obuf, I8U len, I8U *ibuf);
/*start calibration api reference*/
bool uico_touch_ic_floating_calibration_start(void);
#endif
