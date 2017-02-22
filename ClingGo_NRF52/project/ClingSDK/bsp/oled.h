/***************************************************************************//**
 * File oled.h
 * 
 * Description: OLED driver header
 *
 *
 ******************************************************************************/

#ifndef __OLED_HEADER__
#define __OLED_HEADER__
#include "standard_types.h"

// delay from power on is 100ms.  Set to 103 because time is slightly less than 1ms.

#define OLED_POWER_START_DELAY_TIME   10
#define OLED_RESET_INIT_LATENCY       103
#define OLED_POWER_OFF_DELAY_TIME     103
#define OLED_UI_START_LATENCY         100

typedef enum {
	OLED_STATE_IDLE,
	OLED_STATE_APPLYING_POWER,
	OLED_STATE_RESET_OLED, 	    // power on but not reset.  Need to be here for 100ms
	OLED_STATE_INIT_REGISTERS,	// waited time for power to be stable, reset driven high; init regs
	OLED_STATE_INIT_UI,
	OLED_STATE_ON, 				      // at this point, all display registers have been initialized
	OLED_STATE_GOING_OFF,			  // state when we have switched off but before actually off
													// 		any attempt to turn back on when going off adds extra time.
	OLED_STATE_OFF,			        // all power off and reset low
} OLED_STATE_TYPE;

typedef enum {
        OLED_FULL_OFF,
        OLED_FULL_ON,
        OLED_IMG_SHOW
} OLED_DISPLAY_MODE;

enum {
        OLED_MODULE_TURNED_OFF,
        OLED_MODULE_TURNED_ON,
} ;

typedef struct tagCLING_OLED_CTX {
	I8U state;
	BOOLEAN module_state;
} CLING_OLED_CTX;

void OLED_set_contrast(I8U step);
void OLED_set_display(I8U on_off);
int OLED_state_machine(void);
BOOLEAN OLED_set_panel_on(void);
void OLED_set_panel_off(void);
void OLED_im_show(OLED_DISPLAY_MODE mode, I8U *pram, I8U offset);
void OLED_full_scree_show(I8U *pram);
void OLED_power_off(void);
BOOLEAN OLED_is_panel_idle(void);

#endif
