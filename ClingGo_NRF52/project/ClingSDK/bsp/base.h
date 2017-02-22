//
//  File: base.h
//
//  Description: CLING firmware basics
//
//  Created on Mar 3, 2014
//
#ifndef _BASE_HEADER_
#define _BASE_HEADER_

#include "standard_types.h"

// Define all debugging message here

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

I32S BASE_abs(I32S a);
I32S BASE_sqrt(I32S a);
I8U BASE_calculate_occurance(I32U in, I8U size);
// get local time in a precision of 1 ms.
I32U CLK_get_system_time(void);

void BASE_delay_msec(I32U uiMSec);

#endif
