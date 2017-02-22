/***************************************************************************//**
 * File btle.h
 * 
 * Description: BLUETOOTH low energy header
 *
 *
 ******************************************************************************/

#ifndef __SENSOR_HEADER__
#define __SENSOR_HEADER__

#include "standards.h"

#include <string.h>

#define _USE_HW_MOTION_DETECTION_

void SENSOR_accel_processing(void);
void SENSOR_init(void);

#endif // __SENSOR_HEADER__
/** @} */
