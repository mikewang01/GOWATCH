/***************************************************************************/
/**
 * File: weather.c
 * 
 * Description: weather information processing
 *
 * Created on May 15, 2014
 *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>


#include "main.h"

void WEATHER_set_weather(I8U *data)
{
	WEATHER_CTX *w = cling.weather;
	
	memcpy(&w[0], data, 5*sizeof(WEATHER_CTX));
	
	Y_SPRINTF("[WEATHR] %d, %d, %d, %d, %d", w[0].month, w[0].day, w[0].type, w[0].low_temperature, w[0].high_temperature);
	Y_SPRINTF("[WEATHR] %d, %d, %d, %d, %d", w[1].month, w[1].day, w[1].type, w[1].low_temperature, w[1].high_temperature);
	Y_SPRINTF("[WEATHR] %d, %d, %d, %d, %d", w[2].month, w[2].day, w[2].type, w[2].low_temperature, w[2].high_temperature);
	Y_SPRINTF("[WEATHR] %d, %d, %d, %d, %d", w[3].month, w[3].day, w[3].type, w[3].low_temperature, w[3].high_temperature);
	Y_SPRINTF("[WEATHR] %d, %d, %d, %d, %d", w[4].month, w[4].day, w[4].type, w[4].low_temperature, w[4].high_temperature);
}

BOOLEAN WEATHER_get_weather(I8U index, WEATHER_CTX *wo)
{
	WEATHER_CTX *wi = cling.weather;
	SYSTIME_CTX new_time;
	BOOLEAN b_available = FALSE;
	I8U i;
	
	RTC_get_delta_clock_forward(&new_time, index);
	
	for (i = 0; i < MAX_WEATHER_DAYS; i++) {
		if ((wi[i].day == new_time.day) && (wi[i].month == new_time.month)) {
			memcpy(wo, &wi[i], sizeof(WEATHER_CTX));
			b_available = TRUE;
			break;
		}
	}
	
	if (b_available) return TRUE;
	
	// if no weather available, go return the first one
	RTC_get_delta_clock_forward(&new_time, 0);
	
	if ((wi[0].day == new_time.day) && (wi[0].month == new_time.month)) {
		memcpy(wo, &wi[0], sizeof(WEATHER_CTX));
		return TRUE;
	} else {
		wo->high_temperature = 0;
		wo->low_temperature = 0;
		wo->type = 0;
		return FALSE;
	}
}

