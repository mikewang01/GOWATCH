
#ifndef __BUTTERWORTH_H__
#define __BUTTERWORTH_H__

#include "standards.h"

#define BUTTORD      5                   // butterworth low pass filter order
#define MAX_INT      2147483647          // 0x7FFFFFFF
#define MIN_INT     -2147483646          // 0x80000002

typedef struct tagBUTT_CTX {
	BOOLEAN firstElement_lp;
	BOOLEAN firstElement_hp;

	double x_lp[BUTTORD+1];
	double y_lp[BUTTORD+1];
	
	double x_hp[BUTTORD+1];
	double y_hp[BUTTORD+1];
	
	// kalman status
	I8U        pre_guess;
	I8U        post_guess;
	double     pre_error;
	double     post_error;
	double     gain;	
} BUTT_CTX;

void   Butterworth_Filter_Init(void);
double Butterworth_Filter_LP(double invar);
double Butterworth_Filter_HP(double invar);
I8U    Kalman_Filter(I8U sample_width);

#endif
