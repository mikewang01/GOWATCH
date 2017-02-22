//
//  File: main.h
//  
//  Description: the main header contains release info, and global variables
//
//  Created on Feb 6, 2014
//
#ifndef __DEBUG_HEADER__
#define __DEBUG_HEADER__

#include "standard_types.h"
#include "stdio.h"
#define DBG_SWDIAG(bank , field , value)


typedef struct tagDEBUG_CTX {
	I8U b_log;
	I8U b_pending;
	I8U b_read;
	I8U b_write;
	I8U len;
	I8U buf[32];
} DEBUG_CTX;

#define SYSLOG_DEBUG_MESSAGE(...) \
	{ char buf[128]; \
		I8U len;\
		if (cling.dbg.b_log) {\
			len = sprintf(buf, __VA_ARGS__);\
			DEBUG_create_dbg_msg((I8U *)buf, len);\
		}\
	}

#define TIMED_DEBUG_LOG(fmt,...) do{\
        printf("%d-%d-%d, %d:%d:%d " fmt "\r\n",cling.time.local.year,\
                                cling.time.local.month,\
                                cling.time.local.day,\
                                cling.time.local.hour,\
                                cling.time.local.minute,\
                                cling.time.local.second, ##__VA_ARGS__);\
	}while(0)

#define UTC_DEBUG_LOG(...) \
	{ char buf[128]; \
		I8U len = sprintf(buf, "%d ", \
			cling.time.time_since_1970);\
		len += sprintf(buf+len, __VA_ARGS__);\
		len += sprintf(buf+len, "\r\n"); \
		UART2_print(buf);\
	}

#define SYSCLK_DEBUG_LOG(...) \
	{ char buf[128]; \
		I8U len = sprintf(buf, "%d ", \
			CLK_get_system_time());\
		len += sprintf(buf+len, __VA_ARGS__);\
		len += sprintf(buf+len, "\r\n"); \
		UART2_print(buf);\
	}
	
#define TIMED_PC_LOG(...) \
{ char buf[128]; \
	I8U len = sprintf(buf, "%d-%d-%d, %d:%d:%d ", \
	cling.time.local.year, \
	cling.time.local.month, \
	cling.time.local.day, \
	cling.time.local.hour, \
	cling.time.local.minute, \
	cling.time.local.second); \
	len += sprintf(buf + len, __VA_ARGS__); \
	len += sprintf(buf + len, "\r\n"); \
	printf(buf); \
}

#define TIMED_PC_FILE_LOG(...) \
{ char buf[128]; \
	I8U len = sprintf(buf, "%d-%d-%d, %d:%d:%d ", \
	cling.time.local.year, \
	cling.time.local.month, \
	cling.time.local.day, \
	cling.time.local.hour, \
	cling.time.local.minute, \
	cling.time.local.second); \
	len += sprintf(buf + len, __VA_ARGS__); \
	len += sprintf(buf + len, "\r\n"); \
	fprintf(fLog, "%s", buf); \
}

#define PURE_PC_FILE_LOG(...) \
{ char buf[128]; \
	I8U len = sprintf(buf, __VA_ARGS__); \
	len += sprintf(buf + len, "\r\n"); \
	fprintf(fLog, "%s", buf); \
}

#define PURE_DEBUG_LOG(...) \
	{ char buf[128]; \
		I8U len = sprintf(buf, __VA_ARGS__);\
		len += sprintf(buf+len, "\r\n"); \
		UART_print(buf);\
	}

//#define _ENABLE_BLE_DEBUG_

	//Debug information verbosity: lower values indicate higher urgency  
#define MYDEBUG_OFF 0  
#define MYDEBUG_ERR 1  
#define MYDEBUG_WRN 2  
#define MYDEBUG_TRC 3  
#define MYDEBUG_INF 4  

#define MYDEBUG_LEVEL MYDEBUG_INF
#define SET_DEBUG_LEVEL(__X) do\
{\
	MYDEBUG_LEVEL = __X;\
}while(0)

#define _ENABLE_UART_
	
#if defined(_ENABLE_UART_)

#define N_SPRINTF(...) 
#define SPRINTF_err(fmt, ...) do{\
	if(MYDEBUG_LEVEL > MYDEBUG_OFF){\
		TIMED_DEBUG_LOG("[err]:" fmt, ##__VA_ARGS__);\
	}\
}while(0)

#define SPRINTF_wrn(fmt, ...) do{\
	if(MYDEBUG_LEVEL > MYDEBUG_OFF){\
		TIMED_DEBUG_LOG("[wrn]:" fmt, ##__VA_ARGS__);\
	}\
}while(0)


#define SPRINTF_trc(fmt, ...) do{\
	if(MYDEBUG_LEVEL > MYDEBUG_OFF){\
		TIMED_DEBUG_LOG("[trc]:" fmt, ##__VA_ARGS__);\
	}\
}while(0)

#define SPRINTF_inf(fmt, ...) do{\
	if(MYDEBUG_LEVEL > MYDEBUG_OFF){\
		TIMED_DEBUG_LOG("[inf]" fmt, ##__VA_ARGS__);\
	}\
}while(0)


#define SPRINTF(fmt, ...) do{\
        if(MYDEBUG_LEVEL > MYDEBUG_OFF){\
                TIMED_DEBUG_LOG(fmt,##__VA_ARGS__);\
        }\
}while(0)

#define Y_SPRINTF SPRINTF

#define I_SPRINTF(...) PURE_DEBUG_LOG(__VA_ARGS__)
#define B_SPRINTF(...) SYSLOG_DEBUG_MESSAGE(__VA_ARGS__)

#define Y_DEBUG(level, msg...) SPRINTF_##level(msg)  
  
#undef assert  
#define assert(x) { \
    if (!(x)) { \
			printf(__FILE__ ":%d assert " #x " failed\n", __LINE__);\
    }\
}  
    
//Usage:  
//just include this file and define MYDEBUG_LEVEL like  
//  
//int MYDEBUG_LEVEL=MYDEBUG_ERR  
//  
//in the program use the command by  
//mydebug(off|err|wrn|trc|inf, "test %d with %s", 100, "this"); 


#elif defined(_ENABLE_BLE_DEBUG_)
	
#define N_SPRINTF(...)
#define Y_SPRINTF(...) 
#define I_SPRINTF(...)
#define B_SPRINTF(...) SYSLOG_DEBUG_MESSAGE(__VA_ARGS__)

#else 

#define N_SPRINTF(...)
#define Y_SPRINTF(...)
#define I_SPRINTF(...)
#define B_SPRINTF(...) 
#define Y_DEBUG(level, msg...)

#endif

void DEBUG_create_dbg_msg(I8U *msg, I8U len);
void DBG_event_processing(void);

#endif
