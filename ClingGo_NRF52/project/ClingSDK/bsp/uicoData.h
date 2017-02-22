//#include <absacc.h> 
#ifndef __UICO_DATA_H__
#define __UICO_DATA_H__

//#define UICO_FORCE_BURN_FIRMWARE

//#define UICO_INCLUDE_FIRMWARE_BINARY_UPDATE_ENABLE

#ifdef UICO_INCLUDE_FIRMWARE_BINARY_UPDATE_ENABLE

#define MAX_UICO_FIRMWARE_SIZE  14 //KB
#define UICO_BINARY_FILE_SIZE    (unsigned long)(MAX_UICO_FIRMWARE_SIZE<<10)/*max firmware size mutiple by 1024 times*/
#define LOWER_EIGHT_BYE(X)    ((unsigned char)(X&0x00ff))
#define HIGHER_EIGHT_BYE(X)    ((unsigned char)((X>>8)&0x00ff))
#define UICO_FILE_LENTH_AREA_SIZE sizeof(uint16_t)
	
#define UICO_GET_BINARY_LENTH()  (uicoDat[0]|(uicoDat[1]<<8))
#define UICO_GET_BINARY_BUFFER()  ((unsigned char*)&uicoDat[UICO_FILE_LENTH_AREA_SIZE])
extern const unsigned char uicoDat[];
#endif

#endif

