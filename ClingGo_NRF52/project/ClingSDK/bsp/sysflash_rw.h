#ifndef __SYSFLASH_RW_HEADER__
#define __SYSFLASH_RW_HEADER__

#include "standards.h"

enum {
	SYSFLASH_STATE_NOP,
	SYSFLASH_STATE_ERASURE,
	SYSFLASH_STATE_WRITE,
};

void SYSFLASH_drv_init(void);
void SYSFLASH_drv_read_sector(I32U addr, I8U *out, I16U len);
void SYSFLASH_drv_write_sector(I32U addr, I8U *in, I16U len);
I32U SYSFLASH_drv_sector_erasure(I32U add);
void SYSFLASH_drv_event_handler(I32U sys_evt);
void SYSFLASH_drv_page_erasure(I16U page_num);
void SYSFLASH_drv_write(I32U *dst, I32U *src, I32U size);

#endif
