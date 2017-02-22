#ifndef __NFLASH_RW_HEADER__
#define __NFLASH_RW_HEADER__

#include "standards.h"

#define ERASE_WHOLE_CHIP 0
#define ERASE_64K        1
#define ERASE_32K        2
#define ERASE_4K         3

void NFLASH_init(void);
void NFLASH_drv_read_sector(I32U addr, I8U *out, I16U len);
void NFLASH_drv_write_sector(I32U addr, I8U *in, I16U len);
I32U NFLASH_drv_sector_erasure(I32U add, I16U mode);

#endif
