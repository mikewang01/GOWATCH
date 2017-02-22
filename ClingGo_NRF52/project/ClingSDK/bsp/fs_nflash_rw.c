#include "main.h"
#include "nflash_spi.h"
#ifdef _CLING_PC_SIMULATION_
extern char *p_flash;
#endif

void NFLASH_init()
{
#ifndef _CLING_PC_SIMULATION_
	NOR_init();
#else
	memset(p_flash, 0xff, FLASH_SIZE);
#endif
}

// Note: according to nordic spec, it takes typical 21 ms to erase a page, 
// be very careful.
I32U NFLASH_drv_sector_erasure(I32U addr, I16U mode)
{	
#ifndef _CLING_PC_SIMULATION_
	if (mode == ERASE_4K)
		NOR_erase_block_4k(FAT_FLASH_BASE_ADDR + addr);    // erase 4 KB block
	else if (mode == ERASE_32K)
		NOR_erase_block_32k(FAT_FLASH_BASE_ADDR + addr);   // erase 32 KB block
	else if (mode == ERASE_64K)
		NOR_erase_block_64k(FAT_FLASH_BASE_ADDR + addr);   // erase 64 KB block
	else if (mode == ERASE_WHOLE_CHIP)
		NOR_ChipErase();                                   // erase the whole chip
	else
		NOR_erase_block_4k(FAT_FLASH_BASE_ADDR + addr);    // erase 4 KB block
#else
	if (mode == ERASE_4K)
		memset(p_flash + FAT_FLASH_BASE_ADDR + addr, 0xff, 4096);
	else if (mode == ERASE_32K)
		memset(p_flash + FAT_FLASH_BASE_ADDR + addr, 0xff, 32768);
	else if (mode == ERASE_64K)
		memset(p_flash + FAT_FLASH_BASE_ADDR + addr, 0xff, 65536);
	else if (mode == ERASE_WHOLE_CHIP)
		memset(p_flash + FAT_FLASH_BASE_ADDR + addr, 0xff, FLASH_SIZE);
	else
		memset(p_flash + FAT_FLASH_BASE_ADDR + addr, 0xff, 4096);
#endif

	return 0;
}

// ---------------------------------------------------------------------------------------
// Function:   NFLASH_drv_read_sector()
//
// Description:
//    Read out data from NFLASH directly!
//
// Parameter:
//    add: the physical address in flash
//    len: data bytes
//    out: output buffer
//
// Note:
//    All corrsponding register is only writable, can't be read!
// ---------------------------------------------------------------------------------------
void NFLASH_drv_read_sector(I32U addr, I8U *out, I16U len)
{
#ifndef _CLING_PC_SIMULATION_
	NOR_readData(FAT_FLASH_BASE_ADDR + addr, len, out);
#else
	memcpy(out, p_flash + FAT_FLASH_BASE_ADDR + addr, len);
#endif
}

void NFLASH_drv_write_sector(I32U addr, I8U *in, I16U len)
{
#ifndef _CLING_PC_SIMULATION_
	NOR_pageProgram(FAT_FLASH_BASE_ADDR + addr, len, in);
#else
	int i;

	for (i = 0; i < len; i++) {
		p_flash[FAT_FLASH_BASE_ADDR + addr + i] &= in[i];
	}
#endif
}

