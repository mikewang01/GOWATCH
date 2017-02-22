#ifndef __NFLASH_SPI_H__
#define __NFLASH_SPI_H__

#include "standards.h"

#define EXPECTED_DEVICE         (0x13)        /* Device Identification for the M25P80 */
//#define FLASH_SIZE              (0x0100000)   /* Total device size in Bytes (1024 KB) */
#define FLASH_PAGE_COUNT        (0x01000)     /* Total device size in Pages (256 bytes per page) */
#define FLASH_SECTOR_COUNT      (0x10)        /* Total device size in Sectors (In 64 KB) */
#define FLASH_WRITE_BUFFER_SIZE 0x100         /* Write Buffer = 256 bytes */ 
#define FLASH_MWA               1             /* Minimum Write Access */ 
#define BE_TIMEOUT              (0x14)        /* Timeout in seconds suggested for Bulk Erase Operation*/

//
// InstructionsCode
//

enum
{
	//Instruction set
	SPI_FLASH_INS_WREN        = 0x06,		// write enable
	SPI_FLASH_INS_WRDI        = 0x04,		// write disable
	SPI_FLASH_INS_RDSR        = 0x05,		// read status register
	SPI_FLASH_INS_WRSR        = 0x01,		// write status register
	SPI_FLASH_INS_READ        = 0x03,		// read data bytes
	SPI_FLASH_INS_FAST_READ   = 0x0B,		// read data bytes at higher speed
	SPI_FLASH_INS_PP          = 0x02,		// page program
	SPI_FLASH_INS_SE_64K      = 0xD8,   // block erase 64KB
	SPI_FLASH_INS_SE_4K       = 0x20,   // sector erase 4KB
	SPI_FLASH_INS_SE_32K      = 0x52,   // sector erase 32KB
	SPI_FLASH_INS_RES         = 0xAB,		// release from deep power-down
	SPI_FLASH_INS_DP          = 0xB9,		// deep power-down
	SPI_FLASH_INS_RDID        = 0x90,		// read identification
	SPI_FLASH_INS_BE          = 0xC7,		// bulk erase
	SPI_FLASH_INS_DUMMY       = 0xAA
};

/*******************************************************************************
Flash Status Register Definitions (see Datasheet)
*******************************************************************************/
enum
{
	SPI_FLASH_SRWD	= 0x80,				// Status Register Write Protect
	SPI_FLASH_BP2	= 0x10,				// Block Protect Bit2
	SPI_FLASH_BP1	= 0x08,				// Block Protect Bit1
	SPI_FLASH_BP0	= 0x04,				// Block Protect Bit0
	SPI_FLASH_WEL	= 0x02,				// write enable latch
	SPI_FLASH_WIP	= 0x01				// write/program/erase in progress indicator
};

void NOR_init(void);

void NOR_writeEnable(void);

void NOR_writeDisable(void);

I8U NOR_readStatusRegister(void);

void NOR_readData(I32U addr, I16U len, I8U *dataBuf);

void NOR_pageProgram(I32U addr, I16U len, I8U *data);

void NOR_erase_block_4k(I32U addr);

void NOR_erase_block_32k(I32U addr);

void NOR_erase_block_64k(I32U addr);

void NOR_ChipErase(void);

void NOR_powerDown(void);

void NOR_releasePowerDown(void);

void NOR_readID(I8U *id);

#endif
