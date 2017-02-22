#ifndef __MEM_CONFIG_HEADER_
#define __MEM_CONFIG_HEADER_

//
// File system internal read and write buffer size
// Do not modify, because the whole system depend on this value to work
//
#define FAT_SECTOR_SIZE                   128
#define FAT_SECTOR_SIZE_DW                32

/* Every cluster in the file system contains 128 bytes */
#define FAT_CLUSTER_SIZE                  (128)
#define FAT_CLUSTER_SIZE_SHIFT            7
#define FAT_CLUSTER_INDEX_SIZE            1	/* each cluster contains 1 * FAT_SECTOR_SIZE */
#define FAT_CLUSTER_INDEX_SHIFT           0

// The FAT begin position in FLASH is 512, so the begin FLASH index is 4
#define BOOT_SECTOR_SIZE                  ((I16U)512)

//
// Note: the first FAT region is 512 bytes, used for system info.
// The beginning sector is 4.
#define FAT_BEGIN_FLASH_INDEX             4

// System reserved space starts at 249th page, overall 3 KB
//
#define SYSTEM_RSVR_SPACE_START					(254976) // 249 * 1024 - Bond manager, attributes, etc.
#define SYSTEM_RSVR_PAGE_SIZE           3
#define SYSTEM_RSVR_SPACE_SIZE					(3072)
// Bootloader space starts at 252th page, overall 4 KB
//
#define BOOTLOADER_SPACE_START					(258048) // 252 * 1024 - the last 4 kb
#define BOOTLOADER_PAGE_SIZE            4
#define BOOTLOADER_SPACE_SIZE           (4096)

#define APP_UPDATE_FLAG_ENABLE             0x0003FC00
#define APP_UPDATE_FLAG_START_ADDR         0x0003FC04
#define APP_UPDATE_FLAG_FILE_LEN           0x0003FC08
	
// MCU FLASH SIZE: 128 KB
//
//#define MCU_NATIVE_FLASH_SIZE_64KB
//#define EXTERNAL_FLASH_SIZE_1024KB
//#define EXTERNAL_FLASH_SIZE_368KB
#define EXTERNAL_FLASH_SIZE_256KB

#include "fs_mem_native_64k.h"
#include "fs_mem_external_368k.h"
#include "fs_mem_external_256k.h"

#endif//#ifndef __MEM_CONFIG_HEADER_
