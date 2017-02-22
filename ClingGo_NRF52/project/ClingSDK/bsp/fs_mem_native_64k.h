
#ifdef MCU_NATIVE_FLASH_SIZE_64KB
	// 
	#define N_ROOTDIR	128		// Number of root dir entries for FAT16, i.e. overall files, 

	// Starting offset (from the very begining of Nor Flash)
	#define FAT_FLASH_BASE_ADDR             0x00030000
	#define FAT_FLASH_PAGE_ALL              64
	#define FAT_FLASH_START_PAGE            192

	// Flash erase area size - 1 KB
	#define FLASH_ERASE_BLK_SIZE						(1024)
	#define FLASH_BLK_SHIFT                 10

	// FAT Reserved space 8 KB, FAT: (4K - 51), DIR: 4K
	#define FAT_SYSTEM_REVD_SPACE_SIZE			(8192)
	
	// The maximum cluster size for each section
	#define FS_BOOT_CLUSTER_MAX 4
	#define FS_FAT_CLUSTER_MAX 28
	#define FS_ROOT_CLUSTER_MAX 32
	#define FS_DATA_CLUSTER_MAX 272  // 64 KB - 8 KB (APP) - 22 KB (SYSTEM) = 34 KB
	#define FS_APP_CLUSTER_MAX 176
	
	// Using reserved space for application scratch pad (Minute file: 12 KB, Pedo state: 1 KB, AUTH: 1 KB, SYSTEM CRITICAL: 1 KB, UNUSED: 1 KB)
	#define FAT_CLING_APP_SPACE_SIZE				(22528)
	#define FAT_CLING_APP_PAGE_ALL 				   22

	#define SYSTEM_TRACKING_SPACE_START	0					// Minute activity
	#define SYSTEM_TRACKING_SPACE_SIZE	(12288)		// Minute activity SPACE - 10 KB
	#define SYSTEM_TRACKING_PAGE_SIZE   12
	#define SYSTEM_PEDO_STATE_SPACE_START		(SYSTEM_TRACKING_SPACE_START+SYSTEM_TRACKING_SPACE_SIZE)		// Pedometer state - 320 bytes
	#define SYSTEM_PEDO_STATE_PAGE_SIZE     1
	#define SYSTEM_PEDO_STATE_SPACE_SIZE		(1024)
	#define SYSTEM_LINK_SPACE_START					(SYSTEM_PEDO_STATE_SPACE_START+SYSTEM_PEDO_STATE_SPACE_SIZE)
	#define SYSTEM_LINK_PAGE_SIZE           1
	#define SYSTEM_LINK_SPACE_SIZE					(1024)
	#define SYSTEM_INFORMATION_SPACE_START			(SYSTEM_LINK_SPACE_START+SYSTEM_LINK_SPACE_SIZE)
	#define SYSTEM_INFORMATION_PAGE_SIZE       1
	#define SYSTEM_INFORMATION_SPACE_SIZE			(1024)

	#define FLASH_SIZE                      (65536) /* 64K */

	// Non-FAT cling application space starting address cluster fat manages, 
	#define CLING_APP_SPACE                 (FLASH_SIZE - FAT_CLING_APP_SPACE_SIZE)  /* 128k - 22K */
	#define CLING_APP_PAGE_START            (106)

#endif
