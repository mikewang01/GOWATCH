
#ifdef EXTERNAL_FLASH_SIZE_368KB

	// 
	#define N_ROOTDIR	1024		// Number of root dir entries for FAT16, i.e. overall files, 

	// Starting offset (from the very begining of Nor Flash)
	#define FAT_FLASH_BASE_ADDR             0
	#define FAT_FLASH_PAGE_ALL              92

	// Flash erase area size - 4 KB
	#define FLASH_ERASE_BLK_SIZE						(4096)
	#define FLASH_BLK_SHIFT                 12
	
	// FAT Reserved space 32 KB, FAT: (32K - 512), DIR: 32K
	#define FAT_SYSTEM_REVD_SPACE_SIZE			(65536)

	// The maximum cluster size for each section
	#define FS_BOOT_CLUSTER_MAX 4
	#define FS_FAT_CLUSTER_MAX 252
	#define FS_ROOT_CLUSTER_MAX 256
	#define FS_DATA_CLUSTER_MAX 1920  // 368 KB - 64 KB (APP) - 64 KB (SYSTEM) = 240 KB
	#define FS_APP_CLUSTER_MAX 512
	
	// Using reserved space for application scratch pad 
	//
	// Linking space: 4 KB
	// Critical info space: 4 KB
	// Daily statistics: 4 KB
	// General scratch pad: 4 KB
	// Tracking space: 32 KB
	// Reminder space: 4 KB
	//
	// Overall: 4+4+4+4+32+4 = 52 KB
	//
	#define FAT_CLING_APP_SPACE_SIZE				(65536)
	#define FAT_CLING_APP_PAGE_ALL				   16

  // System linking cridentials
	#define SYSTEM_LINK_SPACE_START					0
	#define SYSTEM_LINK_PAGE_SIZE           1
	#define SYSTEM_LINK_SPACE_SIZE					(4096)
	// System information space (for backup per minute)
	#define SYSTEM_INFORMATION_SPACE_START	(SYSTEM_LINK_SPACE_START+SYSTEM_LINK_SPACE_SIZE)
	#define SYSTEM_INFORMATION_PAGE_SIZE    1
	#define SYSTEM_INFORMATION_SPACE_SIZE		(4096)
	#define SYSTEM_DAYSTAT_SPACE_START	    (SYSTEM_INFORMATION_SPACE_START+SYSTEM_INFORMATION_SPACE_SIZE)		// Daily stats
	#define SYSTEM_DAYSTAT_PAGE_SIZE        1
	#define SYSTEM_DAYSTAT_SPACE_SIZE		    (4096)
	#define SYSTEM_SCRATCH_SPACE_START	    (SYSTEM_DAYSTAT_SPACE_START+SYSTEM_DAYSTAT_SPACE_SIZE)		// Scratch pad
	#define SYSTEM_SCRATCH_PAGE_SIZE        1
	#define SYSTEM_SCRATCH_SPACE_SIZE		    (4096)
	//
	// Minute data, each entry is 16 bytes long, and overall we need 16 x 1440 = 23040 KB, 
	// Add extra 8 KB for buffering, so we are getting 31 KB in the application space, round it to 4 KB
	// boundary, so we got 32 KB.
	//
	#define SYSTEM_TRACKING_SPACE_START	    (SYSTEM_SCRATCH_SPACE_START+SYSTEM_SCRATCH_SPACE_SIZE)					// Minute tracking space
	#define SYSTEM_TRACKING_SPACE_SIZE	    (32768)		// Minute activity SPACE - 28 KB
	#define SYSTEM_TRACKING_PAGE_SIZE       8	

	// Reminder space
	//
	#define SYSTEM_REMINDER_SPACE_START	    (SYSTEM_TRACKING_SPACE_START+SYSTEM_TRACKING_SPACE_SIZE)		// System reminder space
	#define SYSTEM_REMINDER_PAGE_SIZE       1
	#define SYSTEM_REMINDER_SPACE_SIZE		  (4096)

	// Message space
	//
	#define SYSTEM_NOTIFICATION_SPACE_START	    (SYSTEM_REMINDER_SPACE_START+SYSTEM_NOTIFICATION_SPACE_SIZE)		// System reminder space
	#define SYSTEM_NOTIFICATION_PAGE_SIZE       1
	#define SYSTEM_NOTIFICATION_SPACE_SIZE		  (4096)

	#define FLASH_SIZE                      (376832) /* 368 KB */

	// Non-FAT cling application space starting address cluster fat manages, 
	#define CLING_APP_SPACE                 (FLASH_SIZE - FAT_CLING_APP_SPACE_SIZE)  /* 368 KB - 64 KB */
	#define CLING_APP_PAGE_START            (76) /* 92 pages - 16 pages. */

#endif
