#ifndef __ROOT_HEADER__
#define __ROOT_HEADER__

#include "standard_types.h"
#include "fs_file.h"
#include "fs_memcfg.h"

// The Root Directory can have 32k/sizeof(root.) = 1024 file records
#define ROOT_BEGIN_POS	0
#define ROOT_END_POS	1023

// there are 4 root record (p_RootItem) within the 128 bytes buffer (map)
#define ROOT_MAP_MASK		0xfffc

// Always use this macro to convert "curr_flash_pos" to actual FLASH index
#define ROOT_FLASH_INDEX_SHIFT	2	

#define ROOTITEM_SIZE			32
#define ROOTITEM_SIZE_SHIFT		5	// sizeof(ROOTITEM) = 32

// 32 bytes -- Root item
typedef struct tagROOT_ITEM {
	I32U  time;
	I32U  size;
	I16U crc;
	I16U name_cluster;
	I16U cluster;
	I8U  attribute;
	I8U  file_type;
	I8U  type_options[10];
	I8U  reserved[4];
	I8U  backup_index;
	I8U  used_flag;
} ROOT_ITEM;

typedef struct tagROOT_DIR_CTX {
	I32U map[FAT_SECTOR_SIZE_DW];
	ROOT_ITEM * p_RootItem;
	I16U curr_page_pos;			// Current p_RootItem position
	I16U last_file_pos;
	I16U curr_flash_pos;	/* Current map position, must be 0, 4, 8, ... */
	I8U root_touched;
} ROOT_DIR_CTX;

// initialize root directory space
void ROOT_init(void);
//
// Creat a file in root directory 
I16S ROOT_CreateFile(I8U *file_name, I16U name_cluster, I16U first_cluster);
//
// Read file indexed in root directory
//
I16U ROOT_OpenReadFile(I16U pos, I16U * first_cluster, I32U * size);
//
// Close file for write
//
void ROOT_CloseWriteFile(I16U iPos, I32U size);

//
// Purpose: List the total files in SDRAM
// Return value: The total files' number
//
I16U ROOT_ListFiles(void);

void ROOT_SetFileInfo(I16U iPos, I8U * info);
BOOLEAN ROOT_GetFileInfo(I16U pos, I8U * info);

void ROOT_DeleteAppAllFiles(I8U iFileType);
//
// Purpose: Delete a file, this include erase the file record in the root directory and 
//          free all the FAT clusters the file formerly used
// Parameters: I16U iPos - the root position of the file to delete
//
void ROOT_DeleteFile(I16U iPos);
/* Purpose: Set the attribute and time of a file
   Parameters: I16U iPos - the root position value of the file
		       I8U attribute - the attribute to set
			   I32U time - the file time to set, in Unix time_t format
   Return value: 1 if successful. 
				 0 if the file do not exist or other errors occur
*/
void ROOT_SetFileType(I16U iPos, I8U file_type);
void ROOT_SetFileCRC(I16U pos, I16U crc);
void ROOT_SetFileAttribute(I16U iPos, I8U attribute);
void ROOT_SetFileTime(I16U iPos, I32U time);
  
/* Purpose: Get the attribute and time of a file
   Parameters: I16U iPos - the root position value of the file
		       I8U * attribute - the attribute to store
			   I32U * time - the file time to store, in Unix time_t format
   Return value: 1 if successful. 
				 0 if the file do not exist or other errors occur
*/
I32U ROOT_GetFileSize(I16U iPos);
I32U ROOT_GetFileTime(I16U iPos);
I8U ROOT_GetFileAttribute(I16U iPos);
I8U ROOT_GetFileType(I16U iPos);

I8U ROOT_GetFileName(I16U pos, I8U * filename);
I16U ROOT_GetLastPos(void);
void ROOT_flush_item(void);

#endif
