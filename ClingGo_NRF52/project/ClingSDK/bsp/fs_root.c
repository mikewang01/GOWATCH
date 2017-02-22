/************************************************************************/
//
// CLING FILE SYSTEM - FAT system root directory operations
//
// Created on Feb, 03, 2014
//
/************************************************************************/
#include <string.h>
#include <ctype.h>

#include "main.h"
#include "fs_root.h"

ROOT_DIR_CTX root;

static void _clear_FAT()
{
	// clear data cluster
	if(root.p_RootItem->cluster)
	{
		FAT_ClearEntries(root.p_RootItem->cluster);
	}
	// clear name cluster
	FAT_ClearEntries(root.p_RootItem->name_cluster);
}

void ROOT_flush_item()
{
	I32U index;

	if (root.root_touched)
	{
		// 4 Root page per sector
		index = (root.curr_flash_pos>>ROOT_FLASH_INDEX_SHIFT);	// index = 0, 4, 8, 12, 16, ...
		FLASH_Write_ROOT(index, (I8U *)root.map);
		root.root_touched = 0;
		N_SPRINTF("[FROOT] update root (flush): %d", root.curr_flash_pos);
	}
}

static void _swap_root_page(I16U new_page_pos)
{
	I8U *pmap;
	I16U pos;
	I32U index;

	if (new_page_pos == root.curr_page_pos) return;

	//
	root.curr_page_pos = new_page_pos;

	pos = new_page_pos;
	pos &= ROOT_MAP_MASK;
	index = new_page_pos - pos;
	index <<= ROOTITEM_SIZE_SHIFT; // 32 bytes per root page, 4 pages per sector

	// Get the rootItem offset, if current root map is not loaded, then, load it!
	pmap = (I8U *)root.map;
	pmap += index;
	root.p_RootItem = (ROOT_ITEM *)pmap;

	// If root page is touched, we should save it before loading the new page
	ROOT_flush_item();

	// If flash position (128 bytes) changes, reload the flash content for 4 pages
	if (pos != root.curr_flash_pos)
	{
		root.curr_flash_pos = pos;

		index = (pos>> ROOT_FLASH_INDEX_SHIFT);	// index = 0, 4, 8, 12, 16, ...
		FLASH_Read_ROOT(index, (I8U *)root.map);
	}
}

I16U ROOT_GetLastPos()
{
	I16U pos;
	root.last_file_pos = 0;

	for (pos = ROOT_BEGIN_POS; pos < N_ROOTDIR; pos++) {
		// Load particular root
		_swap_root_page(pos);

		// check on used flag
		if (root.p_RootItem->used_flag == 0xff)
			break;

		root.last_file_pos ++;
	}

	N_SPRINTF("[FROOT] get last pos: %d, root flash:%d", pos, root.curr_flash_pos);

	return root.last_file_pos;
}

extern FAT_CTX fat_mgr;

void ROOT_init()
{
	FLASH_Read_ROOT(0, (I8U *)root.map);	// read first 128Bytes info from root to controller buffer;

	root.p_RootItem = (ROOT_ITEM *)root.map;

	root.curr_page_pos = 0;
	root.curr_flash_pos = 0;

	root.root_touched = 0;

	ROOT_GetLastPos();

	N_SPRINTF("[FROOT] root init: %d (%d,%d,%d)", root.last_file_pos, fat_mgr.database, fat_mgr.fatbase, fat_mgr.dirbase);

}

//
// Purpose: Delete a file, this include erase the file record in the root directory and
//          free all the FAT clusters the file formerly used
// Parameters: I16U pos - the Root record position of the file to delete
//
void ROOT_DeleteFile(I16U pos)
{
	if (pos > root.last_file_pos)		return;

	_swap_root_page(pos);

	// Clear all FAT clusters (both name and data)
	_clear_FAT();

	if (root.p_RootItem->used_flag == 0x7f) {
		// Clear ROOT space
		// Trash the ROOT space by setting it to 0x3f
		root.p_RootItem->used_flag = 0x3f;
		root.root_touched = 1;

		N_SPRINTF("[FROOT] delete a file (one)");
		ROOT_flush_item();
	}
}

//
// Purpose: List the total files in SDRAM
// Return value: The total files' number
//
I16U ROOT_ListFiles()
{
	return root.last_file_pos;
}

I8U ROOT_GetFileName(I16U pos, I8U *filename)
{
	_swap_root_page(pos);

	FLASH_read_data(root.p_RootItem->name_cluster, filename);

	return 1;
}

void ROOT_SetFileInfo(I16U pos, I8U * info)
{
	_swap_root_page(pos);
	memcpy((I8U *)root.p_RootItem, info, sizeof(ROOT_ITEM));
	root.root_touched = 1;
}

BOOLEAN ROOT_GetFileInfo(I16U pos, I8U * info)
{

	// Swap root page, and save the page if anything gets touched.
	_swap_root_page(pos);

	// Return TRUE if file is valid, FALSE if otherwise
	if (root.p_RootItem->used_flag == 0x7f) {
		// copy out ROOT page if this is a valid file
		memcpy(info, (I8U *)root.p_RootItem, sizeof(ROOT_ITEM));
		return 1;
	} else {
		return 0;
	}
}

I16S ROOT_CreateFile(I8U *file_name, I16U name_cluster, I16U first_cluster)
{
	N_SPRINTF("[FROOT] create file: %d, %s", root.last_file_pos, file_name);

	if (root.last_file_pos >= N_ROOTDIR)
		return -1;			/* can not add file because all root entry has been used */

	N_SPRINTF("[FROOT] --1--");

	_swap_root_page(root.last_file_pos);

	N_SPRINTF("[FROOT] --%d--", (I32U)root.p_RootItem);

	// Set file attributes
	root.p_RootItem->time = 0;
	root.p_RootItem->size = 0;
	root.p_RootItem->file_type = FILE_TYPE_ACTIVITY;
	root.p_RootItem->cluster = first_cluster;
	root.p_RootItem->name_cluster = name_cluster;
	root.p_RootItem->backup_index = 0;
	root.p_RootItem->attribute = FILE_FLAG_ARCHIVE;
	root.p_RootItem->used_flag = 0x7f;   // Used root dir, and a valid file
	root.root_touched = 1;

	N_SPRINTF("[FROOT] file created");

	// Write file name into the first sector of name cluster
	FLASH_write_data(root.p_RootItem->name_cluster, (I8U *)file_name);

	return root.last_file_pos;
}

I16U ROOT_OpenReadFile(I16U pos, I16U * first_cluster, I32U *size)
{
	// Load the specific root page (1 ~ 1024) for the file
	_swap_root_page(pos);

	// Get the first cluster index
	*first_cluster = root.p_RootItem->cluster;

	// Get the file size
	*size = root.p_RootItem->size;

	return root.p_RootItem->crc;
}

void ROOT_CloseWriteFile(I16U pos, I32U size)
{
	_swap_root_page(pos);

	root.p_RootItem->size = size;

	root.root_touched = 1;
}

void ROOT_SetFileCRC(I16U pos, I16U crc)
{
	_swap_root_page(pos);
	root.p_RootItem->crc = crc;
	root.root_touched = 1;
}

void ROOT_SetFileType(I16U pos, I8U file_type)
{
	_swap_root_page(pos);
	root.p_RootItem->file_type = file_type;
	root.root_touched = 1;
}

/* Purpose: Set the attribute and time of a file */
void ROOT_SetFileAttribute(I16U pos, I8U attribute)
{
	_swap_root_page(pos);
	root.p_RootItem->attribute = attribute;
	root.root_touched = 1;
}

void ROOT_SetFileTime(I16U pos, I32U time)
{
	_swap_root_page(pos);
	root.p_RootItem->time = time;
	root.root_touched = 1;
}

/* Purpose: Get the attribute, size and time of a file */
I32U ROOT_GetFileSize(I16U pos)
{
	_swap_root_page(pos);
	return root.p_RootItem->size;
}

I32U ROOT_GetFileTime(I16U pos)
{
	_swap_root_page(pos);
	return root.p_RootItem->time;
}

I8U ROOT_GetFileAttribute(I16U pos)
{
	_swap_root_page(pos);
	return root.p_RootItem->attribute;
}

I8U ROOT_GetFileType(I16U pos)
{
	_swap_root_page(pos);
	return root.p_RootItem->file_type;
}

void ROOT_DeleteAppAllFiles(I8U iFileType)
{
	I16U pos;
	I8U used;

	for (pos = ROOT_BEGIN_POS; pos < N_ROOTDIR; pos ++) {
		// Swap root page
		_swap_root_page(pos);

		// Check whether "used" is flagged a non-0xff value
		used = root.p_RootItem->used_flag;

		if (used == 0xff) break;

		if (used == 0x7f) {
			root.p_RootItem->used_flag = 0x3f;
			root.root_touched = 1;

			N_SPRINTF("[FROOT] delete a file (all)");

		}
	}
}
















