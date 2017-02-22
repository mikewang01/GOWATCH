/************************************************************************/
//
// CLING FILE SYSTEM - FAT related system management
//
// Created on Jan 03, 2014
//
/************************************************************************/
#include <string.h>

#include "standard_types.h"
#include "fs_memcfg.h"
#include "fs_flash.h"
#include "fs_file.h"
#include "fs_root.h"
#include "main.h"

/*--------------------------------*/
/* Multi-byte word access macros  */
#define	ST_I16U(ptr,val)	{*(I8U*)(ptr) = val&0xFF; *(I8U *)(ptr+1) = ((I16U)val>>8) &0xFF;}
#define	ST_I32U(ptr,val)	{*(I8U*)(ptr) = val&0xFF; *(I8U *)(ptr+1) = ((I32U)val>>8) &0xFF;*(I8U *)(ptr+2) = ((I32U)val>>16) &0xFF;*(I8U *)(ptr+3) = ((I32U)val>>24) &0xFF;}


FAT_CTX fat_mgr;
I16U fat_map_ctx[FAT_MAP_NUM];

I16U LD_I16U(I8U *ptr)
{
	I16U v16b;

	v16b = *(ptr+1);
	v16b <<= 8;
	v16b |= *ptr;

	return v16b;
}

I32U LD_I32U(I8U *ptr)
{
	I32U v32b;

	v32b = *(ptr+3);
	v32b <<= 8;
	v32b |= *(ptr+2);
	v32b <<= 8;
	v32b |= *(ptr+1);
	v32b <<= 8;
	v32b |= *ptr;

	return v32b;
}

static void _get_used_clusters()
{
	I16U i, j;
	I32U dw_buf[32];

	for (j = 0; j < fat_mgr.fat_size; j ++)
	{
		FLASH_Read_FAT(j, (I8U *)dw_buf);
		memcpy((I8U *)fat_map_ctx, (I8U *)dw_buf, 128);

		for (i = 0; i < FAT_MAP_NUM; i ++)
		{
			if (fat_map_ctx[i] != 0x7FFF)
			{
				fat_mgr.used_clusters ++;
			}
		}
	}
}

void FAT_Init()
{
	//
	// FAT and Root Dir used 512 clusters (65536 bytes)
	//
	fat_mgr.used_clusters = fat_mgr.database / fat_mgr.sector_per_cluster;

	// Get used clusters
	_get_used_clusters();

	// Reset map sector index
	fat_mgr.map_sector_idx = 0;

	// Load the first FAT mapping context
	FLASH_Read_FAT(fat_mgr.map_sector_idx, (I8U *)fat_map_ctx);

	fat_mgr.next_pos_in_map_sector = 0;

	fat_mgr.fat_touched = 0;
}

I16U FAT_GetEmptyEntry()
{
	I32U i, cnt;
	I16U empty_cluster;

	// Empty sector
	empty_cluster = fat_mgr.map_sector_idx;
	empty_cluster <<= FAT_MAP_NUM_SHIFT; // Each map sector represents 64 clusters

	// Check if there is empty entry in the current FAT map
	for (i = fat_mgr.next_pos_in_map_sector; i < FAT_MAP_NUM; i ++)
	{
		if (fat_map_ctx[i] == 0x7FFF)
		{
			fat_map_ctx[i] = (I16U)0x3fff;
			empty_cluster += i;
			fat_mgr.used_clusters ++;
			fat_mgr.next_pos_in_map_sector = i + 1;
			fat_mgr.fat_touched = 1;
			return empty_cluster;
		}
	}

	// check whether all clusters are used
	cnt = (FLASH_SIZE - FAT_CLING_APP_SPACE_SIZE) >> FAT_CLUSTER_SIZE_SHIFT;
	if (fat_mgr.used_clusters == cnt)
		return 0;

	// If there is a unused cluster, go find it and use it to create a new file.
	// Flush FAT map
	if (fat_mgr.fat_touched)
		FLASH_Write_FAT(fat_mgr.map_sector_idx, (I8U *)fat_map_ctx);

	// circular search to the end, then from the begining, take newly available space from deleted files
	for (cnt = 0; cnt < fat_mgr.fat_size; cnt++)
	{
		if (fat_mgr.map_sector_idx >= fat_mgr.fat_size)
			return 0;

		// Each map contains 64 clusters
		fat_mgr.map_sector_idx ++;
		empty_cluster += FAT_MAP_NUM;

		FLASH_Read_FAT(fat_mgr.map_sector_idx, (I8U *)fat_map_ctx);

		for (i = 0; i < FAT_MAP_NUM; i ++)
		{
			if (fat_map_ctx[i] == 0x7FFF)
			{
				fat_map_ctx[i] = (I16U)0x3fff;
				empty_cluster += i;
				fat_mgr.used_clusters ++;
				fat_mgr.next_pos_in_map_sector = i + 1;
				fat_mgr.fat_touched = 1;
				return empty_cluster;
			}
		}
	}

	return 0;
}

void FAT_flush_map()
{
	if (!fat_mgr.fat_touched) return;

	FLASH_Write_FAT(fat_mgr.map_sector_idx, (I8U *)fat_map_ctx);
}

static void _swap_fat_page(I16U in_entry)
{
	I16U pos;
	I8U iIndex;

	// 64 clusters per FAT map
	pos = in_entry;
	pos &= FAT_MAP_NUM_MASK;
	fat_mgr.map_index = (I8U)(in_entry - pos);

	pos >>= FAT_MAP_NUM_SHIFT;
	iIndex = (I8U)pos;

	if (iIndex != fat_mgr.map_sector_idx)
	{
		// If FAT map ever changed, update it
		if (fat_mgr.fat_touched)
			FLASH_Write_FAT(fat_mgr.map_sector_idx, (I8U *)fat_map_ctx);

		// Load new fat map.
		fat_mgr.map_sector_idx = iIndex;
		FLASH_Read_FAT(fat_mgr.map_sector_idx, (I8U *)fat_map_ctx);

		fat_mgr.next_pos_in_map_sector = 0;
		fat_mgr.fat_touched = 0;
	}
}

void FAT_ClearEntries(I16U first)
{
	//I16U next;

	// To avoid too much of NOR flash erase/write, we skip entry cleanup
	// by erasing whole NOR flash, and back up critical system files
	return ;
#if 0
	next = first;
	do
	{
		_swap_fat_page(next);

		// As FAT map links clusters for the file, we can
		// delete them one by one
		next = fat_map_ctx[fat_mgr.map_index];
		fat_map_ctx[fat_mgr.map_index] = 0;

		fat_mgr.used_clusters --;
		fat_mgr.fat_touched = 1;
	} while (next != (I16U)0xffff);

	fat_mgr.next_pos_in_map_sector = 0;	// find the next empty cluster here
#endif
}

void FAT_BuildChain(I16U entry, I16U val)
{
	// Get FAT map for the cluster -- "iCurEntry"
	_swap_fat_page(entry);

	// Write the value in the particular map location, which indicates
	// the corresponding cluster is used.
	fat_map_ctx[fat_mgr.map_index] = val;
	fat_mgr.fat_touched = 1;
}

I16U FAT_GetNextChain(I16U entry)
{
	// For read content out of a file, the Map chain is already built up
	_swap_fat_page(entry);

	return fat_map_ctx[fat_mgr.map_index];
}

I32U FAT_CheckSpace()
{
	I32U space_all, space_used;

	space_all = FLASH_SIZE - FAT_CLING_APP_SPACE_SIZE;
	space_used = fat_mgr.used_clusters;
	space_used <<= FAT_CLUSTER_SIZE_SHIFT;
	space_all -= space_used;
	return space_all;
}

void FAT_clear_fat_and_root_dir()
{
	memset(&fat_mgr, 0, sizeof(FAT_CTX));

	// Create file system boot sector
	FAT_create_fs();
}

I32U get_fattime()
{
	TIMESET tm;

	/* Get local time */
	tm.year = 2012;
	tm.month = 6;
	tm.dayOfWeek = 2;
	tm.day = 26;
	tm.hour = 11;
	tm.minute = 43;
	tm.second = 20;
	tm.milisecond = 221;

	/* Pack date and time into a DWORD variable */
	return 	  ((I32U)(tm.year - 1980) << 25)
			| ((I32U)tm.month << 21)
			| ((I32U)tm.day << 16)
			| ((I16U)tm.hour << 11)
			| ((I16U)tm.minute << 5)
			| ((I16U)tm.second >> 1);
}

//
// Create file system and initialize the boot sector
//
void FAT_create_fs()
{
	FAT_CTX *fs = &fat_mgr;
	//I32U n_dir;
	I32U n_time, app_rsvr_space;
	I8U md;
	I32U dw_buf[32];
	I32U i, n_info, j, k;
	I8U *tbl;

	tbl = (I8U *)dw_buf;

	// Use FAT 16 in the CLING (customized, not compatible to MS-DOS 5.0)

	fs->fs_type = FS_FAT16; // File system sub-type

	// 512 bytes in boot sector, clear it before the initialization
	memset(tbl, 0, FAT_SECTOR_SIZE);
	FLASH_Write_BS(1, tbl);
	FLASH_Write_BS(2, tbl);

	// ----------------------------------------------
	fs->sector_size = FAT_SECTOR_SIZE; // bytes per sector
	fs->cluster_size = FAT_CLUSTER_SIZE; // bytes per cluster
	fs->sector_per_cluster = fs->cluster_size / fs->sector_size;	// Number of sectors per cluster

	// Partition information
	app_rsvr_space = FAT_CLING_APP_SPACE_SIZE;

	fs->n_sectors = (I16U)((FLASH_SIZE - app_rsvr_space) / (I32U)fs->sector_size); // Number of sectors
	// Determine offset and size of FAT structure.
	fs->n_rsv = 4; // 512 bytes -> corresponding to 4 sectors (128 bytes per sector)
	fs->n_rootdir = N_ROOTDIR;

	// Num of clusters in the FAT excluding the first 8 KB, reserved for fat+dir.
	fs->n_clst = (fs->n_sectors - (FAT_SYSTEM_REVD_SPACE_SIZE / fs->sector_size)) / fs->sector_per_cluster;
	// rounding to 64 cluster group
	fs->fat_size = fs->n_clst << 1; // For FAT 16, each cluster takes 16 bits (2 bytes), plus extra 4 bytes?
	fs->fat_size = (fs->fat_size + fs->sector_size - 1) / fs->sector_size;

	fs->n_fats = N_FATS; // Num of FAT copies
//	n_dir = fs->n_rootdir * ROOTITEM_SIZE / fs->sector_size;

	// | -- System sector --| + | -- FAT area -- | + | -- ROOT DIR area -- | + | -- Data area --|
	fs->fatbase = fs->n_rsv + 0;				// FAT area start sector
	fs->dirbase = (FAT_SYSTEM_REVD_SPACE_SIZE>>1) / fs->sector_size;		// Directory area start sector
	//fs->database = fs->dirbase + n_dir;				// Data area start sector
	fs->database = FAT_SYSTEM_REVD_SPACE_SIZE / fs->sector_size;

	// No patition table (SFD)
	md = 0xF0;

	// Create BPB in the VBR */
	memset(tbl, 0, FAT_SECTOR_SIZE);

	memcpy(tbl, "\xEB\xFE\x90" "MSDOS5.0", 11);/* Boot jump code, OEM name */
	ST_I16U(tbl+FAT_BPB_BytsPerSec, fs->sector_size);
	tbl[FAT_BPB_SecPerClus] = (I8U)fs->sector_per_cluster;			/* Sectors per cluster */
	ST_I16U(tbl+FAT_BPB_RsvdSecCnt, fs->n_rsv);		/* Reserved sectors */
	tbl[FAT_BPB_NumFATs] = fs->n_fats;				/* Number of FATs */
	ST_I16U(tbl+FAT_BPB_RootEntCnt, fs->n_rootdir);/* Number of rootdir entries */
	ST_I16U(tbl+FAT_BPB_TotSec16, fs->n_sectors);
	tbl[FAT_BPB_Media] = md;					/* Media descriptor */
	ST_I16U(tbl+FAT_BPB_SecPerTrk, 63);			/* Number of sectors per track */
	ST_I16U(tbl+FAT_BPB_NumHeads, 255);			/* Number of heads */
	ST_I32U(tbl+FAT_BPB_HiddSec, 0);		/* Hidden sectors */
	n_time = get_fattime();
	ST_I32U(tbl+FAT_BS_VolID, n_time);			/* VSN */
	ST_I16U(tbl+FAT_BPB_FATSz16, fs->fat_size);	/* Number of sectors per FAT */
	tbl[FAT_BS_DrvNum] = 0x80;				/* Drive number */
	tbl[FAT_BS_BootSig] = 0x29;				/* Extended boot signature */
	memcpy(tbl+FAT_BS_VolLab, "NO NAME    " "FAT     ", 19);	/* Volume label, FAT signature */

	// The first 128 bytes
	FLASH_Write_BS(0, tbl);

	// The last 128 bytes
	memset(tbl, 0, fs->sector_size);
	ST_I16U(tbl+FAT_BS_55AA, 0xAA55);			/* Signature (Offset is fixed here regardless of sector size) */
	FLASH_Write_BS(3, tbl);

	/* Initialize FAT area */

	for (i = 0; i < fs->n_fats; i++) {		/* Initialize each FAT copy */
		//
		// 1st sector of the FAT
		//
		memset(tbl, 0xFF, FAT_SECTOR_SIZE);
		for( j=1; j< FAT_SECTOR_SIZE;j+=2)
		{
				tbl[j] = 0x7F;
		}
		n_info = md;								/* Media descriptor byte */

		n_info |= 0xFFFFFF00;

		ST_I32U(tbl+0, n_info);				/* Reserve cluster #0-1 (FAT12/16) */
		FLASH_Write_FAT(0,tbl);		// the first sector.

		for (j = 1; j < fs->fat_size; j++) {
			//
			// Remainning sectors of the FAT
			//
			memset(tbl, 0xFF, FAT_SECTOR_SIZE);			/* Fill following FAT entries with zero */
			for(k=1; k < FAT_SECTOR_SIZE; k+=2)
			{
					tbl[k] = 0x7F;
			}
			FLASH_Write_FAT(j, tbl);
		}
	}
}


static I8U _check_fs (I8U *buf)
{
	I8U tbl[FAT_SECTOR_SIZE];

	// The last 128 bytes
	FLASH_Read_BS(3, tbl);
	if (LD_I16U(&tbl[FAT_BS_55AA]) != 0xAA55)		/* Check record signature (always placed at offset 510 even if the sector size is >512) */
		return 0;

	if ((LD_I32U(&buf[FAT_BS_FilSysType]) & 0xFFFFFF) == 0x544146)	/* Check "FAT" string */
		return 1;

	return 0;
}

void FAT_unmounted()
{
	FAT_CTX *fs = &fat_mgr;// Get the static FAT manager

	fs->fs_type = FS_NONE;					/* Clear the file system object */
	fs->status = FAT_STA_NODISK;
}
/*-----------------------------------------------------------------------*/
/* Check if the file system object is valid or not                       */
/*-----------------------------------------------------------------------*/

I8U FAT_chk_mounted()
{
	FAT_CTX *fs = &fat_mgr;// Get the static FAT manager
	I8U fmt;
	I8U bootSector[FAT_SECTOR_SIZE];
	I32U sysect, bsect=0, szbfat;

	/* The file system object is not valid. */
	FLASH_Read_BS(0, bootSector);

	fs->sector_size = FAT_SECTOR_SIZE; /* Get disk sector size (variable sector size cfg only) */

	/* Search FAT partition on the drive. Supports only generic partitionings, FDISK and SFD. */
	fmt = _check_fs(bootSector);		/* Load sector 0 and check if it is an FAT-VBR (in SFD) */
	if (fmt != 1) {
		return 0;
	}

	/* An FAT volume is found. Following code initializes the file system object */

	if (LD_I16U(bootSector+FAT_BPB_BytsPerSec) != fs->sector_size)		/* (BPB_BytsPerSec must be equal to the physical sector size) */
		return 0;

	fs->fat_size = LD_I16U(bootSector+FAT_BPB_FATSz16);				/* Number of sectors per FAT */

	fs->n_fats = bootSector[FAT_BPB_NumFATs];				/* Number of FAT copies */
	if (fs->n_fats != 1) return 0;		/* (Must be 1 or 2) */

	fs->sector_per_cluster = bootSector[FAT_BPB_SecPerClus];			/* Number of sectors per cluster */
	fs->n_rootdir = LD_I16U(bootSector+FAT_BPB_RootEntCnt);	/* Number of root directory entries */
	fs->n_sectors = LD_I16U(bootSector+FAT_BPB_TotSec16);				/* Number of sectors on the volume */
	fs->n_rsv = LD_I16U(bootSector+FAT_BPB_RsvdSecCnt);				/* Number of reserved sectors */

	/* Determine the FAT sub type */
	sysect = FAT_SYSTEM_REVD_SPACE_SIZE / fs->sector_size;	/* RSV+FAT+DIR */
	if (fs->n_sectors < sysect)
		return 0;		/* (Invalid volume size) */
	fs->n_clst = (fs->n_sectors - sysect) / fs->sector_per_cluster;				/* Number of clusters */
	fs->fs_type = FS_FAT16;

	/* Boundaries and Limits */
	fs->database = bsect + sysect;						/* Data start sector */
	fs->fatbase = bsect + fs->n_rsv; 						/* FAT start sector */

	if (!fs->n_rootdir)	return 0;	/* (BPB_RootEntCnt must not be 0) */
	fs->dirbase = sysect >> 1;				/* Root directory start sector */
	szbfat = fs->n_clst * 2 ;	// Extra 2 clusters at begining of data space.

	if (fs->fat_size < ((szbfat + fs->sector_size - 1) / fs->sector_size))	/* (BPB_FATSz must not be less than required) */
		return 0;

	/* Initialize cluster allocation information */
	fs->free_clust = 0xFFFFFFFF;
	fs->last_clust = 0;

	return 1;
}

