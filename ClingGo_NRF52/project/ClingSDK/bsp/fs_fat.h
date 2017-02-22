#ifndef __FAT_HEADER__
#define __FAT_HEADER__

#include "standard_types.h"

// FAT sub type (FATFS.fs_type)

typedef enum {
	FS_NONE,
	FS_FAT12,
	FS_FAT16,
	FS_FAT32
} FS_TYPE_ENUM;

// FAT sub-type boundaries
// Note that the FAT spec by Microsoft says 4085 but Wi/ndows works with 4087!
#define MIN_FAT16	4086	// Minimum number of clusters for FAT16
#define	MIN_FAT32	65526	// Minimum number of clusters for FAT32

// Disk status bits 
#define FAT_STA_NOINIT		0x01	/* Drive not initialized */
#define FAT_STA_NODISK		0x02	/* No medium in the drive */
#define FAT_STA_PROTECT		0x04	/* Write protected */

// FAT structures 
// -- the members in the FAT structures as byte array 

#define FAT_BS_jmpBoot			0	/* Jump instruction (3) */
#define FAT_BS_OEMName			3	/* OEM name (8) */
#define FAT_BPB_BytsPerSec		11	/* Sector size [byte] (2) */
#define FAT_BPB_SecPerClus		13	/* Cluster size [sector] (1) */
#define FAT_BPB_RsvdSecCnt		14	/* Size of reserved area [sector] (2) */
#define FAT_BPB_NumFATs			16	/* Number of FAT copies (1) */
#define FAT_BPB_RootEntCnt		17	/* Number of root dir entries for FAT12/16 (2) */
#define FAT_BPB_TotSec16		19	/* Volume size [sector] (2) */
#define FAT_BPB_Media			21	/* Media descriptor (1) */
#define FAT_BPB_FATSz16			22	/* FAT size [sector] (2) */
#define FAT_BPB_SecPerTrk		24	/* Track size [sector] (2) */
#define FAT_BPB_NumHeads		26	/* Number of heads (2) */
#define FAT_BPB_HiddSec			28	/* Number of special hidden sectors (4) */
#define FAT_BPB_TotSec32		32	/* Volume size [sector] (4) */
#define FAT_BS_DrvNum			36	/* Physical drive number (2) */
#define FAT_BS_BootSig			38	/* Extended boot signature (1) */
#define FAT_BS_VolID			39	/* Volume serial number (4) */
#define FAT_BS_VolLab			43	/* Volume label (8) */
#define FAT_BS_FilSysType		54	/* File system type (1) */
#define FAT_BPB_FATSz32			36	/* FAT size [sector] (4) */
#define FAT_BPB_ExtFlags		40	/* Extended flags (2) */
#define FAT_BPB_FSVer			42	/* File system version (2) */
#define FAT_BPB_RootClus		44	/* Root dir first cluster (4) */
#define FAT_BPB_FSInfo			48	/* Offset of FSInfo sector (2) */
#define FAT_BPB_BkBootSec		50	/* Offset of backup boot sectot (2) */
#define FAT_BS_DrvNum32			64	/* Physical drive number (2) */
#define FAT_BS_BootSig32		66	/* Extended boot signature (1) */
#define FAT_BS_VolID32			67	/* Volume serial number (4) */
#define FAT_BS_VolLab32			71	/* Volume label (8) */
#define FAT_BS_FilSysType32		82	/* File system type (1) */
#define	FSI_LeadSig			0	/* FSI: Leading signature (4) */
#define	FSI_StrucSig		484	/* FSI: Structure signature (4) */
#define	FSI_Free_Count		488	/* FSI: Number of free clusters (4) */
#define	FSI_Nxt_Free		492	/* FSI: Last allocated cluster (4) */
#define FAT_MBR_Table			446	/* MBR: Partition table offset (2) */
#define	SZ_PTE				16	/* MBR: Size of a partition table entry */
#define FAT_BS_55AA				(510-384)	/* Boot sector signature (2) */

// The Root Directory begin position in SDRAM is 32k, the index of 32k is 256
#define ROOT_BEGIN_SDRAM_INDEX	256

// FAT MAP Number
#define FAT_MAP_NUM			64
#define FAT_MAP_NUM_SHIFT	6
#define FAT_MAP_NUM_MASK	0xffc0

#define N_FATS		1		// Number of FAT copies (1 or 2) 

typedef struct tagFAT_CTX {
	I32U fatbase; // FAT start sector
	I32U dirbase; // ROOT DIR start sector
	I32U database; // data start sector
	I32U fat_size;
	I32U free_clust;
	I16U last_clust;
	I16U used_clusters;
	I16U sector_size;	// Sector size, bytes per sector
	I16U cluster_size;	// Cluster size
	I16U sector_per_cluster;
	I16U n_rootdir;
	I16U n_sectors;
	I16U n_rsv;
	I16U n_clst;
	I8U map_sector_idx; // Point to sector indexed FAT map region
	I8U next_pos_in_map_sector;
	I8U map_index;
	I8U fat_touched;

	// --------------------
	FS_TYPE_ENUM fs_type;
	I8U status;
	I8U n_fats;

} FAT_CTX;

//
// Init the FAT 
// 
void FAT_Init(void);
//
// Find an empty FAT cluster entry hold data 
//
I16U FAT_GetEmptyEntry(void);
//
// Set a chain of FAT cluster entries to empty 
// 
void FAT_ClearEntries(I16U first);
//
// Build a chain of the FAT cluster entry
//
void FAT_BuildChain(I16U entry, I16U val);
//
// Get the next cluster entry from current entry
//
I16U FAT_GetNextChain(I16U entry);
//
// Clear FAT and ROOT directory space
//
void FAT_clear_fat_and_root_dir(void);
//
// Create file system
//
void FAT_create_fs(void);
//
// Sanity check on the file system, for future compatibility to Windows and Mac.
//
I8U FAT_chk_mounted(void);
//
// Unmount FAT file system
//
void FAT_unmounted(void);
//
// FLUSH FAT map when we close a file for writing
//
void FAT_flush_map(void);
//
// Check the remaining free space in FLASH
//
I32U FAT_CheckSpace(void);
#endif
