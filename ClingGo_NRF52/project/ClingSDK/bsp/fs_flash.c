#include <string.h>

#include "main.h"

#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
#include "sysflash_rw.h"
#endif

extern FAT_CTX fat_mgr;

BOOLEAN FLASH_addr_validation(I8U section_index, I32U cluster)
{
	BOOLEAN b_bad_cluster = FALSE;
	switch (section_index) {
	case FS_SECTION_BOOT:
		if (cluster >= FS_BOOT_CLUSTER_MAX)
			b_bad_cluster = TRUE;
		break;
	case FS_SECTION_FAT:
		if (cluster >= FS_FAT_CLUSTER_MAX)
			b_bad_cluster = TRUE;
		break;
	case FS_SECTION_ROOT:
		if (cluster >= FS_ROOT_CLUSTER_MAX)
			b_bad_cluster = TRUE;
		break;
	case FS_SECTION_DATA:
		if (cluster >= FS_DATA_CLUSTER_MAX)
			b_bad_cluster = TRUE;
		break;
	case FS_SECTION_APP:
		if (cluster >= FS_APP_CLUSTER_MAX)
			b_bad_cluster = TRUE;
		break;
	default:
		break;
	}

	if (b_bad_cluster) {
		Y_SPRINTF("[FS] bad cluster index, cluster: %d, section: %d", cluster, section_index);
		// Erase Nor Flash if device is not authorized or File system undetected
		if (LINK_is_authorized()) {
			FLASH_erase_all(FALSE);
		} else {
			// If this is not a authorized device, erase auth info section and critical info section.
			FLASH_erase_all(TRUE);
		}

		SYSTEM_reboot();

		return FALSE;
	} else
		return TRUE;
}

/*----------------------------------------------------------------------------------
*  Read data out of file system data section
*----------------------------------------------------------------------------------*/
void FLASH_read_data(I32U cluster, I8U *cbBuffer)
{
	I32U add = cluster;

	if (!FLASH_addr_validation(FS_SECTION_DATA, cluster)) {
		memset(cbBuffer, 0xff, FAT_SECTOR_SIZE);
		return;
	}

	add <<= FAT_CLUSTER_INDEX_SHIFT;
	add += fat_mgr.database;
	add <<= 7;
#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	SYSFLASH_drv_read_sector(add, cbBuffer, FAT_SECTOR_SIZE);	// read one sector
#else
	NFLASH_drv_read_sector(add, cbBuffer, FAT_SECTOR_SIZE);	// read one sector
#endif
}

/*----------------------------------------------------------------------------------
*  Write data to file system data section
*----------------------------------------------------------------------------------*/
void FLASH_write_data(I32U cluster, I8U *cbBuffer)
{
	I32U  add = cluster;

	if (!FLASH_addr_validation(FS_SECTION_DATA, (I32U)cluster)) {
		return;
	}

	add <<= FAT_CLUSTER_INDEX_SHIFT;
	add += fat_mgr.database;
	add <<= 7;

#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	SYSFLASH_drv_write_sector(add, cbBuffer, FAT_SECTOR_SIZE);
#else
	NFLASH_drv_write_sector(add, cbBuffer, FAT_SECTOR_SIZE);
#endif
}

/*----------------------------------------------------------------------------------
*  Function:	NFLASH_Read_FAT()
*
*  Description:
*     Read out FAT information and save it into "cbBuffer"
*
*  Note:
*     In flash space, the first 128 bytes room have been used as system info space,
*  the real FAT information is saved from 2nd 128bytes.
*
*----------------------------------------------------------------------------------*/
void FLASH_Read_FAT(I32U cluster, I8U * cbBuffer)
{
	I32U add = cluster;

	if (!FLASH_addr_validation(FS_SECTION_FAT, cluster)) {
		memset(cbBuffer, 0xff, FAT_SECTOR_SIZE);
		return;
	}


	add += fat_mgr.fatbase;

	add <<= 7;

#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	SYSFLASH_drv_read_sector(add, cbBuffer, FAT_SECTOR_SIZE);	// read out data from sdram to controller buffer;
#else
	NFLASH_drv_read_sector(add, cbBuffer, FAT_SECTOR_SIZE);	// read out data from sdram to controller buffer;
#endif
}

/*----------------------------------------------------------------------------------
*  Function:	FLASH_Write_FAT()
*
*  Description:
*     Write FAT(64 cluster) information into SDRAM space.
*
*  Note:
*     In flash space, the first 128 bytes room have been used as system info space,
*  the real FAT information is saved from 2nd 128bytes.
*
*----------------------------------------------------------------------------------*/
void FLASH_Write_FAT(I32U cluster, I8U * cbBuffer)
{
	I32U add = cluster;

	if (!FLASH_addr_validation(FS_SECTION_FAT, cluster)) {
		return;
	}

	add += fat_mgr.fatbase;
	add <<= 7;

#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	SYSFLASH_drv_write_sector(add, cbBuffer, FAT_SECTOR_SIZE);
#else
	NFLASH_drv_write_sector(add, cbBuffer, FAT_SECTOR_SIZE);
#endif
}

/*----------------------------------------------------------------------------------
*  Function:	FLASH_Read_ROOT()
*
*  Description:
*     Read out ROOT directory information from SDRAM and save it into "cbBuffer".
*
*----------------------------------------------------------------------------------*/
void FLASH_Read_ROOT(I32U cluster, I8U * cbBuffer)
{
	I32U add = cluster;

	if (!FLASH_addr_validation(FS_SECTION_ROOT, cluster)) {
		memset(cbBuffer, 0xff, FAT_SECTOR_SIZE);
		return;
	}

	add += fat_mgr.dirbase;
	add <<= 7;

#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	SYSFLASH_drv_read_sector(add, cbBuffer, FAT_SECTOR_SIZE);
#else
	NFLASH_drv_read_sector(add, cbBuffer, FAT_SECTOR_SIZE);
#endif
}

/*----------------------------------------------------------------------------------
*  Function:	FLASH_Write_ROOT(I32U sector, I8U * cbBuffer)
*
*  Description:
*     Write ROOT directory information into SDRAM.
*
*----------------------------------------------------------------------------------*/
void FLASH_Write_ROOT(I32U cluster, I8U * cbBuffer)
{
	I32U add = cluster;

	if (!FLASH_addr_validation(FS_SECTION_ROOT, cluster)) {
		return;
	}

	add += fat_mgr.dirbase;
	add <<= 7;

#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	SYSFLASH_drv_write_sector(add, cbBuffer, FAT_SECTOR_SIZE);
#else
	NFLASH_drv_write_sector(add, cbBuffer, FAT_SECTOR_SIZE);
#endif
}


/*----------------------------------------------------------------------------------
*  Function:	FLASH_Read_BS()
*
*  Description:
*     Read out BOOT SECTOR information and save it into "cbBuffer".
*
*----------------------------------------------------------------------------------*/
void FLASH_Read_BS(I32U cluster, I8U * cbBuffer)
{
	I32U add = cluster;

	if (!FLASH_addr_validation(FS_SECTION_BOOT, cluster)) {
		memset(cbBuffer, 0xff, FAT_SECTOR_SIZE);
		return;
	}

	add <<= 7;

#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	SYSFLASH_drv_read_sector(add, cbBuffer, FAT_SECTOR_SIZE);
#else
	NFLASH_drv_read_sector(add, cbBuffer, FAT_SECTOR_SIZE);
#endif
}

/*----------------------------------------------------------------------------------
*  Function:	FLASH_Write_BS(I32U sector, I8U * cbBuffer)
*
*  Description:
*     Write BOOT SECTOR information into NOR FLASH.
*
*----------------------------------------------------------------------------------*/
void FLASH_Write_BS(I32U cluster, I8U * cbBuffer)
{
	I32U add = cluster;

	if (!FLASH_addr_validation(FS_SECTION_BOOT, cluster)) {
		return;
	}

	add <<= 7;

#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	SYSFLASH_drv_write_sector(add, cbBuffer, FAT_SECTOR_SIZE);
#else
	NFLASH_drv_write_sector(add, cbBuffer, FAT_SECTOR_SIZE);
#endif
}


/*----------------------------------------------------------------------------------
*  Function:	FLASH_erase_App(I32U add)
*
*  Description:
*     erase one page in application flash area
*
*----------------------------------------------------------------------------------*/
void FLASH_erase_App(I32U add)
{
	add += CLING_APP_SPACE; // Starting App space

#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	SYSFLASH_drv_sector_erasure(add);
#else
	Y_SPRINTF("[FS] App flash erase (4K): %d", add);
	NFLASH_drv_sector_erasure(add, ERASE_4K);
#endif
}

void FLASH_Read_Flash(I32U addr, I8U *cbBuffer, I8U len)
{
#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	SYSFLASH_drv_read_sector(addr, cbBuffer, len);
#else
	NFLASH_drv_read_sector(addr, cbBuffer, len);
#endif
}

void FLASH_Write_Flash(I32U addr, I8U *cbBuffer, I8U len)
{
	I32U align_buf[32];

	memcpy(align_buf, cbBuffer, len);

#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	SYSFLASH_drv_write_sector(addr, (I8U *)align_buf, len);
#else
	NFLASH_drv_write_sector(addr, (I8U *)align_buf, len);
#endif
}

/*----------------------------------------------------------------------------------
*  Function:	FLASH_Read_App(I32U add, I8U * cbBuffer, I8U len)
*
*  Description:
*     Read out BOOT SECTOR information and save it into "cbBuffer".
*
*----------------------------------------------------------------------------------*/
void FLASH_Read_App(I32U add, I8U * cbBuffer, I8U len)
{
	I32U cluster = add >> 7;
	if (!FLASH_addr_validation(FS_SECTION_APP, cluster)) {
		memset(cbBuffer, 0xff, FAT_SECTOR_SIZE);
		return;
	}

	add += CLING_APP_SPACE; // Starting App space


#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	SYSFLASH_drv_read_sector(add, cbBuffer, len);
#else
	NFLASH_drv_read_sector(add, cbBuffer, len);
#endif
}

/*----------------------------------------------------------------------------------
*  Function:	FLASH_Write_App(I32U add, I8U * cbBuffer, I8U len)
*
*  Description:
*     Write BOOT SECTOR information into NOR FLASH.
*
*----------------------------------------------------------------------------------*/
void FLASH_Write_App(I32U add, I8U * cbBuffer, I8U len)
{
	I32U cluster = add >> 7;

	I32U align_buf[32];

	if (!FLASH_addr_validation(FS_SECTION_APP, cluster)) {
		return;
	}

	memcpy(align_buf, cbBuffer, len);

	add += CLING_APP_SPACE; // Starting App space

#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	SYSFLASH_drv_write_sector(add, (I8U *)align_buf, len);
#else
	NFLASH_drv_write_sector(add, (I8U *)align_buf, len);
#endif
}


static void _core_erase(I16U page_end, I32U addr)
{
#if defined (MCU_NATIVE_FLASH_SIZE_64KB)
	I32U i;

	for (i = 0; i < page_end; i++) {
		addr = i;
		SYSFLASH_drv_sector_erasure(addr << 10);
	}
#else
	I16U page_to_erase;
	page_to_erase = page_end;
	//
	// Note: each page is 4 KB
	while (page_to_erase > 0) {
		if (page_to_erase >= 256) {
			Y_SPRINTF("[FS] erase(%d): %d, Whole chip", page_to_erase, addr);
			NFLASH_drv_sector_erasure(addr, ERASE_WHOLE_CHIP);
			addr += 1048576;
			page_to_erase -= 256;
		} else if ((page_to_erase >= 16) && ((addr%65536) == 0)) {
			Y_SPRINTF("[FS] erase(%d): %d, 64K", page_to_erase, addr);
			// we should erase 64 KB block
			NFLASH_drv_sector_erasure(addr, ERASE_64K);
			addr += 65536;
			page_to_erase -= 16;
		} else if ((page_to_erase >= 8) && ((addr%32768) == 0)) {
			Y_SPRINTF("[FS] erase(%d): %d, 32K", page_to_erase, addr);
			// we should erase 32 KB block
			NFLASH_drv_sector_erasure(addr, ERASE_32K);
			addr += 32768;
			page_to_erase -= 8;
		} else {
			Y_SPRINTF("[FS] erase(%d): %d, 4K", page_to_erase, addr);
			// we should erase 4 KB block
			NFLASH_drv_sector_erasure(addr, ERASE_4K);
			addr += 4096;
			page_to_erase -= 1;
		}
	}
#endif

}

I16U FLASH_erase_file_system()
{
	I16U page_end;
	I32U addr = 0;

	// FAT memory space clean up, exclude the APP space.
	page_end = FAT_FLASH_PAGE_ALL-FAT_CLING_APP_PAGE_ALL;

	N_SPRINTF("+++ erase application data: %d, %d", page_end, addr);
	_core_erase(page_end, addr);

	return page_end;
}

I16U FLASH_erase_application_data(BOOLEAN b_erase_auth)
{
	I16U page_end = 0;
	I32U addr = 0;

	if (!b_erase_auth) {
		N_SPRINTF("[FLASH] Erase %d application blocks (4 KB) ", page_end);
		// user doesn't want to erase auth, we go return (not erasue any data on the Flash)
		return page_end;
	}

	// Otherwise, we erase everything

	// FAT memory space clean up, exclude the APP space.
	addr = CLING_APP_SPACE;
	page_end = FAT_CLING_APP_PAGE_ALL;

	N_SPRINTF("+++ erase application data: %d, %d", page_end, addr);
	_core_erase(page_end, addr);

	N_SPRINTF("[FLASH] Erase %d application blocks (4 KB) ", page_end);

	return page_end;
}

I16U FLASH_erase_all(BOOLEAN b_erase_auth)
{
	I16U page_erased;

	page_erased = FLASH_erase_file_system();
	page_erased += FLASH_erase_application_data(b_erase_auth);

	N_SPRINTF("[FLASH] Erase %d blocks (4 KB) ", page_erased);

	return page_erased;
}

