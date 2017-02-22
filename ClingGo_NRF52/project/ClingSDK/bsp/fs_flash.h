#ifndef __FS_FLASH_HEADER__
#define __FS_FLASH_HEADER__

#define _FLASH_ENABLED_
enum {
	FS_SECTION_BOOT,
	FS_SECTION_FAT,
	FS_SECTION_ROOT,
	FS_SECTION_DATA,
	FS_SECTION_APP,
};

I16U FLASH_erase_all(BOOLEAN b_erase_auth);

void FLASH_read_data(I32U cluster, I8U *cbBuffer);
void FLASH_write_data(I32U cluster, I8U *cbBuffer);

void FLASH_Read_FAT(I32U cluster, I8U * cbBuffer);
void FLASH_Write_FAT(I32U cluster, I8U * cbBuffer);

void FLASH_Read_ROOT(I32U cluster, I8U * cbBuffer);
void FLASH_Write_ROOT(I32U cluster, I8U * cbBuffer);

void FLASH_Read_BS(I32U cluster, I8U * cbBuffer);
void FLASH_Write_BS(I32U cluster, I8U * cbBuffer);

void FLASH_erase_App(I32U add);
void FLASH_Read_App(I32U add, I8U * cbBuffer, I8U len);
void FLASH_Write_App(I32U add, I8U * cbBuffer, I8U len);

void FLASH_Read_Flash(I32U addr, I8U *cbBuffer, I8U len);
void FLASH_Write_Flash(I32U addr, I8U *cbBuffer, I8U len);

I16U FLASH_erase_application_data(BOOLEAN b_erase_auth);
I16U FLASH_erase_file_system(void);

#endif
