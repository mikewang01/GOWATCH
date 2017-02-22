/************************************************************************/
// 
// CLING FILE SYSTEM - FAT system file operations
//
// Created on Jan 03, 2014
//
/************************************************************************/
#include <string.h>
#include <stdio.h>
#include "stdlib.h"

#include "main.h"

#include "fs_file.h"
#include "fs_root.h"
#include "fs_memcfg.h"

/* internal CLING file context. */
FILE_CTX pbl_file;

// Static cache space, minimize read/write the NOR FLASH operations
I8U NFLASH_pCache_wr[FAT_SECTOR_SIZE];
I8U NFLASH_pCache_rd[FAT_SECTOR_SIZE];

void _step_read_index(FILE_CTX *f)
{
	I16U next_cluster;

	// Get next available FAT cluster
	next_cluster = FAT_GetNextChain(f->cur_cluster);
	if ((next_cluster != (I16U)0xffff) && 
		(next_cluster != (I16U)0x7fff) &&
		(next_cluster != (I16U)0x3fff))
	{
		// A valid cluster is found
		f->cur_cluster = next_cluster;
	}

}

static void _read_flash_cache(FILE_CTX *f)
{
	// Read out one sector
	FLASH_read_data(f->cur_cluster, NFLASH_pCache_rd);

	// Ready for reading next sector, which updates "cur_cluster"
	_step_read_index(f);

	// Buffer position reset to 0.
	f->buf_pos = 0;
}

/* internal functions used in p_fwrite() */

void FILE_init()
{
	// clear global file context
	memset(&pbl_file, 0, sizeof(FILE_CTX));

	// Init the FATs
	FAT_Init();

	// Init the Root Dir.
	ROOT_init();
	
}

BOOLEAN FILE_if_exists(I8U *file_name) 
{
	ROOT_ITEM root_item;
	I32U f_total;
	I32U name_str[32];
	I32U i;

	// if we are in middle of file write, we then close out this file
	FILE_fclose_force();

	f_total = ROOT_ListFiles();

	if (!f_total) return FALSE;

	for (i = 0; i < f_total; i++) {
		if (ROOT_GetFileInfo(i, (I8U *)&root_item)) {
			// Get file name ...
			ROOT_GetFileName(i, (I8U *)name_str);
			if (!strcmp((char *)name_str, (char *)file_name)) {
				return TRUE;
			}
		}
	}

	return FALSE;
}

I32U FILE_exists_with_prefix(I8U *file_name, I8U character_num) 
{
	ROOT_ITEM root_item;
	I32U f_total;
	I32U name_str[32];
	I32U i, j, cnt;
	I8U *pname;
	BOOLEAN b_file_found;

	// Reset amount
	cnt = 0;
	
	if (!file_name || (character_num == 0))
		return cnt;
	
	// if we are in middle of file write, we then close out this file
	FILE_fclose_force();

	f_total = ROOT_ListFiles();

	if (!f_total) return cnt;

	pname = (I8U *)name_str;
	
	for (i = 0; i < f_total; i++) {
		if (ROOT_GetFileInfo(i, (I8U *)&root_item)) {
			// Get file name ...
			ROOT_GetFileName(i, (I8U *)name_str);
			b_file_found = TRUE;
			for (j = 0; j < character_num; j++) {
				if (pname[j] != file_name[j]) {
					b_file_found = FALSE;
					break;
				}
			}
			
			if (b_file_found) {
				b_file_found = FALSE;
				cnt ++;
				N_SPRINTF("--- Found %d minute files: %s", cnt, pname);
			}
		}
	}

	return cnt;
}

I32U FILE_get_next_file_with_prefix(I8U *file_name, I8U character_num, BOOLEAN b_delete_first_one)
{
	ROOT_ITEM root_item;
	I32U f_total;
	I32U name_str[32];
	I32U i, j;
	I32U addr = 0;
	I8U *pname;
	BOOLEAN b_file_found;
	BOOLEAN b_first_file=TRUE;

	// if we are in middle of file write, we then close out this file
	FILE_fclose_force();

	// Delete the first file with this prefix
	f_total = ROOT_ListFiles();

	if (!f_total) return addr;

	pname = (I8U *)name_str;

	for (i = 0; i < f_total; i++) {
		if (ROOT_GetFileInfo(i, (I8U *)&root_item)) {
			// Get file name ...
			ROOT_GetFileName(i, (I8U *)name_str);
			b_file_found = TRUE;
			for (j = 0; j < character_num; j++) {
				if (pname[j] != file_name[j]) {
					b_file_found = FALSE;
					break;
				}
			}

			if (b_file_found) {
				if (b_delete_first_one && b_first_file) {
					ROOT_DeleteFile(i);
					b_first_file = FALSE;
				}
				else
				{
					FILE_CTX *fc;
					fc = FILE_fopen(pname, FILE_IO_READ);
					if (fc)
						addr = (fc->begin_cluster + 512) << 7;
					FILE_fclose(fc);

					break;
				}
			}
		}
	}

	return addr;
}

void FILE_delete(I8U *file_name)
{
	ROOT_ITEM root_item;
	I16U f_total, i;
	I32U name_str[32];

	// if we are in middle of file write, we then close out this file
	FILE_fclose_force();

	f_total = ROOT_ListFiles();

	if (!f_total) return;

	for (i = 0; i < f_total; i++) {

		if (ROOT_GetFileInfo(i, (I8U *)&root_item)) {
			// Get file name ...
			ROOT_GetFileName(i, (I8U *)name_str);
			if (!strcmp((char *)name_str, (char *)file_name)) {
				ROOT_DeleteFile(i);
				// In case that we have multiple files with same name, remove all of them
				//break;
			}
		}
	}
}

I32U FILE_getFileInfo(I8U f_id, I8U *buf, I32U buf_pos)
{
	I32U f_total;
	I16U f_valid = 0;
	ROOT_ITEM root_item;
	I32U name_str[32];
	I8U *p_name_str;
	I16U i, f_name_len, max_name_len;

	// if we are in middle of file write, we then close out this file
	FILE_fclose_force();

	// Get current total files
	f_total = ROOT_ListFiles();

	for (i = 0; i < f_total; i++) {

		if (ROOT_GetFileInfo(i, (I8U *)&root_item)) {

			if (f_valid == f_id) {
				// 1/ get file length
				buf[buf_pos++] = (root_item.size >> 24) & 0xff;
				buf[buf_pos++] = (root_item.size >> 16) & 0xff;
				buf[buf_pos++] = (root_item.size >> 8) & 0xff;
				buf[buf_pos++] = root_item.size & 0xff;

				// 2/ Get file CRC
				buf[buf_pos++] = (root_item.crc>>8) & 0xff;
				buf[buf_pos++] = root_item.crc & 0xff;

				// 3/ Get name
				p_name_str = (I8U *) name_str;
				
				// Get file name ...
				ROOT_GetFileName(i, p_name_str);

				// Get file name length
				f_name_len = strlen((char *)p_name_str)+1;
				
				// The buffer length is limited to 128 bytes, so that we have to limit file name length here
				max_name_len = 127 - buf_pos;
				if (f_name_len > max_name_len)
					f_name_len = max_name_len;
				p_name_str[f_name_len] = 0;
				
				buf[buf_pos++] = (I8U)f_name_len;
				
				// Copy file name string
				memcpy(buf+buf_pos, p_name_str, f_name_len);
				buf_pos += f_name_len;

				return buf_pos;
			}

			f_valid ++;
		}
	}

	return buf_pos;
}

I16U FILE_GetFileNum(I32U *msg_len)
{
	I16U f_valid = 0;
	I32U f_name_len = 0;
	I32U f_total;
	ROOT_ITEM root_item;
	I32U name_str[32];
	I16U i;

	// if we are in middle of file write, we then close out this file
	FILE_fclose_force();

	f_total = ROOT_ListFiles();

	*msg_len = 0;
	if (!f_total) return 0;

	for (i = 0; i < f_total; i++) {

		if (ROOT_GetFileInfo(i, (I8U *)&root_item)) {
			f_valid ++;
			// Get file name ...
			ROOT_GetFileName(i, (I8U *)name_str);
			f_name_len += strlen((char *)name_str)+1; // name, and zero delimited
			f_name_len += 4; // File size (4 bytes)
			f_name_len += 2; // File CRC (2 bytes)
			f_name_len += 1; // file name length
			// We could potentially add some extra file info here, such as time.
		}
	}

	// 2-byte: number of files
	*msg_len = f_name_len;

	return f_valid;
}

FILE_CTX *FILE_fopen(I8U *fname, I8U fIO)
{
	ROOT_ITEM root_item;
	I16S iPos;
	I16U name_fat, data_fat;
	I32U lSize, i, f_total;
	I32U dw_buf[32];
	I32U dw_name[32];
	I8U *file_name;
	I8U *name_str;
	I32U free;
	
	// Dword alignment
	name_str = (I8U *)dw_buf;
	
	file_name = (I8U *)dw_name;
	
	memcpy(dw_name, fname, strlen((char*)fname)+1);
	
	// if we are in middle of file write, we then close out this file
	FILE_fclose_force();

	pbl_file.io_mode = fIO;

	if (fIO == FILE_IO_READ) {
		f_total = ROOT_ListFiles();

		if (!f_total) return 0;

		for (i = 0; i < f_total; i++) {

			if (ROOT_GetFileInfo(i, (I8U *)&root_item)) {
				// Get file name ...
				ROOT_GetFileName(i, (I8U *)name_str);
				if (!strcmp((char *)name_str, (char *)file_name)) break;
			}
		}

		if (i < f_total)
			pbl_file.file_idx = i;
		else
			return 0;

		// Read out this file.
		pbl_file.crc = ROOT_OpenReadFile(pbl_file.file_idx, &data_fat, &lSize);
		pbl_file.buf_pos = FAT_SECTOR_SIZE;
		pbl_file.root_pos = pbl_file.file_idx;		// pos: 1 ~ 1024, one per file
		pbl_file.cur_cluster = data_fat;
		pbl_file.begin_cluster = data_fat;
		pbl_file.tell = lSize;
		pbl_file.size = lSize;
		pbl_file.fw_on = FALSE;			// Not a file write
		return &pbl_file;
	}
	else {
		N_SPRINTF("[FS] fopen (write): %s", file_name);

		// Before we create a file, let's re-init FAT
		FAT_Init();

		// Check free space
		free = FAT_CheckSpace();

		if (free < 10240)
			return 0;

		// find an empty FAT entry as name cluster
		name_fat = FAT_GetEmptyEntry();
		if (name_fat == 0) {
			return 0;	// no free FLASH space left
		}

		// the first data cluster
		data_fat = FAT_GetEmptyEntry();
		if (data_fat == 0) {
			FAT_ClearEntries(name_fat);
			return 0;
		}

		// Create file with a root position
		iPos = ROOT_CreateFile(file_name, name_fat, data_fat);
		if (iPos == -1) {
			FAT_ClearEntries(name_fat);
			FAT_ClearEntries(data_fat);
			return 0;
		}
		// Before we start file write, set the CRC seed
		pbl_file.crc = 0xFFFF;
		pbl_file.buf_pos = 0;
		pbl_file.root_pos = iPos;
		pbl_file.cur_cluster = data_fat;
		pbl_file.begin_cluster = data_fat;
		pbl_file.tell = 0;

		// fill file entry in the root directory.
		pbl_file.file_idx = iPos;

		// Flag file write as we start writting data into this file
		pbl_file.fw_on = TRUE;

		return &pbl_file;
	}
}

void FILE_fclose_force()
{
	if (pbl_file.fw_on) {
		FILE_fclose(&pbl_file);

		// Any forced file closure will clear over the air update flag
		OTA_set_state(FALSE);
	}
}

I8U FILE_fread(FILE_CTX *f, I8U * buffer, I8U count)
{
	I8U cur, remain, in;
	I8U * buf;

	if (!f) return 0;

	in = 0;
	
	buf = buffer;
	
	if (count > f->tell)
		count = f->tell;

	if (f->buf_pos == FAT_SECTOR_SIZE)
	{
		// Filled up cache from next sector
		_read_flash_cache(f);
	}

	if (f->buf_pos + count < FAT_SECTOR_SIZE)
	{
		memcpy(buffer, NFLASH_pCache_rd + f->buf_pos, count);
		f->buf_pos += count;
		in = count;
	}
	else
	{
		// Read out all bytes "count", if "count" goes beyond
		// actual file size, we fill up buffer with content from the last sector
		cur = FAT_SECTOR_SIZE - f->buf_pos;
		memcpy(buf, NFLASH_pCache_rd + f->buf_pos, cur);
		in += cur;
		buf += cur;
		_read_flash_cache(f);

		while (in < count)
		{
			remain = count - in;
			cur = (remain >= FAT_SECTOR_SIZE) ? FAT_SECTOR_SIZE : remain;
			memcpy(buf, NFLASH_pCache_rd, cur);
			in += cur;
			buf += cur;
			if (cur == FAT_SECTOR_SIZE)
			{
				_read_flash_cache(f);
			}
			else
			{
				f->buf_pos = cur;
			}
		}
	}

	// Update valid data read from the file
	if (f->tell < in)
		in = (I8U) f->tell;

	// Update remainning file size
	f->tell -= in;

	// For the last sector, we return the fractional as the last number bytes remaining
	// in the file
	return in;
}

I32U FILE_ftell(FILE_CTX *f)
{
	return (f->size - f->tell);
}

static I8U _write_to_flash(FILE_CTX *f)
{
	I16U empty_fat;

	// Write one full sector
	FLASH_write_data(f->cur_cluster, NFLASH_pCache_wr);

	f->tell += f->buf_pos;

	if (f->buf_pos < FAT_SECTOR_SIZE) 
		return 1;

	f->buf_pos = 0;

	empty_fat = FAT_GetEmptyEntry();
	if (empty_fat != 0)
	{
		// We got this empty cluster, and flag it in the map
		// that this cluster is taken!
		FAT_BuildChain(f->cur_cluster, empty_fat);
		f->cur_cluster = empty_fat;
	}
	else
	{
		return 0;
	}

	return 1;
}


I8U FILE_feof(FILE_CTX *f)
{
	if (!f) return 0;

	return (f->tell > 0) ? 1 : 0;
}

void FILE_fclose(FILE_CTX *f)
{
	I8U fIO;
	I32U ftime;
	
	if (!f) return;

	fIO = f->io_mode;

	if (fIO == FILE_IO_WRITE) {
		// flush write data
		if (f->buf_pos != 0) {
			_write_to_flash(f);
		}

		// Set file tributes
		ROOT_SetFileType(f->file_idx, ROOT_GetFileType(f->file_idx) | FILE_TYPE_ACTIVITY);
		
		N_SPRINTF("[FS] file close, crc: %04x", f->crc);

		// Set CRC
		ROOT_SetFileCRC(f->file_idx, f->crc);

		// Set file time - current local time?
		ftime = cling.time.local.year;
		ftime <<= 16;
		ftime |= cling.time.local.month;
		ftime <<= 8;
		ftime |= cling.time.local.day;
		ROOT_SetFileTime(f->file_idx, ftime);
		ROOT_CloseWriteFile(f->file_idx, f->tell);

		// Flush root items and FAT content, these are important since it
		// is only portion of one sector, and file will get ignored if otherwise
		// Crashing point
		FAT_flush_map();
		
		// For a root page that has any change, we should write it into NOR flash before closing out this file.
		ROOT_flush_item();

		// Clear fwrite flag
		f->fw_on = FALSE;
		
		// Since we have a file close, update file last position
		ROOT_GetLastPos();
	} 
	
}

//
// The file write is limited at 128 bytes, a large content has to be 
// partitioned and written into file system in sequence.
//
I8U FILE_fwrite(FILE_CTX *f, I8U *in, I8U count)
{
	I8U cur, remain, out;
	I8U *addr = in;
//	I8U *data = in;
	//I8U len;

	if (!f) return 0;
	
	f->crc = SYSTEM_CRC(in, count, f->crc);
	
	out = 0;

	if (f->buf_pos + (I16U) count < FAT_SECTOR_SIZE)
	{
		memcpy(NFLASH_pCache_wr + f->buf_pos, addr, count);

		f->buf_pos += count;
		out += count;
	}
	else
	{
		cur = FAT_SECTOR_SIZE - f->buf_pos;
		memcpy(NFLASH_pCache_wr + f->buf_pos, addr, cur);
		out += cur;
		addr += cur;
		f->buf_pos = FAT_SECTOR_SIZE;
		if(_write_to_flash(f))
		{
			while (out < count)
			{
				remain = count - out;
				cur = (remain >= FAT_SECTOR_SIZE) ? FAT_SECTOR_SIZE : remain;
				memcpy(NFLASH_pCache_wr, addr, cur);
				out += cur;
				addr += cur;
				if (cur == FAT_SECTOR_SIZE)
				{
					f->buf_pos = FAT_SECTOR_SIZE;
					if(!_write_to_flash(f))
						break;
				}
				else
				{
					f->buf_pos = cur;
				}
			}
		}
	}

	return out;
}

I8U * FILE_fgets(I8U * str, I8U count, FILE_CTX *f)
{
	I8U in;
	I8U *buf;

	if (!f) return 0;

	if (!FILE_feof(f)) return 0;

	in = 0;

	if (count > f->tell)
		count = f->tell;
	
	buf = str;

	if (f->buf_pos == FAT_SECTOR_SIZE)
	{
		// Filled up cache from next sector
		_read_flash_cache(f);
	}

	while (in < count) {
		*buf = NFLASH_pCache_rd[f->buf_pos];
		buf ++;
		in ++;
		f->buf_pos++;

		if (f->buf_pos == FAT_SECTOR_SIZE)
		{
			// Filled up cache from next sector
			_read_flash_cache(f);
		}

		// Read out a new line
		if (NFLASH_pCache_rd[f->buf_pos] == 0x0a)
			break;
	}

	// Null terminated
	*buf = 0;

	if (f->tell > in)
		f->tell -= in;
	else
		f->tell = 0;

	return str;
}

