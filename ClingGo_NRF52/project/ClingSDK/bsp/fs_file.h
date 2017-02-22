#ifndef __FILE_HEADER__
#define __FILE_HEADER__

#include "standard_types.h"
#include "fs_memcfg.h"

#define  _FILE_SYS_ENABLED_
#define MAX_FILE_NAME_LEN	79

typedef enum {
	FILE_TYPE_ACTIVITY,
	FILE_TYPE_SYSTEM_LOG,
	FILE_TYPE_STORAGE,
	FILE_TYPE_UNKNOWN,
} FILE_TYPE;

enum {
	FILE_IO_WRITE,
	FILE_IO_READ,
};

typedef struct tagFILE_CTX {
	I32U  tell;
	I32U  size;
	I16U root_pos;
	I16U cur_cluster;
	I16U begin_cluster;
	I16U file_idx;			// internal file index
	I16U crc;
	I8U  buf_pos;
	I8U io_mode;			// read/write mode
	BOOLEAN fw_on;			// True: if a file write just gets started, and set to false upon FCLOSE.
} FILE_CTX;

typedef struct tagCLING_FILE {
	FILE_TYPE type;
	FILE_CTX *fc;

} CLING_FILE;

enum {
	FILE_OP_NONE,
	FILE_OP_READ,
	FILE_OP_WRITE,
	FILE_OP_DELETE,
	FILE_OP_ERASURE
};

enum {
	FILE_WRITE_STATE_IDLE,
	FILE_WRITE_STATE_WRITING,
	FILE_WRITE_STATE_COMPLETED,
	FILE_WRITE_STATE_ERROR
};

enum {
	FILE_READ_STATE_IDLE,
	FILE_READ_STATE_READING,
	FILE_READ_STATE_COMPLETED,
	FILE_READ_STATE_ERROR
};

typedef struct tagTIMESET {
	I16U  year;
	I16U  month;
	I16U  dayOfWeek;
	I16U  day;
	I16U  hour;
	I16U  minute;
	I16U  second;
	I16U  milisecond;
} TIMESET;

// We surpport Win9x long file names up to 128 bytes in length
#define CLING_MAX_FILENAME	FAT_SECTOR_SIZE

// enum Attribute {normal = 0x00, readOnly = 0x01, hidden = 0x02, system = 0x04, archive = 0x20};
//
//	Normal:		0	Normal file. No attributes are set. 
//	ReadOnly:	1	Read-only file. Attribute is read/write. 
//	Hidden:		2	Hidden file. Attribute is read/write. 
//	System:		4	System file. Attribute is read/write. 
//	Volume:		8	Disk drive volume label. Attribute is read-only. 
//	Directory:	16	Folder or directory. Attribute is read-only. 
//	Archive:	32	File has changed since last backup. Attribute is read/write. 
//	Alias:		64	Link or shortcut. Attribute is read-only. 
//	Compressed:	128	Compressed file. Attribute is read-only. 
//
#define FILE_FLAG_READONLY	0x01
#define FILE_FLAG_ARCHIVE	0x20

typedef enum {
	FILE_ERR_NONE,
	FILE_ERR_DISCONNECTED,
	FILE_ERR_LOW_MEMORY,
	FILE_ERR_CORRUPTED_DATA,
	FILE_ERR_FILE_FORMAT,
	FILE_ERR_CREATE_FILE,
	FILE_ERR_WRITE_FILE,
	FILE_ERR_READ_FILE,
	FILE_ERR_INVALID_PARAMETER,
	FILE_ERR_INVALID_OPERATION,
	FILE_ERR_WRONG_DATA
} CLING_FS_ERR_CODE;

//
// File system initialization
//
void FILE_init(void);
//
// Open or create a file with a given file name.
//
FILE_CTX *FILE_fopen(I8U * file_name, unsigned char fIO);
//
// Close a file
//
void FILE_fclose(FILE_CTX *fc);
//
// Check if the end of a file
//
I8U FILE_feof(FILE_CTX *fc);
//
// Read from a file
//
I8U FILE_fread(FILE_CTX *fc, I8U * buffer, I8U count);
//
// Write to a file
//
I8U FILE_fwrite(FILE_CTX *fc, I8U *in, I8U count);
//
// Tell the file position
//
I32U FILE_ftell(FILE_CTX *f);
// 
// Read out a line of text
//
I8U * FILE_fgets (I8U * str, I8U num, FILE_CTX *fc);
//
// Check whether a file is existed in the file system
//
BOOLEAN FILE_if_exists(I8U *file_name);
//
// Get amount of files with known prefix
//
I32U FILE_exists_with_prefix(I8U *file_name, I8U character_num);
//
// Get next file with prefix
//
I32U FILE_get_next_file_with_prefix(I8U *file_name, I8U character_num, BOOLEAN b_delete_first_one);
//
// Delete a file
//
void FILE_delete(I8U *file_name);
//
// Get total file number
//
I16U FILE_GetFileNum(I32U *msg_len);
//
// Load file info
//
I32U FILE_getFileInfo(I8U f_id, I8U *buf, I32U buf_pos);
//
// Force close out a file
//
void FILE_fclose_force(void);
#endif

