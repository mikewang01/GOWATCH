//#include "main.h"
//#include "sysflash_rw.h"
//
//I8U g_system_state;
//
//
//void SYSFLASH_drv_init()
//{
//	g_system_state = SYSFLASH_STATE_NOP;
//}
//
//// Note: according to nordic spec, it takes typical 21 ms to erase a page,
//// be very careful.
//I32U SYSFLASH_drv_sector_erasure(I32U addr)
//{
//	I32U err_code=0;
//#ifndef _CLING_PC_SIMULATION_
//	I32U flash_page_num = addr+FAT_FLASH_BASE_ADDR;
//
//	flash_page_num >>= 10;// 1024 bytes per page
//
//  err_code = ble_flash_page_erase(flash_page_num);
//#endif
//	return err_code;
//}
//
//void SYSFLASH_drv_event_handler(uint32_t sys_evt)
//{
//	if (g_system_state == SYSFLASH_STATE_ERASURE) {
//		N_SPRINTF("[SYSTEM] Flash operation: erasure with event %d", sys_evt);
//	} else if (g_system_state == SYSFLASH_STATE_WRITE) {
//		N_SPRINTF("[SYSTEM] Flash operation: write with event %d", sys_evt);
//	} else {
//		N_SPRINTF("[SYSTEM] Flash operation: NONE with event %d", sys_evt);
//	}
//
//	g_system_state = SYSFLASH_STATE_NOP;
//}
//
//static void _flash_op_wait()
//{
//#ifndef _CLING_PC_SIMULATION_
//	I8U i;
//
//	for (i = 0; i < 100; i++) {
//		if (g_system_state == SYSFLASH_STATE_NOP)
//			break;
//		else
//			BASE_delay_msec(2);
//	}
//#endif
//}
//
//void SYSFLASH_drv_page_erasure(I16U page_num)
//{
//#ifndef _CLING_PC_SIMULATION_
//	g_system_state = SYSFLASH_STATE_ERASURE;
//	sd_flash_page_erase(page_num);
//	_flash_op_wait();
//#endif
//}
//
//void SYSFLASH_drv_write(I32U *dst, I32U *src, I32U size)
//{
//#ifndef _CLING_PC_SIMULATION_
//	g_system_state = SYSFLASH_STATE_WRITE;
//	sd_flash_write(dst, src, size);
//	_flash_op_wait();
//#endif
//}
//
//// ---------------------------------------------------------------------------------------
//// Function:   SYSFLASH_read_data()
////
//// Description:
////    Read out data from NFLASH directly!
////
//// Parameter:
////    add: the physical address in flash
////    len: data bytes
////    out: output buffer
////
//// Note:
////    All corrsponding register is only writable, can't be read!
//// ---------------------------------------------------------------------------------------
//void SYSFLASH_drv_read_sector(I32U addr, I8U *out, I16U len)
//{
//	I16U i;
//	I8U *flash_addr = (I8U *)(FAT_FLASH_BASE_ADDR + addr);
//
//	for (i = 0; i < len; i++) {
//		*out++ = *flash_addr++;
//	}
//}
//
//void SYSFLASH_drv_write_sector(I32U addr, I8U *in, I16U len)
//{
//#ifndef _CLING_PC_SIMULATION_
//	I16U num_in_32b = len >> 2;
//	I8U *flash_addr = (I8U *)(FAT_FLASH_BASE_ADDR + addr);
//
//	ble_flash_block_write((I32U *)flash_addr, (I32U *)in, num_in_32b);
//#endif
//}
//
