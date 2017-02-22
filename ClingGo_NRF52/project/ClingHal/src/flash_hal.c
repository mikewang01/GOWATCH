
/******************************************************************************
 * Copyright 2013-2016 hicling Systems (MikeWang)
 *
 * FileName: spi_flash_hal.c
 *
 * Description:  abstract a layer between rtos and cling sdk for flash driver.
 *
 * Modification history:
 *     2016/7/23, v1.0 create this file mike.
 *******************************************************************************/
#include "oop_hal.h"
#include "flash_hal.h"
#include "main.h"
#include "flatform_dev.h"
/*********************************************************************
* MACROS
*/



/*********************************************************************
* TYPEDEFS
*/


/*********************************************************************
* GLOBAL VARIABLES
*/

/*********************************************************************
* LOCAL VARIABLES
*/
//static nvms_t nvms_handle = NULL;
/*********************************************************************
* LOCAL DECLARATION
*/
static int HalFlash_init(CLASS(HalFlash) *arg);
static int write_enable(CLASS(HalFlash) *arg);
static int write_disable(CLASS(HalFlash) *arg);
static int get_status_register(CLASS(HalFlash) *arg);
static int read_data(CLASS(HalFlash) *arg, uint32_t addr, uint16_t len, uint8_t *data_buf);
static int page_program(CLASS(HalFlash) *arg, uint32_t addr, uint16_t len, uint8_t *data_buf);
static int erase_one_block_4k(CLASS(HalFlash) *arg, uint32_t addr);
static int erase_one_block_32k(CLASS(HalFlash) *arg, uint32_t addr);
static int erase_one_block_64k(CLASS(HalFlash) *arg, uint32_t addr);
static int erase_all(CLASS(HalFlash) *arg);
static size_t get_erase_size(CLASS(HalFlash) *arg);
//static size_t get_page_size(CLASS(HalFlash) *arg);
static size_t get_size(CLASS(HalFlash) *arg);
static CLASS(HalFlash) *flash_instance = NULL;
static int erase_all(CLASS(HalFlash) *arg);
/******************************************************************************
* FunctionName :HalFlash_get_instance
* Description  : get flash hal instance
* Parameters   : none
* Returns      : CLASS(HalFlash)*: spi flash instance object pointer
*******************************************************************************/
CLASS(HalFlash)* HalFlash_get_instance(void)
{
        static CLASS(HalFlash) p;
        if (flash_instance == NULL) {
                flash_instance = &p;
                memset(&p, 0, sizeof(p));
                HalFlash_init(flash_instance);
        }
        return flash_instance;
}
/******************************************************************************
* FunctionName : HalFlash_init
* Description  : internally used to functuon as constuction function
* Parameters   : CLASS(HalFlash) *arg : flash object pointer
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static int HalFlash_init(CLASS(HalFlash) *arg)
{
        if(arg == NULL){
                return -1;
        }
        arg->write_enable = write_enable;
        arg->write_disable = write_disable;
        arg->get_status_register = get_status_register;
        arg->get_size = get_size;
        arg->read_data = read_data;
        arg->page_program = page_program;
        arg->erase_one_block_4k = erase_one_block_4k;
        arg->erase_one_block_64k = erase_one_block_64k;
        arg->erase_one_block_32k = erase_one_block_32k;
        arg->get_erase_size = get_erase_size;
        arg->erase_all = erase_all;
        /*initialte nvms devices*/
        //ad_nvms_init();
       // nvms_handle = ad_nvms_open(NVMS_GENERIC_PART);
//        if(nvms_handle == NULL){
//                Y_SPRINTF("[HALFLASH] NVMS_GENERIC_PART Opened Failed");
//                return -1;
//        }
        Y_SPRINTF("[HALFLASH] NVMS_GENERIC_PART Opened sucessfully");
        return 0;
}

/******************************************************************************
* FunctionName : write_enable
* Description  : internally used enbale spi flash
* Parameters   : CLASS(HalFlash) *arg : flash object pointer
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static int write_enable(CLASS(HalFlash) *arg)
{
        return 0;
}
static int write_disable(CLASS(HalFlash) *arg)
{
        return 0;
}

static int get_status_register(CLASS(HalFlash) *arg)
{
        return 0;
}
/******************************************************************************
* FunctionName : read_data
* Description  : internally used read a specifc legth data from adress
* Parameters   : CLASS(HalFlash) *arg : flash object , ,
*                uint32_t addr : FLASH START ADRESS
*                uint16_t len : DATA LENGTH
*                uint8_t *data_buf: DATA BUFFER POINTER
* Returns      : data lenth readed
*******************************************************************************/
static int read_data(CLASS(HalFlash) *arg, uint32_t addr, uint16_t len, uint8_t *data_buf)
{
//        if (arg == NULL || nvms_handle == NULL) {
//                return -1;
//        }
        return  1;//ad_nvms_read(nvms_handle, addr, data_buf, len);
}
/******************************************************************************
* FunctionName : page_program
* Description  : internally used PROGRAM DATA TO A SPEFIC PAGE
* Parameters   : CLASS(HalFlash) *arg : flash object , ,
*                uint32_t addr : FLASH START ADRESS
*                uint16_t len : DATA LENGTH
*                uint8_t *data_buf: DATA BUFFER POINTER
* Returns      : data size writed into flash sucessfully
*******************************************************************************/
static int page_program(CLASS(HalFlash) *arg, uint32_t addr, uint16_t len, uint8_t *data_buf)
{
        return 1;//ad_nvms_write(nvms_handle, addr, data_buf, len);;
}

/******************************************************************************
* FunctionName : erase_one_block_4k
* Description  :  internally used erase 4k data sequensely after addr
* Parameters   : CLASS(HalFlash) *arg : flash object , ,
*                uint32_t addr : FLASH START ADRESS
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static int erase_one_block_4k(CLASS(HalFlash) *arg, uint32_t addr)
{
//        addr/=(arg->get_erase_size(arg));
//        addr*=(arg->get_erase_size(arg));
//#define __FLASH_4K_LENTH__  ((4*1024))
//        N_SPRINTF("[hal_flash]:erase 64k start \r\n");
//        uint32_t end_addr = addr + __FLASH_4K_LENTH__;
//        uint32_t length = 0;
//        length = __FLASH_4K_LENTH__;
//        if(end_addr > 0 + ad_nvms_get_size(nvms_handle)){
//                length = ad_nvms_get_size(nvms_handle) - addr;
//        }
//        ad_nvms_erase_region(nvms_handle, addr, length);
        return 0;
}
/******************************************************************************
* FunctionName : erase_one_block_32k
* Description  : internally used erase 32k data sequensely after addr
* Parameters   : CLASS(HalFlash) *arg : flash object , ,
*                uint32_t addr : FLASH START ADRESS
* Returns      : data size writed into flash sucessfully
*******************************************************************************/
static int erase_one_block_32k(CLASS(HalFlash) *arg, uint32_t addr)
{
//        addr/=(arg->get_erase_size(arg));
//        addr*=(arg->get_erase_size(arg));
//#define __FLASH_32K_LENTH__  ((32*1024))
//        Y_SPRINTF("[hal_flash]:erase 32k start \r\n");
//        uint32_t end_addr = addr + __FLASH_32K_LENTH__;
//        uint32_t length = 0;
//        length = __FLASH_32K_LENTH__;
//        if(end_addr > 0 + ad_nvms_get_size(nvms_handle)){
//                length = ad_nvms_get_size(nvms_handle) - addr;
//        }
//        if(ad_nvms_erase_region(nvms_handle, addr, length) == true){
//                return 0;
//        }else{
//                return -1;
//        }
}
/******************************************************************************
* FunctionName : erase_one_block_64k
* Description  : internally used erase 64k data sequensely after addr
* Parameters   : CLASS(HalFlash) *arg : flash object , ,
*                uint32_t addr : FLASH START ADRESS
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static int erase_one_block_64k(CLASS(HalFlash) *arg, uint32_t addr)
{
#define __FLASH_64K_LENTH__  ((64*1024))
//        /*aligned check*/
//        addr/=(arg->get_erase_size(arg));
//        addr*=(arg->get_erase_size(arg));
//        Y_SPRINTF("[hal_flash]:erase 64k start \r\n");
//        uint32_t end_addr = addr + __FLASH_64K_LENTH__;
//        uint32_t length = 0;
//        length = __FLASH_64K_LENTH__;
//        /*boder check*/
//        if(end_addr > 0 + ad_nvms_get_size(nvms_handle)){
//                length = ad_nvms_get_size(nvms_handle) - addr;
//        }
//        if(ad_nvms_erase_region(nvms_handle, addr, length) == true){
//                return 0;
//        }else{
//                return -1;
//        }
}
/******************************************************************************
* FunctionName : erase_all
* Description  : internally used erase all data in flash
* Parameters   : CLASS(HalFlash) *arg
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static int erase_all(CLASS(HalFlash) *arg)
{

//        if(ad_nvms_erase_region(nvms_handle, 0, ad_nvms_get_size(nvms_handle)) == true){
//                return 0;
//        }else{
//                return -1;
//        }
}

/******************************************************************************
* FunctionName : get_erase_size
* Description  : internally used get flash minimum flash erase size
* Parameters   : CLASS(HalFlash) *arg
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static size_t get_erase_size(CLASS(HalFlash) *arg)
{
      //  return ad_flash_erase_size();
}



/******************************************************************************
* FunctionName : get_size
* Description  : internally used get flash minimum flash erase size
* Parameters   : CLASS(HalFlash) *arg
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static size_t get_size(CLASS(HalFlash) *arg)
{
       // return ad_nvms_get_size(nvms_handle);
	return 1;
}


#define _NOR_FLASH_SPI_TEST_
#ifdef _NOR_FLASH_SPI_TEST_
void _nor_flash_spi_test()
{
    uint32_t  i, j, err_cnt;
    uint32_t add = 0;
    uint32_t buf[32];
    uint8_t *temp;
    uint32_t index, len, cnt;

    // Nor Flash initialize
    CLASS(HalFlash) *p = HalFlash_get_instance();

    // Read ID
    //NOR_readID((I8U *)buf);
   // temp = (I8U *)buf;
   // Y_SPRINTF("ID: %02x, %02x", temp[0], temp[1]);

    // Erase the 256 KB
#if 1
    add = 0;
    for (i = 0; i < 2; i ++) {
        Y_SPRINTF("[FS] erase: %d, 64K\r\n", add);
        p->erase_one_block_64k(p, add);
//        NOR_erase_block_64k(add);
        add += 65536;
        //Watchdog_Feed();
    }
#else
    NOR_ChipErase();

#endif
    temp = (uint8_t *)buf;

    // Read out one byte just to confirm
    p->read_data(p, 0x000fff, 1, temp);
    Y_SPRINTF("Value at 0x00ffff: 0x%02x\r\n", temp[0]);

    cnt = 1000;

    // Program Flash, 1000x128 = 125 K
    add = 0;
    len = 128;
    for (i = 0; i < cnt; i++) {
        index = 0;
        for (j = 0; j < len; j++) {
            temp[j] = index ++;
        }
        p->page_program(p, add, len, temp);
//        NOR_pageProgram(add, len, temp);
        add += len;
//        Watchdog_Feed();
    }

    // Read out the data
    add = 0;
    len = 128;
    err_cnt = 0;
    for (i = 0; i < cnt; i++) {
        temp = (uint8_t *)buf;
        p->read_data(p, add, len, temp);
//        NOR_readData(add, len, temp);
        for (j = 0; j < len; j++) {
            if (j != temp[j]) {
                err_cnt ++;
                Y_SPRINTF("[MAIN] %d, %d\r\n", j, temp[j]);
            }
        }
        /*
        temp = (I8U *)&buf[16];
                N_SPRINTF("%02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x,",
                        temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6], temp[7]);
                */
        add += len;
//        Watchdog_Feed();
    }

    Y_SPRINTF("----  test nor flash: %d errors\r\n", err_cnt);

#if 0
    // Erase the whole chip
    add = 0;
    NOR_ChipErase();

    // Write the data
    add = 0;
    for (i = 0; i < 1; i++) {
        index = 0;
        for (j = 0; j < 128; j++) {
            temp[j] = index ++;
        }
        NOR_pageProgram(add, 32, (I8U *)buf);
        add += 128;
    }

    // Read out the data
    add = 0;
    for (i = 0; i < 30; i++) {
        NOR_readData(add, 8, (I8U *)buf);
        N_SPRINTF("%02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x,",
                  temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6], temp[7]);
        add += 8;
    }
#endif
    return;

    // Read out the data
    add = 0;
    err_cnt = 0;
    for (i = 0; i < 4096; i++) {
        index = 0;
        for (j = 0; j < 32; j++) {
            buf[j] = 0;
        }
        p->page_program(p, add, 128, (uint8_t*)buf);
//        NOR_readData(add, 128, (I8U *)buf);

        for (j = 0; j < 128; j++) {
            if (j != temp[j]) {
                err_cnt ++;
                //N_SPRINTF("%02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x,",
                //temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6], temp[7]);
            }
        }

        add += 128;
    }
    Y_SPRINTF("---- first test: %d errors", err_cnt);

#if 0

    N_SPRINTF("---- second test ---------");
    // Erase 32 K block
    add = 0;
    NOR_ChipErase();

    // Write the data
    add = 0;
    index = 0x11;
    for (i = 0; i < 30; i++) {
        for (j = 0; j < 8; j++) {
            temp[j] = index ++;
        }
        NOR_pageProgram(add, 8, (I8U *)buf);
        add += 8;
    }

    // Read out the data
    add = 0;
    for (i = 0; i < 30; i++) {
        NOR_readData(add, 8, (I8U *)buf);
        N_SPRINTF("%02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x,",
                  temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6], temp[7]);
        add += 8;
    }
#endif
}
#endif

