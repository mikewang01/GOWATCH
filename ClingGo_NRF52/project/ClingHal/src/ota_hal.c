
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
#include "ota_hal.h"
#include "main.h"
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
static int HalOta_init(CLASS(HalOta) *arg);
static int write_enable(CLASS(HalOta) *arg);
static int write_disable(CLASS(HalOta) *arg);
static int get_status_register(CLASS(HalOta) *arg);
static int read_data(CLASS(HalOta) *arg, uint32_t addr, uint16_t len, uint8_t *data_buf);
static int page_program(CLASS(HalOta) *arg, uint32_t addr, uint16_t len, uint8_t *data_buf);
static int erase_one_block_4k(CLASS(HalOta) *arg, uint32_t addr);
static int erase_one_block_32k(CLASS(HalOta) *arg, uint32_t addr);
static int erase_one_block_64k(CLASS(HalOta) *arg, uint32_t addr);
static int erase_all(CLASS(HalOta) *arg);
static size_t get_erase_size(CLASS(HalOta) *arg);
//static size_t get_page_size(CLASS(HalOta) *arg);
static size_t get_size(CLASS(HalOta) *arg);
static CLASS(HalOta) *flash_instance = NULL;
static int erase_all(CLASS(HalOta) *arg);
/******************************************************************************
* FunctionName :HalOta_get_instance
* Description  : get flash hal instance
* Parameters   : none
* Returns      : CLASS(HalOta)*: spi flash instance object pointer
*******************************************************************************/
CLASS(HalOta)* HalOta_get_instance(void)
{
        static CLASS(HalOta) p;
        if (flash_instance == NULL) {
                flash_instance = &p;
                memset(&p, 0, sizeof(p));
                HalOta_init(flash_instance);
        }
        return flash_instance;
}
/******************************************************************************
* FunctionName : HalOta_init
* Description  : internally used to functuon as constuction function
* Parameters   : CLASS(HalOta) *arg : flash object pointer
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static int HalOta_init(CLASS(HalOta) *arg)
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
       // nvms_handle = ad_nvms_open(NVMS_FW_UPDATE_PART);
        //if(nvms_handle == NULL){
      //          Y_SPRINTF("[HalOta] NVMS_GENERIC_PART Opened Failed");
    //            return -1;
    //    }
        Y_SPRINTF("[HalOta] NVMS_GENERIC_PART Opened sucessfully");
        return 0;
}

/******************************************************************************
* FunctionName : write_enable
* Description  : internally used enbale spi flash
* Parameters   : CLASS(HalOta) *arg : flash object pointer
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static int write_enable(CLASS(HalOta) *arg)
{
        return 0;
}
static int write_disable(CLASS(HalOta) *arg)
{
        return 0;
}

static int get_status_register(CLASS(HalOta) *arg)
{
        return 0;
}
/******************************************************************************
* FunctionName : read_data
* Description  : internally used read a specifc legth data from adress
* Parameters   : CLASS(HalOta) *arg : flash object , ,
*                uint32_t addr : FLASH START ADRESS
*                uint16_t len : DATA LENGTH
*                uint8_t *data_buf: DATA BUFFER POINTER
* Returns      : data lenth readed
*******************************************************************************/
static int read_data(CLASS(HalOta) *arg, uint32_t addr, uint16_t len, uint8_t *data_buf)
{
        //if (arg == NULL || nvms_handle == NULL) {
     //           return -1;
   //    }
    //    return  ad_nvms_read(nvms_handle, addr, data_buf, len);
}
/******************************************************************************
* FunctionName : page_program
* Description  : internally used PROGRAM DATA TO A SPEFIC PAGE
* Parameters   : CLASS(HalOta) *arg : flash object , ,
*                uint32_t addr : FLASH START ADRESS
*                uint16_t len : DATA LENGTH
*                uint8_t *data_buf: DATA BUFFER POINTER
* Returns      : data size writed into flash sucessfully
*******************************************************************************/
static int page_program(CLASS(HalOta) *arg, uint32_t addr, uint16_t len, uint8_t *data_buf)
{
        return 1;//ad_nvms_write(nvms_handle, addr, data_buf, len);
}

/******************************************************************************
* FunctionName : erase_one_block_4k
* Description  :  internally used erase 4k data sequensely after addr
* Parameters   : CLASS(HalOta) *arg : flash object , ,
*                uint32_t addr : FLASH START ADRESS
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static int erase_one_block_4k(CLASS(HalOta) *arg, uint32_t addr)
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
* Parameters   : CLASS(HalOta) *arg : flash object , ,
*                uint32_t addr : FLASH START ADRESS
* Returns      : data size writed into flash sucessfully
*******************************************************************************/
static int erase_one_block_32k(CLASS(HalOta) *arg, uint32_t addr)
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
* Parameters   : CLASS(HalOta) *arg : flash object , ,
*                uint32_t addr : FLASH START ADRESS
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static int erase_one_block_64k(CLASS(HalOta) *arg, uint32_t addr)
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
* Parameters   : CLASS(HalOta) *arg
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static int erase_all(CLASS(HalOta) *arg)
{

        //if(ad_nvms_erase_region(nvms_handle, 0, ad_nvms_get_size(nvms_handle)) == true){
      //          return 0;
     //   }else{
     //           return -1;
   //     }
	return -1;
}

/******************************************************************************
* FunctionName : get_erase_size
* Description  : internally used get flash minimum flash erase size
* Parameters   : CLASS(HalOta) *arg
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static size_t get_erase_size(CLASS(HalOta) *arg)
{
        return 1;//ad_flash_erase_size();
}



/******************************************************************************
* FunctionName : get_size
* Description  : internally used get flash minimum flash erase size
* Parameters   : CLASS(HalOta) *arg
* Returns      : >=0:sucess
*                <0:failed
*******************************************************************************/
static size_t get_size(CLASS(HalOta) *arg)
{
        return 1;//ad_nvms_get_size(nvms_handle);
}


