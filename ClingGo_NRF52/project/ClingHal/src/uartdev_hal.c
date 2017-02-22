#include <btle_api.h>
/******************************************************************************
 #include <uartdev_hal>
 #include <uartdev_hal>
 * Copyright 2013-2016 hicling Systems (MikeWang)
 *
 * FileName: rtc_hal.c
 *
 * Description: abstract layer between cling sdk and freertos.
 *
 * Modification history:
 *     2016/7/22, v1.0 create this file mike.
 *******************************************************************************/
#include "oop_hal.h"
//#include "ad_flash.h"
#include "rtc_hal.h"
#include "osal.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "pin_names.h"
#include "ad_spi.h"
#include "platform_devices.h"
#include "uartdev_hal.h"
#ifdef __cplusplus
extern "C" {
#endif

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
static CLASS(UartDevHal) *p_instance = NULL;
/*********************************************************************
 * LOCAL DECLARATION
 */
static int read(CLASS(UartDevHal) *arg, uart_dev_handle_t hdl, uint8_t *p_buf, size_t size, uint16_t timeout);
static int write(CLASS(UartDevHal) *arg, uart_dev_handle_t hdl, uint8_t *p_buf, size_t size);
static uart_dev_handle_t open(CLASS(UartDevHal) *arg);
static int close(CLASS(UartDevHal) *arg, uart_dev_handle_t hdl);
static int read_async(CLASS(UartDevHal) *arg, uart_dev_handle_t hdl, uint8_t *p_buf, size_t size, uart_user_cb cb, void *user_data);
/******************************************************************************
 * FunctionName : UartDevHal_init
 * Description  : UartDevHal object constuction function
 * Parameters   : CLASS(UartDevHal) *arg: object pointer
 * Returns          : 0: sucess
 *                   -1: error
 *******************************************************************************/

static int UartDevHal_init(CLASS(UartDevHal) *arg)
{
        if (arg == NULL) {
                return -1;
        }
        arg->open = open;
        arg->read = read;
        arg->write = write;
        arg->close = close;
        arg->read_asnyc = read_async;
        ad_uart_init();

        return 0;
}

/******************************************************************************
 * FunctionName : UartDevHal_get_instance
 * Description  : for user to get single instance object pointer
 * Parameters   : none
 * Returns          : 0: sucess
 *                   -1: error
 *******************************************************************************/

CLASS(UartDevHal)* UartDevHal_get_instance(void)
{
        static CLASS(UartDevHal) p;
        if (p_instance == NULL) {
                p_instance = &p;
                memset(&p, 0, sizeof(p));
                UartDevHal_init(p_instance);
        }
        return p_instance;
}

/******************************************************************************
 * FunctionName :open
 * Description  : used to get aobject pointer
 * Parameters   : arg:object pointer
 * Returns          : 0: sucess
 -1: error
 *******************************************************************************/

static uart_dev_handle_t open(CLASS(UartDevHal) *arg)
{
  if ((CONFIG_RETARGET_UART) == (HW_UART1)){
        return ad_uart_open(SERIAL2);
  }
  else{
        return ad_uart_open(SERIAL1);
  }
}

static int write(CLASS(UartDevHal) *arg, uart_dev_handle_t hdl, uint8_t *p_buf, size_t size)
{
        ad_uart_write(hdl, (char*)p_buf, size);
        return 0;
}


static int read(CLASS(UartDevHal) *arg, uart_dev_handle_t hdl, uint8_t *p_buf, size_t size, uint16_t timeout)
{
        ad_uart_read(hdl, (char*)p_buf, size, OS_MS_2_TICKS(timeout));
        return 0;
}

static int read_async(CLASS(UartDevHal) *arg, uart_dev_handle_t hdl, uint8_t *p_buf, size_t size, uart_user_cb cb, void *user_data)
{
        ad_uart_read_async(hdl, (char*)p_buf, size, cb, user_data);
        return 0;
}

static int close(CLASS(UartDevHal) *arg, uart_dev_handle_t hdl)
{
        ad_uart_close(hdl);
        return 0;
}

#ifdef __cplusplus
}
#endif
