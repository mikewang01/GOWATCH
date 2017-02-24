
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
#include "osal.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "pin_names.h"
#include "uartdev_hal.h"
#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * MACROS
 */
/*
 * Create local version of I2C bus building block macros before inclusion of platform_devices.h
 * This allows to create private variables used internally by i2c adapter implementation.
 */
#define I2C_DEVICE_DEF(bus_name, name, scl_pin, sda_pin, _address, _addr_mode,  _speed) \
        const twi_device_config dev_##name = { \
								.bus_id = NRF_DRV_TWI_INSTANCE(0),\
								.dev_address = _address,\
					      .hw_init.scl                = scl_pin,\
								.hw_init.sda                = sda_pin,\
								.hw_init.frequency          = _speed,\
								.hw_init.interrupt_priority = TWI_DEFAULT_CONFIG_IRQ_PRIORITY,\
								.hw_init.clear_bus_init     = false\
        }; \
        const void *const name = &dev_##name;

/*********************************************************************
 * TYPEDEFS
 */
/*********************************************************************
 * TYPEDEFS
 */
#define HW_UART_ID nrf_drv_uart_t
typedef struct config {
        HW_UART_ID bus_id;               /**< bus id as needed by hw_spi_... functions */
        nrf_drv_uart_config_t hw_init;
} uart_device_config;

struct twi_resouce{
		CLASS(I2CDevHal) *p_instance;
		OS_MUTEX  res_lock;
		OS_EVENT  bus_busy;
		uint16_t  ref_count;
		OS_TASK 	owner;
		twi_device_config *cur_dev_cfg;/*use to record current device config*/
};
#include "flatform_dev.h"

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
