/********************************************************************************

 **** Copyright (C), 2015, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : uartdev_hal.h
 * Author        : MikeWang
 * Date          : 2017-07-31
 * Description   : __UART_HAL_H__ header file
 * Version       : 1.0
 * Function List :
 *
 * Record        :
 * 1.Date        : 2017-07-31
 *   Author      : MikeWang
 *   Modification: Created file

 *************************************************************************************************************/

#ifndef __UART_HAL_H__
#define __UART_HAL_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "stdint.h"
#include "oop_hal.h"
#include "pinmap.h"

/*use ext
 * #include "platform_devices.h"
 *      int main()
 *      {
 *                     CLING_UART_DEV_HAL_INIT();
                        CLASS(UartDevHal) *p = UartDevHal_get_instance();
                        uart_dev_handle_t t = p->open(p);
                        p->write(p,t,"+++++++++++++hello hicling++++++++ \r\n", 32);
 *
 *      }
 *
 *
 * */
typedef int (*uart_write_callback)(void *p);
/*================================type def===============================================*/

typedef void* uart_dev_handle_t;
typedef void (*uart_user_cb)(void *user_data, uint16_t transferred);
/*================================CONFIGRATION===============================================*/
DEF_CLASS(UartDevHal)
        uart_dev_handle_t (*open)(CLASS(UartDevHal) *arg);
        int (*write)(CLASS(UartDevHal) *arg, spi_dev_handle_t hdl, uint8_t *p_buf,  size_t size);
				int (*write_asnyc)(CLASS(UartDevHal) *arg, spi_dev_handle_t hdl, uint8_t *p_buf,  size_t size,  uart_write_callback t);
        int (*read)(CLASS(UartDevHal) *arg, spi_dev_handle_t hdl, uint8_t *p_buf,  size_t size, uint16_t timeout);/*blocked read, wait for timeout ms*/
        int (*read_asnyc)(CLASS(UartDevHal) *arg, uart_dev_handle_t hdl, uint8_t *p_buf, size_t size, uart_user_cb cb, void *user_data);
        int (*close)(CLASS(UartDevHal) *arg, spi_dev_handle_t hdl);
END_DEF_CLASS(UartDevHal)


CLASS(UartDevHal)* UartDevHal_get_instance(void);


/*================================CONFIGRATION===============================================*/




/*WRAP IT TP A INITIZIZED ONE */
#define CLING_UART_DEV_HAL_INIT()  do{\
        UartDevHal_get_instance();\
}while(0)


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* __UART_HAL_H__ */
