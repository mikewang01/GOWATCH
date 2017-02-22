/********************************************************************************

 **** Copyright (C), 2015, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : wrist_detect.h
 * Author        : MikeWang
 * Date          : 2016-03-28
 * Description   : __SPI_HAL_H__ header file
 * Version       : 1.0
 * Function List :
 *
 * Record        :
 * 1.Date        : 2016-03-28
 *   Author      : MikeWang
 *   Modification: Created file

 *************************************************************************************************************/

#ifndef __SPI_HAL_H__
#define __SPI_HAL_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "stdint.h"
#include "oop_hal.h"
#include "ad_spi.h"

/*use ex
 * #include "platform_devices.h"
 *      int main()
 *      {
 *              CLING_SPI_DEV_HAL_INIT();
 *              CLASS(SpiDevHal)* p =  SpiDevHal_get_instance();
                spi_dev_handle_t t = p->acc_open(p);
                uint8_t aaa[32];
                p->read(p,t,aaa,32);
 *
 *      }
 *
 *
 * */

/*================================type def===============================================*/
#define  IS_SPI_ACC_SENSOR_SUPPORTED (1)

#define  IS_SPI_OLED_SUPPORTED       (1)

#define  IS_SPI_PPG_SENSOR_SUPPORTED       (0)

#define  IS_SPI_FLASH_SUPPORTED      (1)
#define  IS_SPI_FM1280_SUPPORTED     (1)

#define  ATTACHED_BUS_NAME      SPI1 //SPI2
/*================================type def===============================================*/

typedef void* spi_dev_handle_t;
#define  INVALID_SPI_DEV_HANDLE NULL
#define  SPI_BUS_DEVICE_NAME    spi_device_id

/*================================CONFIGRATION===============================================*/
DEF_CLASS(SpiDevHal)
#if     IS_SPI_ACC_SENSOR_SUPPORTED
        spi_dev_handle_t (*acc_open)(CLASS(SpiDevHal) *arg);
#endif
#if     IS_SPI_FLASH_SUPPORTED
        spi_dev_handle_t (*flash_open)(CLASS(SpiDevHal) *arg);
#endif
#if     IS_SPI_OLED_SUPPORTED
        spi_dev_handle_t (*oled_open)(CLASS(SpiDevHal) *arg);
#endif
#if     IS_SPI_OLED_SUPPORTED
        spi_dev_handle_t (*fm1280_open)(CLASS(SpiDevHal) *arg);
#endif

#if     IS_SPI_PPG_SENSOR_SUPPORTED
        spi_dev_handle_t (*ppg_open)(CLASS(SpiDevHal) *arg);
#endif
        int (*write)(CLASS(SpiDevHal) *arg, spi_dev_handle_t hdl, uint8_t *p_buf,  size_t size);
        int (*read)(CLASS(SpiDevHal) *arg, spi_dev_handle_t hdl, uint8_t *p_buf,  size_t size);
        int (*write_read)(CLASS(SpiDevHal) *arg, spi_dev_handle_t hdl, uint8_t *p_txbuf,  size_t tx_size, uint8_t *p_rxbuf,  size_t rx_size);
        int (*close)(CLASS(SpiDevHal) *arg, spi_dev_handle_t hdl);
END_DEF_CLASS(SpiDevHal)


CLASS(SpiDevHal)* SpiDevHal_get_instance(void);


/*================================CONFIGRATION===============================================*/




/*WRAP IT TP A INITIZIZED ONE */
#define CLING_SPI_DEV_HAL_INIT()  do{\
        SpiDevHal_get_instance();\
}while(0)


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* __SPI_HAL_H__ */
