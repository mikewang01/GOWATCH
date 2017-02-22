
/******************************************************************************
 #include <spidev_hal.h>
 #include <spidev_hal.h>
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
#include "platform_devices.h"
#include "ad_spi.h"
#include "spidev_hal.h"

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
static CLASS(SpiDevHal) *p_instance = NULL;
/*********************************************************************
 * LOCAL DECLARATION
 */
static int write_read(CLASS(SpiDevHal) *arg, spi_dev_handle_t hdl, uint8_t *p_txbuf, size_t tx_size,
        uint8_t *p_rxbuf, size_t rx_size);
static int read(CLASS(SpiDevHal) *arg, spi_dev_handle_t hdl, uint8_t *p_buf, size_t size);
static int write(CLASS(SpiDevHal) *arg, spi_dev_handle_t hdl, uint8_t *p_buf, size_t size);
//static spi_dev_handle_t open(CLASS(SpiDevHal) *arg, const SPI_BUS_DEVICE_NAME dev_id);
static int close(CLASS(SpiDevHal) *arg, spi_dev_handle_t hdl);
static spi_dev_handle_t acc_open(CLASS(SpiDevHal) *arg);
static spi_dev_handle_t oled_open(CLASS(SpiDevHal) *arg);
static spi_dev_handle_t flash_open(CLASS(SpiDevHal) *arg);
static spi_dev_handle_t fm1280_open(CLASS(SpiDevHal) *arg);
/******************************************************************************
 * FunctionName : SpiDevHal_init
 * Description  : SpiDevHal object constuction function
 * Parameters   : CLASS(SpiDevHal) *arg: object pointer
 * Returns          : 0: sucess
 *                   -1: error
 *******************************************************************************/
static int SpiDevHal_init(CLASS(SpiDevHal) *arg)
{
        if (arg == NULL) {
                return -1;
        }
#if IS_SPI_ACC_SENSOR_SUPPORTED
        arg->acc_open = acc_open;
#endif
#if IS_SPI_FLASH_SUPPORTED
				 arg->flash_open = flash_open;
#endif
#if IS_SPI_OLED_SUPPORTED
				 arg->oled_open = oled_open;
#endif
#if IS_SPI_FM1280_SUPPORTED
				 arg->fm1280_open = fm1280_open;
#endif
				
        arg->read = read;
        arg->write = write;
        arg->write_read = write_read;
        arg->close = close;

        ad_spi_init();
        SPI_BUS_INIT(SPI1);

#if IS_SPI_ACC_SENSOR_SUPPORTED
        SPI_DEVICE_INIT(ACC_SENSOR);
#endif
#if IS_SPI_FM1280_SUPPORTED
        SPI_DEVICE_INIT(FM1280);
#endif
#if IS_SPI_OLED_SUPPORTED
        SPI_DEVICE_INIT(OLED);
#endif


#if IS_SPI_FLASH_SUPPORTED
				SPI_DEVICE_INIT(FLASH);
#endif
        return 0;
}

/******************************************************************************
 * FunctionName : SpiDevHal_get_instance
 * Description  : for user to get single instance object pointer
 * Parameters   : none
 * Returns          : 0: sucess
 *                   -1: error
 *******************************************************************************/

CLASS(SpiDevHal)* SpiDevHal_get_instance(void)
{
        static CLASS(SpiDevHal) p;
        if (p_instance == NULL) {
                p_instance = &p;
                memset(&p, 0, sizeof(p));
                SpiDevHal_init(p_instance);
        }
        return p_instance;
}
 
/******************************************************************************
 * FunctionName :btle_fsm_process
 * Description  : fsm
 * Parameters   : write: write call back function
 * Returns          : 0: sucess
 -1: error
 *******************************************************************************/
#if 0
static spi_dev_handle_t open(CLASS(SpiDevHal) *arg, const SPI_BUS_DEVICE_NAME dev_id)
{
        return ad_spi_open(dev_id);
}
#endif
#if IS_SPI_ACC_SENSOR_SUPPORTED
static spi_dev_handle_t acc_open(CLASS(SpiDevHal) *arg)
{
       return ad_spi_open(ACC_SENSOR);
}
#endif

#if IS_SPI_FLASH_SUPPORTED
static spi_dev_handle_t flash_open(CLASS(SpiDevHal) *arg)
{
       return ad_spi_open(FLASH);
}
#endif

#if IS_SPI_OLED_SUPPORTED
static spi_dev_handle_t oled_open(CLASS(SpiDevHal) *arg)
{
       return ad_spi_open(OLED);
}
#endif

#if IS_SPI_FM1280_SUPPORTED
static spi_dev_handle_t fm1280_open(CLASS(SpiDevHal) *arg)
{
       return ad_spi_open(FM1280);
}
#endif
static int write(CLASS(SpiDevHal) *arg, spi_dev_handle_t hdl, uint8_t *p_buf, size_t size)
{
        ad_spi_write(hdl, p_buf, size);
        return 0;
}

static int read(CLASS(SpiDevHal) *arg, spi_dev_handle_t hdl, uint8_t *p_buf, size_t size)
{
        ad_spi_read(hdl, p_buf, size);
        return 0;
}

static int write_read(CLASS(SpiDevHal) *arg, spi_dev_handle_t hdl, uint8_t *p_txbuf, size_t tx_size,
        uint8_t *p_rxbuf, size_t rx_size)
{
        ad_spi_transact(hdl, p_txbuf, tx_size, p_rxbuf, rx_size);
        return 0;
}

static int close(CLASS(SpiDevHal) *arg, spi_dev_handle_t hdl)
{
        ad_spi_close(hdl);
        return 0;
}

#ifdef __cplusplus
}
#endif
