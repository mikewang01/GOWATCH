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
//#include "ad_spi.h"
#include "pinmap.h"

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


#define  ATTACHED_BUS_NAME      SPI1 //SPI2
/*================================type def===============================================*/

typedef void* spi_dev_handle_t;
#define  INVALID_SPI_DEV_HANDLE NULL
#define  SPI_BUS_DEVICE_NAME    uint32_t

/*================================CONFIGRATION===============================================*/
DEF_CLASS(SpiDevHal)
#if     IS_SPI_ACC_SENSOR_SUPPORTED
        spi_dev_handle_t (*acc_open)(CLASS(SpiDevHal) *arg);
#endif
#if     IS_SPI_OLED_SUPPORTED
        spi_dev_handle_t (*oled_open)(CLASS(SpiDevHal) *arg);
#endif

#if     IS_I2C_PPG_SUPPORTED
        spi_dev_handle_t (*ppg_open)(CLASS(SpiDevHal) *arg);
#endif

				spi_dev_handle_t (*flash_open)(CLASS(SpiDevHal) *arg);
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

/*********************************************************************
 * MACROS
 */
/**
 * \brief Entry for slave device
 *
 * \param [in] bus_id value must match SPI_BUS() argument valid values: SPI1, SPI2
 * \param [in] name name that will be used later to open device
 * \param [in] cs_port port id for chip select
 * \param [in] cs_pin pin number for chip select
 * \param [in] word_mode SPI word size from HW_SPI_WORD enum
 * \param [in] pol_mode idle state of clock line from HW_SPI_POL enum
 * \param [in] phase_mode which edge of clock is used for latching data from HW_SPI_PHA enum
 * \param [in] xtal_div divisor of main clock, used for generating SPI clock from HW_SPI_FREQ enum
 * \param [in] dma_channel DMA number for rx channel, tx will have next number, pass -1 for no DMA
 *
 */
#define SPI_SLAVE_DEVICE(bus, name, cs_port, cs_pin, _word_mode, spi_mode, spi_rate, bit_order)  \
        extern const void *const name;


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* __SPI_HAL_H__ */
