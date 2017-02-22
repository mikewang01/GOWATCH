/********************************************************************************

 **** Copyright (C), 2015, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : wrist_detect.h
 * Author        : MikeWang
 * Date          : 2016-03-28
 * Description   : __I2C_DEV_HAL_H__ header file
 * Version       : 1.0
 * Function List :
 *
 * Record        :
 * 1.Date        : 2016-03-28
 *   Author      : MikeWang
 *   Modification: Created file

 *************************************************************************************************************/

#ifndef __I2C_DEV_HAL_H__
#define __I2C_DEV_HAL_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "stdint.h"
#include "oop_hal.h"
//#include "ad_i2c.h"

/*use ex
 * #include "platform_devices.h"
 *      int main()
 *      {
 *              CLING_I2C_DEV_HAL_INIT();
 *              CLASS(I2CDevHal)* p =  I2CDevHal_get_instance();
 *              i2c_dev_handle_t t = p->acc_open(p);
 *              uint8_t aaa[32];
 *              p->read(p,t,aaa,32);
 *
 *      }
 *
 *
 * */

/*================================type def===============================================*/
#define  IS_I2C_ACC_SENSOR_SUPPORTED (0)
#define  I2C_ACC_SENSOR_ADDR        0X00

#define  IS_I2C_OLED_SUPPORTED      (0)
#define  I2C_OLED_ADDR              0X3C


#define  IS_I2C_PMU_SUPPORTED       (1) /*power manager unit enabled*/
#define  I2C_PMU_ADDR								(0X12>>1)

#define  IS_I2C_PPG_SUPPORTED				(1)/*gps manager unit enabled*/
#define  I2C_PPG_ADDR								(0x60>>1)

#define  IS_I2C_GPS_SUPPORTED				(1)/*gps manager unit enabled*/
#define  I2C_GPS_ADDR								(0x42)

typedef void* i2c_dev_handle_t;
#define  INVALID_SPI_DEV_HANDLE NULL
#define  I2C_BUS_DEVICE_NAME    i2c_device_id

/*================================CONFIGRATION===============================================*/
DEF_CLASS(I2CDevHal)
#if     IS_I2C_ACC_SENSOR_SUPPORTED
i2c_dev_handle_t (*acc_open)(CLASS(I2CDevHal) *arg);
#endif
#if     IS_I2C_OLED_SUPPORTED
i2c_dev_handle_t (*oled_open)(CLASS(I2CDevHal) *arg);
#endif

#if     IS_I2C_PMU_SUPPORTED
i2c_dev_handle_t (*pmu_open)(CLASS(I2CDevHal) *arg);
#endif
#if     IS_I2C_PPG_SUPPORTED
i2c_dev_handle_t (*ppg_open)(CLASS(I2CDevHal) *arg);
#endif

#if     IS_I2C_GPS_SUPPORTED
i2c_dev_handle_t (*gps_open)(CLASS(I2CDevHal) *arg);
#endif

        int (*write)(CLASS(I2CDevHal) *arg, i2c_dev_handle_t hdl, uint8_t *p_buf, size_t size);
        int (*read)(CLASS(I2CDevHal) *arg, i2c_dev_handle_t hdl, uint8_t *p_buf, size_t size);
        int (*write_read)(CLASS(I2CDevHal) *arg, i2c_dev_handle_t hdl, uint8_t *fist_stage_buf, size_t first_stage_size, uint8_t *second_stage_buf, size_t second_stage_size);
        int (*close)(CLASS(I2CDevHal) *arg);
END_DEF_CLASS(I2CDevHal)

CLASS(I2CDevHal)* I2CDevHal_get_instance(void);

/*================================CONFIGRATION===============================================*/

/*WRAP IT TP A INITIZIZED ONE */
#define CLING_I2C_DEV_HAL_INIT()  do{\
        I2CDevHal_get_instance();\
}while(0)


#define I2C_DEVICE_DEF(bus_name, name, scl_pin, sda_pin, _address, _addr_mode,  _speed) \
	extern const void *const name;
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* __I2C_DEV_HAL_H__ */
