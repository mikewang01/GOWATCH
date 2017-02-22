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

#ifndef __PLATFORM_DEV_H__
#define __PLATFORM_DEV_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */
#include "sdk_config.h"
#include "stdint.h"
#include "oop_hal.h"
#include "pinmap.h"
#include "spidev_hal.h"
#include "i2cdev_hal.h"
#include "nrf_drv_spi.h"
#include "battmgr_hal.h"
	
SPI_SLAVE_DEVICE(SPI0, OLED, 0, 	GPIO_OLED_CS, 0, NRF_DRV_SPI_MODE_0, SPI0_DEFAULT_FREQUENCY, NRF_DRV_SPI_BIT_ORDER_MSB_FIRST)
SPI_SLAVE_DEVICE(SPI0, FLASH, 0,	GPIO_FLASH_CS, 0, NRF_DRV_SPI_MODE_0, SPI0_DEFAULT_FREQUENCY, NRF_DRV_SPI_BIT_ORDER_MSB_FIRST)
SPI_SLAVE_DEVICE(SPI0, ACC, 0, 		GPIO_SPI_CS_ACC, 0, NRF_DRV_SPI_MODE_0, SPI0_DEFAULT_FREQUENCY, NRF_DRV_SPI_BIT_ORDER_MSB_FIRST)	

	
I2C_DEVICE_DEF(I2C0, PMU, GPIO_TWI_CLK, GPIO_TWI_DATA, I2C_PMU_ADDR, HW_I2C_ADDRESSING_7B,  TWI_FREQUENCY_FREQUENCY_K400) 
I2C_DEVICE_DEF(I2C0, GPS, GPIO_TWI_CLK, GPIO_TWI_DATA, I2C_GPS_ADDR, HW_I2C_ADDRESSING_7B,  TWI_FREQUENCY_FREQUENCY_K400) 
I2C_DEVICE_DEF(I2C0, PPG, GPIO_TWI_CLK, GPIO_TWI_DATA, I2C_PPG_ADDR, HW_I2C_ADDRESSING_7B,  TWI_FREQUENCY_FREQUENCY_K400)
SAADC_DEVICE(0, BATT, GPIO_CHG_IN, SAADC_CONFIG_RESOLUTION, SAADC_CONFIG_OVERSAMPLE);
	#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* __UART_HAL_H__ */
