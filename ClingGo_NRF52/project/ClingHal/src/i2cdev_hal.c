#include <btle_api.h>
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
#include "main.h"
#include "pin_names.h"
#include "i2cdev_hal.h"
#include "nrf_drv_twi.h"
#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * MACROS
 */
#define  I2C_ATTACHED_BUS_NAME      I2C1 //SPI2
/*
 * Create local version of I2C bus building block macros before inclusion of platform_devices.h
 * This allows to create private variables used internally by i2c adapter implementation.
 */
#define I2C_DEVICE_DEF(bus_name, name, scl_pin, sda_pin, _address, _addr_mode,  _speed) \
        const twi_device_config dev_##name = { \
								.bus_id = NRF_DRV_TWI_INSTANCE(1),\
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
#define HW_TWI_ID nrf_drv_twi_t
typedef struct twi_device_config {
        HW_TWI_ID bus_id;               /**< SPI id as needed by hw_spi_... functions */
				uint8_t dev_address;
        nrf_drv_twi_config_t hw_init;
} twi_device_config;

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
static CLASS(I2CDevHal) *p_instance = NULL;
static struct twi_resouce twi_res_mgr= {
	.p_instance = NULL,
	.cur_dev_cfg = NULL,
};
/*********************************************************************
 * LOCAL DECLARATION
 */
static int write_read(CLASS(I2CDevHal) *arg, i2c_dev_handle_t hdl, uint8_t *fist_stage_buf, size_t first_stage_size, uint8_t *second_stage_buf, size_t second_stage_size);
static int read(CLASS(I2CDevHal) *arg, i2c_dev_handle_t hdl, uint8_t *p_buf, size_t size);
static int write(CLASS(I2CDevHal) *arg, i2c_dev_handle_t hdl, uint8_t *p_buf, size_t size);
//static i2c_dev_handle_t open(CLASS(I2CDevHal) *arg, const I2C_BUS_DEVICE_NAME dev_id);
static int close(CLASS(I2CDevHal) *arg);
static i2c_dev_handle_t oled_open(CLASS(I2CDevHal) *arg);
static i2c_dev_handle_t gps_open(CLASS(I2CDevHal) *arg);
static i2c_dev_handle_t pmu_open(CLASS(I2CDevHal) *arg);
static i2c_dev_handle_t ppg_open(CLASS(I2CDevHal) *arg);
//static i2c_dev_handle_t acc_open(CLASS(I2CDevHal) *arg);
/******************************************************************************
 * FunctionName : I2CDevHal_init
 * Description  : I2CDevHal object constuction function
 * Parameters   : CLASS(I2CDevHal) *arg: object pointer
 * Returns          : 0: sucess
 *                   -1: error
 *******************************************************************************/

static int I2CDevHal_init(CLASS(I2CDevHal) *arg)
{
        if (arg == NULL) {
                return -1;
        }
        arg->read = read;
        arg->write = write;
        arg->write_read = write_read;
        arg->close = close;
#if IS_I2C_ACC_SENSOR_SUPPORTED
        arg->acc_open = acc_open;
#endif
#if IS_I2C_OLED_SUPPORTED
        arg->oled_open = oled_open;
#endif
#if     IS_I2C_PMU_SUPPORTED
				arg->pmu_open = pmu_open;
#endif
#if     IS_I2C_GPS_SUPPORTED
				arg->gps_open = gps_open;
#endif


#if     IS_I2C_PPG_SUPPORTED
				arg->ppg_open = ppg_open;
#endif

				
#if IS_I2C_ACC_SENSOR_SUPPORTED
       // I2C_DEVICE_INIT(I2C_ACC_SENSOR_NAME);
#endif

#if IS_I2C_OLED_SUPPORTED
        //I2C_DEVICE_INIT(I2C_OLED);
#endif

        //ad_spi_init();
      //  SPI_BUS_INIT(SPI1);
				if(OS_MUTEX_CREATE(twi_res_mgr.res_lock) == false){
						
				}
				OS_EVENT_CREATE(twi_res_mgr.bus_busy);
				twi_res_mgr.ref_count = 0;
				twi_res_mgr.owner = NULL;
				
        return 0;
}

/******************************************************************************
 * FunctionName : I2CDevHal_get_instance
 * Description  : for user to get single instance object pointer
 * Parameters   : none
 * Returns          : 0: sucess
 *                   -1: error
 *******************************************************************************/

CLASS(I2CDevHal)* I2CDevHal_get_instance(void)
{
        static CLASS(I2CDevHal) p;
        if(p_instance  == NULL){
                p_instance  = &p;
                memset(&p, 0, sizeof(p));
                I2CDevHal_init(p_instance);
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

static int open(CLASS(I2CDevHal) *arg)
{
				int ret  = 0;
				/*resouce has been taken*/
				if(twi_res_mgr.ref_count > 0){
						/*this means same task request resource here*/
						if(OS_GET_CURRENT_TASK() == twi_res_mgr.owner){
								twi_res_mgr.ref_count ++;
						}else{
							  Y_SPRINTF("[twi hal]: waiting for spi resource from 0x%08x  owned by 0x%08x count = %d", (uint32_t)OS_GET_CURRENT_TASK(), (uint32_t)twi_res_mgr.owner, twi_res_mgr.ref_count );
								OS_MUTEX_GET(twi_res_mgr.res_lock, OS_MUTEX_FOREVER);
							Y_SPRINTF("[spi hal]: wait for spi resource end");
								//ret = -1;
						}
				}else{
						OS_MUTEX_GET(twi_res_mgr.res_lock, OS_MUTEX_FOREVER);
						twi_res_mgr.owner = OS_GET_CURRENT_TASK();
						twi_res_mgr.ref_count ++;
				}
        return ret;//ad_spi_open(dev_id);
}

/******************************************************************************
 * FunctionName :btle_fsm_process
 * Description  : fsm
 * Parameters   : write: write call back function
 * Returns          : 0: sucess
 -1: error
 *******************************************************************************/

#if IS_I2C_ACC_SENSOR_SUPPORTED
static i2c_dev_handle_t acc_open(CLASS(I2CDevHal) *arg)
{
       return ad_i2c_open(ACC_SENSOR);
}
#endif

#if IS_I2C_OLED_SUPPORTED
static i2c_dev_handle_t oled_open(CLASS(I2CDevHal) *arg)
{
       return NULL;//ad_i2c_open(I2C_OLED);
}
#endif

#if IS_I2C_PMU_SUPPORTED
static i2c_dev_handle_t pmu_open(CLASS(I2CDevHal) *arg)
{
       	i2c_dev_handle_t* t = (i2c_dev_handle_t*)PMU;
			if(open(arg) >= 0){
				return (i2c_dev_handle_t*)PMU;//ad_spi_open(OLED);
			}else{
				return NULL;
			}
			
       return NULL;//ad_i2c_open(I2C_OLED);
}
#endif

#if IS_I2C_GPS_SUPPORTED
static i2c_dev_handle_t gps_open(CLASS(I2CDevHal) *arg)
{
	
			i2c_dev_handle_t* t = (i2c_dev_handle_t*)GPS;
			if(open(arg) >= 0){
				return (i2c_dev_handle_t*)GPS;//ad_spi_open(OLED);
			}else{
				return NULL;
			}
			
       return NULL;//ad_i2c_open(I2C_OLED);
}
#endif


#if 1
static i2c_dev_handle_t ppg_open(CLASS(I2CDevHal) *arg)
{
	
			i2c_dev_handle_t* t = (i2c_dev_handle_t*)PPG;
			if(open(arg) >= 0){
				return (i2c_dev_handle_t*)PPG;//ad_spi_open(OLED);
			}else{
				return NULL;
			}
			
       return NULL;//ad_i2c_open(I2C_OLED);PPG_READDEV
}
#endif

/**
 * @brief TWI events handler.
 */
static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                
            }
						OS_EVENT_SIGNAL_FROM_ISR(twi_res_mgr.bus_busy);
            break;
        default:
            break;
    }
}

static int write(CLASS(I2CDevHal) *arg, i2c_dev_handle_t hdl, uint8_t *p_buf, size_t size)
{
/*if no hanlder provided , then just return error code*/
				if(hdl == NULL){
						return -1;
				}
				twi_device_config *t = (twi_device_config*)hdl;
				if(twi_res_mgr.cur_dev_cfg != t){
								twi_res_mgr.cur_dev_cfg = t;
								nrf_drv_twi_disable(&(t->bus_id));
								nrf_drv_twi_uninit(&(t->bus_id));
						    int err_code = nrf_drv_twi_init(&(t->bus_id), &t->hw_init, twi_handler, NULL);
								APP_ERROR_CHECK(err_code);
								nrf_drv_twi_enable(&(t->bus_id));
				}
				nrf_drv_twi_xfer_desc_t xfer_des = {
							    .type = NRF_DRV_TWI_XFER_TX,             ///< Type of transfer.
									.address = t->dev_address,          ///< Slave address.
									.primary_length = size,   ///< Number of bytes transferred.
									.p_primary_buf = p_buf,    ///< Pointer to transferred data.
				};
				int result = nrf_drv_twi_xfer(&(t->bus_id), &xfer_des,0);
				if(result != NRF_SUCCESS){
						return -1;
				}
				result = OS_EVENT_WAIT(twi_res_mgr.bus_busy, OS_MS_2_TICKS(5000));
				if(result == pdFALSE){
					/*if event wait timeout event happened*/
					return -1;
				}
				return 0;
       // ad_spi_transact(hdl, p_txbuf, tx_size, p_rxbuf, rx_size);
}


static int write_read(CLASS(I2CDevHal) *arg, i2c_dev_handle_t hdl, uint8_t *fist_stage_buf, size_t first_stage_size, uint8_t *second_stage_buf, size_t second_stage_size)
{ 
				/*if no hanlder provided , then just return error code*/
				if(hdl == NULL){
						return -1;
				}
				twi_device_config *t = (twi_device_config*)hdl;
				if(twi_res_mgr.cur_dev_cfg != t){
								twi_res_mgr.cur_dev_cfg = t;
								nrf_drv_twi_disable(&(t->bus_id));
								nrf_drv_twi_uninit(&(t->bus_id));
						    int err_code = nrf_drv_twi_init(&(t->bus_id), &t->hw_init, twi_handler, NULL);
								APP_ERROR_CHECK(err_code);
								nrf_drv_twi_enable(&(t->bus_id));
				}
				nrf_drv_twi_xfer_desc_t xfer_des = {
							    .type = NRF_DRV_TWI_XFER_TXRX,             ///< Type of transfer.
									.address = t->dev_address,          ///< Slave address.
									.primary_length = first_stage_size,   ///< Number of bytes transferred.
									.secondary_length = second_stage_size,///< Number of bytes transferred.
									.p_primary_buf = fist_stage_buf,    ///< Pointer to transferred data.
									.p_secondary_buf = second_stage_buf,  ///< Pointer to transferred data.
				};
				int result = nrf_drv_twi_xfer(&(t->bus_id), &xfer_des,0);
				if(result != NRF_SUCCESS){
						return -1;
				}
				result = OS_EVENT_WAIT(twi_res_mgr.bus_busy, OS_MS_2_TICKS(5000));
				if(result == pdFALSE){
					/*if event wait timeout event happened*/
					return -1;
				}
				return 0;
       // ad_spi_transact(hdl, p_txbuf, tx_size, p_rxbuf, rx_size);
}


static int read(CLASS(I2CDevHal) *arg, i2c_dev_handle_t hdl, uint8_t *p_buf, size_t size)
{
	
      	/*if no hanlder provided , then just return error code*/
				if(hdl == NULL){
						return -1;
				}
				twi_device_config *t = (twi_device_config*)hdl;
				if(twi_res_mgr.cur_dev_cfg != t){
								twi_res_mgr.cur_dev_cfg = t;
								nrf_drv_twi_disable(&(t->bus_id));
								nrf_drv_twi_uninit(&(t->bus_id));
						    int err_code = nrf_drv_twi_init(&(t->bus_id), &t->hw_init, twi_handler, NULL);
								APP_ERROR_CHECK(err_code);
								nrf_drv_twi_enable(&(t->bus_id));
				}
				nrf_drv_twi_xfer_desc_t xfer_des = {
							    .type = NRF_DRV_TWI_XFER_RX,             ///< Type of transfer.
									.address = t->dev_address,          ///< Slave address.
									.primary_length = size,   ///< Number of bytes transferred.
									.p_primary_buf = p_buf,    ///< Pointer to transferred data.
				};
				int result = nrf_drv_twi_xfer(&(t->bus_id), &xfer_des,0);
				if(result != NRF_SUCCESS){
						return -1;
				}
				result = OS_EVENT_WAIT(twi_res_mgr.bus_busy, OS_MS_2_TICKS(5000));
				if(result == pdFALSE){
					/*if event wait timeout event happened*/
					return -1;
				}
				return 0;
       // ad_spi_transact(hdl, p_txbuf, tx_size, p_rxbuf, rx_size);
}



static int close(CLASS(I2CDevHal) *arg)
{
 //ad_spi_close(hdl);
				int ret  = 0;
				/*resouce has been taken*/
				if(twi_res_mgr.ref_count > 0){
						twi_res_mgr.ref_count --;
						/*this means same task request resource here*/
						if(twi_res_mgr.ref_count == 0){
								//spi_res_mgr.ref_count ++;
								OS_MUTEX_PUT(twi_res_mgr.res_lock);
						}
				}
        return ret;//ad_spi_open(dev_id);			
}

#ifdef __cplusplus
}
#endif
