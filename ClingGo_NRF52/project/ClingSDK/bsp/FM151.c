/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
* 
* @defgroup gpiote_example_main main.c
* @{
* @ingroup nrf_gpiote_example
* @brief GPIOTE Example Application main file.
*
* This file contains the source code for a sample application using GPIOTE. 
*/
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "iso7816_hal.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include <string.h>
#include "pin_names.h"

#ifdef NRF51
    #define PIN_VCC             4       /**< ISO7816-3 Vcc pin */
    #define PIN_CLK             3       /**< ISO7816-3 clock pin */
    #define PIN_RST             24      /**< ISO7816-3 reset pin */
    #define PIN_IO              13      /**< ISO7816-3 I/O pin */
    #define PIN_DBG_ACTIVITY    22      /**< Debug pin on CPU activities. */
    #define PIN_DBG_PPI         21      /**< Debug pin on PPI activities. */
#elif defined(NRF52)
    #define PIN_VCC             NRF_DRV_SPI_PIN_NOT_USED      /**< ISO7816-3 Vcc pin */
    #define PIN_CLK             28      /**< ISO7816-3 clock pin */
    #define PIN_RST             22      /**< ISO7816-3 reset pin */
    #define PIN_IO              12      /**< ISO7816-3 I/O pin */
    #define PIN_DBG_ACTIVITY    NRF_DRV_SPI_PIN_NOT_USED      /**< Debug pin on CPU activities. */
    #define PIN_DBG_PPI         NRF_DRV_SPI_PIN_NOT_USED      /**< Debug pin on PPI activities. */
#else
    #error Platform not supported.
#endif

#define LOW_POWER_SLEEP         0       /**< Low power idle mode, which will result in lower bit-rate. */
#define SLEEP_TIME              1000
#define RETRY                   3

static volatile bool m_sys_rdy = false;
static volatile bool m_cont = false;
static uint8_t m_buff[128];
iso7816_hal_tpdu_t tpdu;

#if LOW_POWER_SLEEP
#define SLEEP_UNTIL(cond)   \
    while (!(cond))         \
    {                       \
        __WFE();            \
        __SEV();            \
        __WFE();            \
    }
#else
#define SLEEP_UNTIL(cond)   \
    while (!(cond))         \
    {                       \
    }
#endif

static void print_hex (uint8_t *data, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len - 1; i++)
    {
        NRF_LOG_HEX_CHAR(data[i]);
        NRF_LOG(" ");
    }
    NRF_LOG_HEX_CHAR(data[i]);
}


static void print_atr ()
{
    uint8_t *data;
    uint16_t len;

    NRF_LOG ("ATR: ");
    iso7816_hal_get_atr (&data, &len);
    print_hex (data, len);
    NRF_LOG ("\r\n");
}
static void get_cid (void)
{
		uint32_t err_code = NRF_SUCCESS;

    tpdu.cla  = 0x00;   // CLA: Proprietary, GSM
    tpdu.ins  = 0xCA;   // INS: SELECT
    tpdu.p1   = 0x01;   // P1 : Select MF, DF or ER, with file identifier in data field
    tpdu.p2   = 0x01;   // P2 : 1st or only occurance, return FCI template
    tpdu.req  = ISO7816_HAL_TPDU_REQ_GET;
    tpdu.len  = 8;
    tpdu.data = m_buff;

    err_code = iso7816_hal_send_tpdu (&tpdu);
    APP_ERROR_CHECK(err_code);

    iso7816_hal_log_tpdu_req (&tpdu);		
}


static void select (void)
{
    uint32_t err_code = NRF_SUCCESS;

    tpdu.cla  = 0xA0;   // CLA: Proprietary, GSM
    tpdu.ins  = 0xA4;   // INS: SELECT
    tpdu.p1   = 0x00;   // P1 : Select MF, DF or ER, with file identifier in data field
    tpdu.p2   = 0x00;   // P2 : 1st or only occurance, return FCI template
    tpdu.req  = ISO7816_HAL_TPDU_REQ_PUT;
    tpdu.len  = 2;
    tpdu.data = m_buff;

    m_buff[0] = 0x2F;   // File ID: ICCID (mandatory in all GSM IC Cards)
    m_buff[1] = 0xE2;

    err_code = iso7816_hal_send_tpdu (&tpdu);
    APP_ERROR_CHECK(err_code);

    iso7816_hal_log_tpdu_req (&tpdu);
}


static void get_response (uint8_t len)
{
    uint32_t err_code = NRF_SUCCESS;

    tpdu.cla  = 0xA0;   // CLA: Proprietary, GSM
    tpdu.ins  = 0xC0;   // INS: GET RESPONSE
    tpdu.p1   = 0x00;   // P1 : reserved
    tpdu.p2   = 0x00;   // P2 : reserved
    tpdu.req  = ISO7816_HAL_TPDU_REQ_GET;
    tpdu.len  = len;
    tpdu.data = m_buff;

    err_code  = iso7816_hal_send_tpdu (&tpdu);
    APP_ERROR_CHECK(err_code);

    iso7816_hal_log_tpdu_req (&tpdu);
}

static uint8_t sw1 = 0;
static uint8_t sw2 = 0;
typedef enum {
    FM1280_DEACTIVE,
    FM1280_IDLE,
    FM1280_ACTIVED,
    FM1280_SEND,
    FM1280_ACTIVING,
    FM1280_ACTIVE_END,
    FM1280_RESPONSE,
} fm1280_state_t;

static fm1280_state_t current_state = FM1280_IDLE;
static uint32_t iso7816_handler (iso7816_hal_evt_t * p_event)
{

    Y_SPRINTF("[fm1280]fm1280_handler event id= %d", p_event->evt_id);
    switch (p_event->evt_id) {
    case ISO7816_HAL_EVT_TPDU_RESPONSE:
			/* Log response. */
					iso7816_hal_log_tpdu_rsp (&tpdu, p_event->evt.tpdu_rsp.sw1, p_event->evt.tpdu_rsp.sw2);

				if (p_event->evt.tpdu_rsp.sw1 == 0x9F)
				{
						get_response (p_event->evt.tpdu_rsp.sw2);
				}
				else if (p_event->evt.tpdu_rsp.sw1 == 0x90)
				{
						m_sys_rdy = true;
						sw1 = p_event->evt.tpdu_rsp.sw1;
						sw2 = p_event->evt.tpdu_rsp.sw2;
						current_state = FM1280_RESPONSE;	
				}else{
						m_sys_rdy = true;
						sw1 = p_event->evt.tpdu_rsp.sw1;
						sw2 = p_event->evt.tpdu_rsp.sw2;
						current_state = FM1280_RESPONSE;				
				}
        
        break;

    case ISO7816_HAL_EVT_TPDU_TIMEOUT:
        break;

    case ISO7816_HAL_EVT_RDY:
        m_sys_rdy = true;
        current_state = FM1280_ACTIVE_END;
        break;

    case ISO7816_HAL_EVT_NOT_INITIALIZED:
        m_sys_rdy = false;
        current_state = FM1280_DEACTIVE;
        break;

    default:
        break;
    }
    return NRF_SUCCESS;
}


int active_fm1280()
{
	    iso7816_hal_init_t hal_init_param;
    int i;
		int32_t err_code = 0;

		if(current_state == FM1280_IDLE){
#ifdef NRF52
					NRF_NVMC->ICACHECNF = NVMC_ICACHECNF_CACHEEN_Enabled << NVMC_ICACHECNF_CACHEEN_Pos;
#endif
					NRF_POWER->TASKS_CONSTLAT = 1;

					//NRF_LOG_INIT ();
					NRF_LOG ("ISO7816-3 PHY/HAL demo.\r\n");

					err_code = nrf_drv_ppi_init();
					APP_ERROR_CHECK(err_code);

					err_code = nrf_drv_gpiote_init();
					APP_ERROR_CHECK(err_code);
					memset (&hal_init_param, 0, sizeof(iso7816_hal_init_t));
					hal_init_param.clock_freq  = ISO7816_HAL_FREQ_4MHZ;
					hal_init_param.vcc_pin     = GPIO_7816_VCC;
					hal_init_param.clk_pin     = GPIO_7816_CLK;
					hal_init_param.rst_pin     = GPIO_7816_RST;
					hal_init_param.io_pin      = GPIO_7816_IO;
					hal_init_param.dbg_act_pin = P_NOT_USED;
					hal_init_param.dbg_ppi_pin = P_NOT_USED; 
					hal_init_param.evt_cb      = iso7816_handler;
					hal_init_param.enable_pps	 = false;

					err_code = iso7816_hal_init (&hal_init_param);
					APP_ERROR_CHECK(err_code);
		
		  err_code = NRF_SUCCESS;
			
			current_state  = FM1280_ACTIVING;
			err_code = NRF_ERROR_BUSY;
		}
		return err_code;
}

int deactive_fm1280()
{
		 int32_t err_code = NRF_SUCCESS;
		if(current_state == FM1280_IDLE){
	
		}
		return err_code;
}

static bool data_sended = true;
static iso7816_hal_tpdu_t m_tpdu;
#define RESPONSE_SW_RETIEVE_DATA 		    0x61
#define RESPONSE_SW_WRONG_LEGNTH        0X6C
#define RESPONSE_SW_CMD_NOT_SUPPORTED   0X6D
#define RESPONSE_SW_CMD_NULL						0x60
#define RESPONSE_SW_SUCESS_MASK1		    0X60
#define RESPONSE_SW_SUCESS_MASK2		    0X90

static int core_fsm(void *data)
{
    static  uint16_t data_length = 0;
    static uint8_t last_state=0xff;
		static uint32_t t_curr = 0;
    uint32_t err_code = NRF_SUCCESS;
    int ret = 0;	
    if(current_state != last_state) {
        Y_SPRINTF("[fm1280]state change to %d", current_state);
        last_state = current_state;
				t_curr = CLK_get_system_time();
    }else{
				/*when no oepration happened in 30s to fsm 1280 switch to */
				if(CLK_get_system_time() - t_curr > 600000){
					if(current_state != FM1280_IDLE){
							Y_SPRINTF("[fm1280] fsm1280 active timeout");
							deactive_fm1280();
							current_state = FM1280_IDLE;
					}
							t_curr = CLK_get_system_time();
				}
		}
		
    switch(current_state) {
    case  FM1280_IDLE:
				t_curr = CLK_get_system_time();
        break;
    case FM1280_ACTIVING:
        break;
    case FM1280_ACTIVE_END:  {
        static const uint8_t tmp[] = {0x90, 0x00};
//        CLASS(cling_pay_app)*p =cling_pay_app_get_instance();
//        /*notify app 1280 has been actived sucessfully*/
//        p->send_event_from_1280(p, (uint8_t *)tmp, sizeof(tmp));
        current_state = FM1280_ACTIVED;
    }
    break;

    case	FM1280_ACTIVED:
        break;
    case   FM1280_SEND:
        if(data_sended  == false) {
            err_code = iso7816_hal_send_tpdu (&m_tpdu);
            if(err_code == NRF_ERROR_BUSY) {

                Y_SPRINTF("NRF_ERROR_BUSY 0");
                //iso7816_hal_cleanup ();
                current_state = FM1280_DEACTIVE;
                break;
            } else {
                data_sended = true;
            }
            /*since every transmition cycle starts with ISO7816_HAL_TPDU_REQ_GET, data lenth recorded for echaed
            	cycle should be cleared once a new one get requested*/
            if(m_tpdu.req == ISO7816_HAL_TPDU_REQ_GET) {
                data_length = 0;
            }else{
								data_length = m_tpdu.len;
						}
        } else {
            /*if no data pending, put state machine into active fress state*/
            current_state = FM1280_ACTIVED;
        }
        break;
    case	FM1280_RESPONSE:
        //Y_SPRINTF("[fm1280]: sw1=0x%02x,sw2=0x%02x", sw1, sw2);
        switch(sw1) {
        case RESPONSE_SW_WRONG_LEGNTH: {
            m_tpdu.len = sw2;
            m_tpdu.req = ISO7816_HAL_TPDU_REQ_GET;
            data_sended = false;
            current_state = FM1280_SEND;
            data_length = sw2;
            /*one more loop is needed here to transmit data*/
            ret = 1;
					  Y_SPRINTF("RESPONSE_SW_WRONG_LEGNTH 0");
        }
        break;
        case RESPONSE_SW_CMD_NULL:
            break;
        case RESPONSE_SW_RETIEVE_DATA: {
            //Y_SPRINTF("RESPONSE_SW_RETIEVE_DATA");
            iso7816_hal_tpdu_t tpdu;
            tpdu.cla  = 0x00;
            tpdu.ins  = 0xc0;
            tpdu.p1   = 0x00;
            tpdu.p2   = 0x00;
            tpdu.req  = ISO7816_HAL_TPDU_REQ_GET;
            tpdu.len  = sw2;
            data_length = sw2;
            tpdu.data = m_tpdu.data;
            memcpy(&m_tpdu, &tpdu, sizeof(tpdu));
            data_sended = false;
            current_state = FM1280_SEND;
            /*one more loop is needed here to transmit data*/
            ret = 1;
						Y_SPRINTF("RESPONSE_SW_RETIEVE_DATA 0");
        }
        break;
        default: {
            /*we take it as a successfully recieve*/
            /*	{
            				notify mater we get dat a here
            		}
            	*/
            if((sw1&RESPONSE_SW_SUCESS_MASK1)== RESPONSE_SW_SUCESS_MASK1 || (sw1&RESPONSE_SW_SUCESS_MASK2) == RESPONSE_SW_SUCESS_MASK2) {
                if(sw1 != RESPONSE_SW_SUCESS_MASK2){
										data_length = 0;
								}
							  /*this means  we get data wanted successfuly*/
                m_tpdu.data[data_length++] = sw1;
                m_tpdu.data[data_length++] = sw2;
                Y_SPRINTF("[fm1280]: sw1=0x%02x,sw2=0x%02x length = %d", sw1, sw2, data_length);
//                CLASS(cling_pay_app)*p =cling_pay_app_get_instance();
//                /*send event to 1280 mgr*/
//                p->send_event_from_1280(p, m_tpdu.data, data_length);
            }
            data_sended = true; /*clear data stored in buffer*/
            current_state = FM1280_ACTIVED;
            break;
        }
        }
        break;
    case	FM1280_DEACTIVE:{
			  static int retry_times = 0;
//        if(retry_times < 3) {
//            Y_SPRINTF("iso7816_reinit");
//            iso7816_init ((iso7816_init_t*)&iso7816_init_param);
//            /*if sended sucessfully*/
//            if(wait_process(300)== false) {
//                retry_times ++;
//                iso7816_warm_reset();
//                iso7816_cleanup ();
//                current_state = FM1280_DEACTIVE;
//            } else {
//                current_state = FM1280_SEND;
//                retry_times = 0;
//            }
//        }else{
//						retry_times = 0;
//						current_state = FM1280_IDLE;
//				}
			}
        break;
    default :
        break;
    }

    return ret;
}

static uint32_t on_iso7816_hal_evt (iso7816_hal_evt_t * p_event)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_event->evt_id)
    {
        case ISO7816_HAL_EVT_TPDU_RESPONSE:
            /* Log response. */
            iso7816_hal_log_tpdu_rsp (&tpdu, p_event->evt.tpdu_rsp.sw1, p_event->evt.tpdu_rsp.sw2);

            if (p_event->evt.tpdu_rsp.sw1 == 0x9F)
            {
                get_response (p_event->evt.tpdu_rsp.sw2);
            }
            else if (p_event->evt.tpdu_rsp.sw1 == 0x90)
            {
                m_cont = true;
            }
            break;

        case ISO7816_HAL_EVT_TPDU_TIMEOUT:
            NRF_LOG("ISO7816_HAL_EVT_TPDU_TIMEOUT\r\n");
            m_cont = true;
            break;

        case ISO7816_HAL_EVT_RDY:
            m_sys_rdy = true;
            break;

        case ISO7816_HAL_EVT_NOT_INITIALIZED:
            NRF_LOG("ISO7816_HAL_EVT_NOT_INITIALIZED\r\n");
            break;

        case ISO7816_HAL_EVT_FAULT:
            NRF_LOG("ISO7816_HAL_EVT_FAULT\r\n");
            break;

        default:
            break;
    }

    return err_code;
}


int fm151_test(void)
{
    ret_code_t err_code;
    iso7816_hal_init_t hal_init_param;
    int i;

#ifdef NRF52
    NRF_NVMC->ICACHECNF = NVMC_ICACHECNF_CACHEEN_Enabled << NVMC_ICACHECNF_CACHEEN_Pos;
#endif
    NRF_POWER->TASKS_CONSTLAT = 1;

    //NRF_LOG_INIT ();
    NRF_LOG ("ISO7816-3 PHY/HAL demo.\r\n");

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    memset (&hal_init_param, 0, sizeof(iso7816_hal_init_t));
    hal_init_param.clock_freq  = ISO7816_HAL_FREQ_4MHZ;
    hal_init_param.vcc_pin     = GPIO_7816_VCC;
    hal_init_param.clk_pin     = GPIO_7816_CLK;
    hal_init_param.rst_pin     = GPIO_7816_RST;
    hal_init_param.io_pin      = GPIO_7816_IO;
    hal_init_param.dbg_act_pin = P_NOT_USED;
    hal_init_param.dbg_ppi_pin = P_NOT_USED;
    hal_init_param.evt_cb      = on_iso7816_hal_evt;
		hal_init_param.enable_pps	 = false;

    err_code = iso7816_hal_init (&hal_init_param);
    APP_ERROR_CHECK(err_code);

    NRF_LOG ("Waiting for HAL......");

    SLEEP_UNTIL (m_sys_rdy);
    NRF_LOG ("Done.\r\n");
    print_atr();
    nrf_delay_ms (5);

    for (i = 0; i < RETRY; i++)
    {
        NRF_LOG_PRINTF ("Try #%d:\r\n", i);

        /* send APDU. */
        //select();
				get_cid();

        SLEEP_UNTIL (m_cont);
        m_cont = false;

        NRF_LOG ("\r\n");

        if (i < RETRY - 1)
        {
            /* Suspend. */
            err_code = iso7816_hal_suspend();
            APP_ERROR_CHECK(err_code);
        }

        /* wait (Not low power loop). */
        nrf_delay_ms (SLEEP_TIME * (1 << i));

        if (i < RETRY - 1)
        {
            /* Resume. */
            err_code = iso7816_hal_resume();
            APP_ERROR_CHECK(err_code);
        }
    }

    /* cleanup */
    NRF_LOG ("Clean-up.\r\n");
    err_code = iso7816_hal_uninit ();
    APP_ERROR_CHECK(err_code);

    NRF_LOG ("END.\r\n");

}

/** @} */

