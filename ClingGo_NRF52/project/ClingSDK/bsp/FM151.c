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
#include "ble_pay_app.h"
#include "sys_power_mgr.h"
#include "fm151.h"
#include  "osal.h"
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


#define RESPONSE_SW_RETIEVE_DATA 		    0x61
#define RESPONSE_SW_WRONG_LEGNTH        0X6C
#define RESPONSE_SW_CMD_NOT_SUPPORTED   0X6D
#define RESPONSE_SW_CMD_NULL						0x60
#define RESPONSE_SW_SUCESS_MASK1		    0X60
#define RESPONSE_SW_SUCESS_MASK2		    0X90


#define LOW_POWER_SLEEP         0       /**< Low power idle mode, which will result in lower bit-rate. */
#define SLEEP_TIME              1000
#define RETRY                   100


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
typedef enum {
    fm151_DEACTIVE,
    fm151_IDLE,
    fm151_ACTIVED,
    fm151_SEND,
    fm151_ACTIVING,
    fm151_ACTIVE_END,
    fm151_RESPONSE,
} fm151_state_t;

static volatile bool m_sys_rdy = false;
static volatile bool m_cont = false;
static uint8_t m_buff[128];
iso7816_hal_tpdu_t tpdu;
static bool data_sended = true;
static iso7816_hal_tpdu_t m_tpdu;
static uint8_t sw1 = 0;
static uint8_t sw2 = 0;
static fm151_state_t current_state = fm151_IDLE;
static OS_TASK reg_task = NULL; 
static OS_TIMER timeout_mgr = NULL;



static void print_hex (uint8_t *data, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len - 1; i++) {
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
    tpdu.ins  = 0xA4;   // INS: SELECT
    tpdu.p1   = 0x04;   // P1 : Select MF, DF or ER, with file identifier in data field
    tpdu.p2   = 0x00;   // P2 : 1st or only occurance, return FCI template
    tpdu.req  = ISO7816_HAL_TPDU_REQ_GET;
    tpdu.len  = 0;
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




static uint32_t iso7816_handler (iso7816_hal_evt_t * p_event)
{

    Y_SPRINTF("[fm151]fm151_handler event id= %d", p_event->evt_id);
    switch (p_event->evt_id) {
    case ISO7816_HAL_EVT_TPDU_RESPONSE:
        /* Log response. */
        //iso7816_hal_log_tpdu_rsp (&tpdu, p_event->evt.tpdu_rsp.sw1, p_event->evt.tpdu_rsp.sw2);

        if (p_event->evt.tpdu_rsp.sw1 == 0x9F) {
            get_response (p_event->evt.tpdu_rsp.sw2);
        } else if (p_event->evt.tpdu_rsp.sw1 == 0x90) {
            m_sys_rdy = true;
            sw1 = p_event->evt.tpdu_rsp.sw1;
            sw2 = p_event->evt.tpdu_rsp.sw2;
            current_state = fm151_RESPONSE;
        } else {
            m_sys_rdy = true;
            sw1 = p_event->evt.tpdu_rsp.sw1;
            sw2 = p_event->evt.tpdu_rsp.sw2;
            current_state = fm151_RESPONSE;
        }

        break;

    case ISO7816_HAL_EVT_TPDU_TIMEOUT:
        break;

    case ISO7816_HAL_EVT_RDY:
        m_sys_rdy = true;
        current_state = fm151_ACTIVE_END;
        break;

    case ISO7816_HAL_EVT_NOT_INITIALIZED:
        m_sys_rdy = false;
        current_state = fm151_DEACTIVE;
        break;

    default:
        break;
    }
		if(reg_task != NULL){
			OS_TASK_NOTIFY_FROM_ISR(reg_task, BLE_APP_NOTIFY_CLING_PAY_CMD_MASK, OS_NOTIFY_SET_BITS);
		}
    return NRF_SUCCESS;
}

static iso7816_hal_init_t hal_init_param;
int active_fm151()
{
    
    int i;
    int32_t err_code = 0;

    if(current_state == fm151_IDLE) {


        //NRF_LOG_INIT ();
        //NRF_LOG ("ISO7816-3 PHY/HAL demo.\r\n");

        err_code = nrf_drv_ppi_init();
        //APP_ERROR_CHECK(err_code);
			
				if(nrf_drv_gpiote_is_init() == false){
					err_code = nrf_drv_gpiote_init();
					APP_ERROR_CHECK(err_code);
				}

        memset (&hal_init_param, 0, sizeof(iso7816_hal_init_t));
        hal_init_param.clock_freq  = ISO7816_PHY_FREQ_1MHZ;
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

        current_state  = fm151_ACTIVING;
        err_code = NRF_ERROR_BUSY;
    }
    return err_code;
}

int deactive_fm151()
{
    int32_t err_code = NRF_SUCCESS;
    iso7816_hal_uninit ();
		current_state = fm151_IDLE;
		OS_TIMER_STOP(timeout_mgr, 0);
    return err_code;
}


static int fm151_send(uint8_t *data, size_t length)
{
#define TPFU_GET_REQ_PACKAGE_SIZE  5
    int ret = 0;
    if(current_state == fm151_ACTIVED) {
        iso7816_hal_tpdu_t tpdu;
        int i = 0;
        tpdu.cla  = data[i++];   // CLA: Proprietary, GSM
        tpdu.ins  = data[i++];   // INS: GET RESPONSE
        tpdu.p1   = data[i++];   // P1 : reserved
        tpdu.p2   = data[i++];  // P2 : reserved
        if(length <= TPFU_GET_REQ_PACKAGE_SIZE) {
            tpdu.req  = ISO7816_HAL_TPDU_REQ_GET;
        } else {
            tpdu.req  = ISO7816_HAL_TPDU_REQ_PUT;
        }
        //tpdu.req  = ISO7816_TPDU_REQ_PUT;
        tpdu.len  = data[i++];
        tpdu.data = &data[i++];
        memcpy(&m_tpdu, &tpdu, sizeof(tpdu));
        Y_SPRINTF("[fm151]: cla = 0x%02x, ins = 0x%02x REQ = 0X%02X", tpdu.cla, tpdu.ins, tpdu.req);
        current_state = fm151_SEND;
        data_sended = false;
    } else {

        ret = -1;
    }
    return ret;
}

static uint32_t iso7816_fsm_init()
{

    current_state = fm151_IDLE;
    return 1;
}




static int core_fsm(void *data)
{
#define CLING_PAY_TIMEOUT_IDLE   600000
    static  uint16_t data_length = 0;
    static uint8_t last_state=0xff;
    static uint32_t t_curr = 0;
    uint32_t err_code = NRF_SUCCESS;
    int ret = 0;
    if(current_state != last_state) {
        Y_SPRINTF("[fm151]state change to %d", current_state);
        last_state = current_state;
        t_curr = CLK_get_system_time();
				OS_ASSERT(OS_TIMER_CHANGE_PERIOD(timeout_mgr, OS_MS_2_TICKS(CLING_PAY_TIMEOUT_IDLE), 0) == OS_TIMER_SUCCESS);
				OS_ASSERT(OS_TIMER_START(timeout_mgr, 0) == OS_TIMER_SUCCESS);
    } else {
        /*when no oepration happened in 30s to fsm 1280 switch to */
        if(CLK_get_system_time() - t_curr >= CLING_PAY_TIMEOUT_IDLE) {
            if(current_state != fm151_IDLE) {
                Y_SPRINTF("[fm151] fsm1280 active timeout");
                deactive_fm151();
                current_state = fm151_IDLE;
            }
            t_curr = CLK_get_system_time();
        }
    }

    switch(current_state) {
    case  fm151_IDLE:
        t_curr = CLK_get_system_time();
        break;
    case fm151_ACTIVING:
        break;
    case fm151_ACTIVE_END:  {
        static const uint8_t tmp[] = {0x90, 0x00};
        CLASS(cling_pay_app)*p =cling_pay_app_get_instance();
        /*notify app 1280 has been actived sucessfully*/
        p->send_event_from_1280(p, (uint8_t *)tmp, sizeof(tmp));
         current_state = fm151_ACTIVED;
    }
    break;

    case	fm151_ACTIVED:
        break;
    case   fm151_SEND:
        if(data_sended  == false) {
            err_code = iso7816_hal_send_tpdu (&m_tpdu);
            if(err_code == NRF_ERROR_BUSY) {

                Y_SPRINTF("NRF_ERROR_BUSY 0");
                //iso7816_hal_cleanup ();
                current_state = fm151_DEACTIVE;
                break;
            } else {
                data_sended = true;
            }
            /*since every transmition cycle starts with ISO7816_HAL_TPDU_REQ_GET, data lenth recorded for echaed
            	cycle should be cleared once a new one get requested*/
            if(m_tpdu.req == ISO7816_HAL_TPDU_REQ_PUT) {
                data_length = 0;
            } else {
                data_length = m_tpdu.len;
            }
        } else {
            /*if no data pending, put state machine into active fress state*/
            current_state = fm151_ACTIVED;
        }
        break;
    case	fm151_RESPONSE:
        //Y_SPRINTF("[fm151]: sw1=0x%02x,sw2=0x%02x", sw1, sw2);
        switch(sw1) {
        case RESPONSE_SW_WRONG_LEGNTH: {
            m_tpdu.len = sw2;
            m_tpdu.req = ISO7816_HAL_TPDU_REQ_GET;
            data_sended = false;
            current_state = fm151_SEND;
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
            current_state = fm151_SEND;
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
                if(sw1 != RESPONSE_SW_SUCESS_MASK2) {
                    data_length = 0;
                }
                /*this means  we get data wanted successfuly*/
                m_tpdu.data[data_length++] = sw1;
                m_tpdu.data[data_length++] = sw2;
                Y_SPRINTF("[fm151]: sw1=0x%02x,sw2=0x%02x length = %d", sw1, sw2, data_length);
                CLASS(cling_pay_app)*p =cling_pay_app_get_instance();
                /*send event to 1280 mgr*/
                p->send_event_from_1280(p, m_tpdu.data, data_length);
            }
            data_sended = true; /*clear data stored in buffer*/
            current_state = fm151_ACTIVED;
            break;
        }
        }
        break;
    case	fm151_DEACTIVE: {
        static int retry_times = 0;
        if(retry_times < 3) {
            Y_SPRINTF("iso7816_reinit");
						OS_ASSERT(OS_TIMER_CHANGE_PERIOD(timeout_mgr, OS_MS_2_TICKS(300), 0) == OS_TIMER_SUCCESS);
						OS_ASSERT(OS_TIMER_START(timeout_mgr, 0) == OS_TIMER_SUCCESS);
            /*if sended sucessfully*/
            if(m_sys_rdy == false) {
								OS_TIMER_STOP(timeout_mgr, 0);
                retry_times ++;
								iso7816_hal_uninit();
                current_state = fm151_DEACTIVE;
								iso7816_hal_init(&hal_init_param);
								OS_TIMER_START(timeout_mgr, 0);
            } else {
								OS_TIMER_STOP(timeout_mgr, 0);
                current_state = fm151_SEND;
                retry_times = 0;
								ret = 1;
            }
        }else{
						retry_times = 0;
						current_state = fm151_IDLE;
				}
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

    switch (p_event->evt_id) {
    case ISO7816_HAL_EVT_TPDU_RESPONSE:
        /* Log response. */
        //iso7816_hal_log_tpdu_rsp (&tpdu, p_event->evt.tpdu_rsp.sw1, p_event->evt.tpdu_rsp.sw2);

        if (p_event->evt.tpdu_rsp.sw1 == 0x9F) {
            get_response (p_event->evt.tpdu_rsp.sw2);
        } else if (p_event->evt.tpdu_rsp.sw1 == 0x90) {
            m_cont = true;
        }
				m_cont = true;
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



static bool ad_fm151_prepare_for_sleep(void)
{
    bool ret = true;
    if(current_state != fm151_IDLE) {
        ret = false;
    }
    return ret;
}


static void ad_fm151_sleep_canceled(void)
{

}

static void ad_fm151_wake_up_ind(bool arg)
{

}

static const adapter_call_backs_t ad_fm151_pm_call_backs = {
    .ad_prepare_for_sleep = ad_fm151_prepare_for_sleep,
    .ad_sleep_canceled = ad_fm151_sleep_canceled,
    .ad_wake_up_ind = ad_fm151_wake_up_ind,
    .ad_xtal16m_ready_ind = NULL,
    .ad_sleep_preparation_time = 0
};



static struct fm151_adapter_call_backs callback = {
    .init = NULL,
    .core_fsm = core_fsm,
    .rx_cb = NULL,
};


/******************************************************************************
* FunctionName :isr_routine
* Description  : get hal rtc instance
* Parameters   : write: write call back function
* Returns          : 0: sucess
                     -1: error
*******************************************************************************/
static void isr_routine(OS_TIMER t)
{
        OS_TIMER_STOP(t, 0);
				N_SPRINTF("[FM151] Rtc Isr Routine Entered \r\n");
				OS_TASK_NOTIFY_FROM_ISR(OS_GET_CURRENT_TASK(), BLE_APP_NOTIFY_CLING_PAY_CMD_MASK, OS_NOTIFY_SET_BITS);
        return ;
}

void ISO7816_Init()
{
#ifdef NRF52
       // NRF_NVMC->ICACHECNF = NVMC_ICACHECNF_CACHEEN_Enabled << NVMC_ICACHECNF_CACHEEN_Pos;
#endif
    //    NRF_POWER->TASKS_CONSTLAT = 1;
		Y_SPRINTF("[FM151] ISO7816_Init \r\n");
		reg_task = OS_GET_CURRENT_TASK();
    CLASS(cling_pay_app)*p =cling_pay_app_get_instance();
    p->reg_adaptor(p, (struct ble_pay_adapter_call_backs *)&callback);
    p->reg_iso7816_tx(p, fm151_send);
		timeout_mgr = OS_TIMER_CREATE("PAY_timer", OS_MS_2_TICKS(1000), pdTRUE, (void*)0, isr_routine);
		OS_ASSERT(timeout_mgr != NULL);
    iso7816_fsm_init();
    pm_register_adapter(&ad_fm151_pm_call_backs);
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
    hal_init_param.enable_pps	 = 	false;

    err_code = iso7816_hal_init (&hal_init_param);
    APP_ERROR_CHECK(err_code);

    NRF_LOG ("Waiting for HAL......");

    SLEEP_UNTIL (m_sys_rdy);
    NRF_LOG ("Done.\r\n");
    print_atr();
    nrf_delay_ms (5);

    for (i = 0; i < RETRY; i++) {
        NRF_LOG_PRINTF ("Try #%d:\r\n", i);

        /* send APDU. */
        //select();
        get_cid();

        SLEEP_UNTIL (m_cont);
        m_cont = false;

        NRF_LOG ("\r\n");

//        if (i < RETRY - 1) {
//            /* Suspend. */
//            err_code = iso7816_hal_suspend();
//            APP_ERROR_CHECK(err_code);
//        }

//        /* wait (Not low power loop). */
//        nrf_delay_ms (SLEEP_TIME * (1 << i));

//        if (i < RETRY - 1) {
//            /* Resume. */
//            err_code = iso7816_hal_resume();
//            APP_ERROR_CHECK(err_code);
//        }
    }

    /* cleanup */
    NRF_LOG ("Clean-up.\r\n");
    err_code = iso7816_hal_uninit ();
    APP_ERROR_CHECK(err_code);

    NRF_LOG ("END.\r\n");

}

/** @} */

