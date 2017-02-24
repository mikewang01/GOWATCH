/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
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

/**@cond To Make Doxygen skip documentation generation for this file.
 * @{
 */

#include "iso7816_phy.h"
#include "iso7816_hal.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "sdk_common.h"
#include <string.h>
#include "main.h"

/**
 * @defgroup iso7816_hal ISO7816-3 HAL.
 * @{
 */

/**
 * @defgroup iso7816_hal_const Constants
 * @{
 */

#define ISO7816_HAL_INVALID_BYTE            0xFF        /**< Invalid byte. */

#define ISO7816_HAL_ATR_INVERSE             0x03        /**< First byte of ATR, indicates inverse coding. */
#define ISO7816_HAL_ATR_DIRECT              0x3B        /**< First byte of ATR, indicates direct coding. */
#define ISO7816_HAL_ATR_MAX_LEN             32          /**< Maximum size of ATR. */
#define ISO7816_HAL_ATR_MAX_WAIT            40000       /**< Maximum waiting time for the first character after ATR. */

#define ISO7816_HAL_PPS_MAX_LEN             6           /**< Maximum size of PPS request/response message. */
#define ISO7816_HAL_PPSS                    0xFF        /**< First byte of PPS messages. */
#define ISO7816_HAL_PPS_EXPECTED_LEN        4           /**< Expected length of PPS response. */

#define ISO7816_HAL_TPDU_HDR_LEN            5           /**< TPDU header length. */
#define ISO7816_HAL_TPDU_PROCD_NULL         0x60        /**< TPDU procedure byte: NULL. */
#define ISO7816_HAL_TPDU_PROCD_SW1_MASK     0xF0        /**< TPDU procedure byte: mask to check SW1. */
#define ISO7816_HAL_TPDU_PROCD_SW1_OK       0x90        /**< TPDU procedure byte: first half of SW1 - OK. */
#define ISO7816_HAL_TPDU_PROCD_SW1_ERR      0x60        /**< TPDU procedure byte: first half of SW1 - ERROR. */

#define ISO7816_HAL_PREFERRED_IDLE_CLK_LV   0x00        /**< Preferred CLK level (if not specified in ATR). */

/** @} */ // End tag for iso7816_hal_const


/**
 * @defgroup iso7816_hal_enums Enumerations
 * @{
 */

/**@brief Finite state machine (system). */
typedef enum
{
    ISO7816_HAL_SYS_FSM_OFF = 0,            /**< ISO7816 module is turned off. */
    ISO7816_HAL_SYS_FSM_INIT,               /**< Initialization in progress, waiting for ATR. */
    ISO7816_HAL_SYS_FSM_ATR_RECVD,          /**< ATR received, parse and exchange PPS whenever necessary. */
    ISO7816_HAL_SYS_FSM_RDY,                /**< System ready for Tx/Rx. */
    ISO7816_HAL_SYS_FSM_SUSPENDED,          /**< System suspended. */
    ISO7816_HAL_SYS_FSM_MAX                 /**< Invalid. */
} iso7816_hal_sys_fsm_t;


/**@brief Events for system finite state machine. */
typedef enum
{
    ISO7816_HAL_SYS_FSM_EVT_RX = 0,         /**< Indicates that a byte is received. */
    ISO7816_HAL_SYS_FSM_EVT_TX,             /**< Indicates that data is transmitted. */
    ISO7816_HAL_SYS_FSM_EVT_TIMEOUT,        /**< System FSM timer timeout. */
    ISO7816_HAL_SYS_FSM_EVT_SUSPEND,        /**< Suspend. */
    ISO7816_HAL_SYS_FSM_EVT_RESUME,         /**< Resume. */
    ISO7816_HAL_SYS_FSM_EVT_CLEANUP,        /**< Clean-up (destructor). */
    ISO7816_HAL_SYS_FSM_EVT_MAX             /**< Invalid. */
} iso7816_hal_sys_fsm_evt_t;


/**@brief Finite state machine (ATR). */
typedef enum
{
    ISO7816_HAL_ATR_FSM_INIT = 0,           /**< Initial: Expects Format byte T0. */
    ISO7816_HAL_ATR_FSM_FORMAT,             /**< Expects Format byte T0. */
    ISO7816_HAL_ATR_FSM_GLOBAL_1,           /**< Expects global values in TA1, TC1 and TD1. */
    ISO7816_HAL_ATR_FSM_GLOBAL_2,           /**< Expects global values in TA2, TC2 and TD2. */
    ISO7816_HAL_ATR_FSM_STRUCTURAL,         /**< Expects structural values in TAi, TCi and TDi. */
    ISO7816_HAL_ATR_FSM_HISTORICAL,         /**< Expects historical bytes. */
    ISO7816_HAL_ATR_FSM_TCK,                /**< Expects TCK. */
    ISO7816_HAL_ATR_FSM_RDY,                /**< ATR Ready. */
    ISO7816_HAL_ATR_FSM_FAIL,               /**< ATR failed. */
    ISO7816_HAL_ATR_FSM_MAX                 /**< Invalid. */
} iso7816_hal_atr_fsm_t;


/**@brief Finite state machine (PPS). */
typedef enum
{
    ISO7816_HAL_PPS_FSM_WAITING = 0,        /**< Receiving PPSS. */
    ISO7816_HAL_PPS_FSM_PPS0,               /**< Receiving PPS0. */
    ISO7816_HAL_PPS_FSM_PPSX,               /**< Receiving PPS1~PPS3. */
    ISO7816_HAL_PPS_FSM_PCK,                /**< Receiving PCK. */
    ISO7816_HAL_PPS_FSM_RDY,                /**< PPS ready. */
    ISO7816_HAL_PPS_FSM_FAIL,               /**< PPS failed. */
    ISO7816_HAL_PPS_FSM_MAX                 /**< Invalid. */
} iso7816_hal_pps_fsm_t;


/**@brief Finite state machine (TPDU exchange). */
typedef enum
{
    ISO7816_HAL_TPDU_FSM_DISABLED = 0,      /**< System disabled. */
    ISO7816_HAL_TPDU_FSM_INIT,              /**< Initialization in progress. */
    ISO7816_HAL_TPDU_FSM_IDLE,              /**< Idle state, can receive request from application. */
    ISO7816_HAL_TPDU_FSM_HEADER,            /**< Sending header. */
    ISO7816_HAL_TPDU_FSM_LISTEN,            /**< Listening to procedure bytes. */
    ISO7816_HAL_TPDU_FSM_TX_DATA,           /**< Sending data (1 or multiple bytes) to card. */
    ISO7816_HAL_TPDU_FSM_RX_DATA,           /**< Receiving data from card. */
    ISO7816_HAL_TPDU_FSM_RX_1BYTE,          /**< Receiving 1-byte from card. */
    ISO7816_HAL_TPDU_FSM_SW1,               /**< Received SW1, waiting for SW2. */
    ISO7816_HAL_TPDU_FSM_MAX                /**< Invalid. */
} iso7816_hal_tpdu_fsm_t;


/**@brief Events for TPDU exchange finite state machine. */
typedef enum
{
    ISO7816_HAL_TPDU_FSM_EVT_REQ,           /**< Request from application. */
    ISO7816_HAL_TPDU_FSM_EVT_TX_RDY,        /**< Data transmitted. */
    ISO7816_HAL_TPDU_FSM_EVT_RX_DATA,       /**< On receive of a byte, be it procedure byte or data. */
    ISO7816_HAL_TPDU_FSM_EVT_TIMEOUT,       /**< Timeout. */
    ISO7816_HAL_TPDU_FSM_EVT_SYS_RDY,       /**< ISO7816 PHY and HAL modules has been initialized. */
    ISO7816_HAL_TPDU_FSM_EVT_MAX            /**< Invalid. */
} iso7816_hal_tpdu_fsm_evt_t;

/** @} */ // End tag for iso7816_hal_enums


/**
 * @defgroup iso7816_hal_types Types
 * @{
 */

/**@brief Driver context */
typedef struct
{
    iso7816_hal_sys_fsm_t      sys_state;                           /**< System finite state machine. */
    iso7816_hal_evt_handler_t  evt_cb;                              /**< Event callback. */
    iso7816_hal_freq_t         clk_freq;                            /**< Clock frequency. */

    uint8_t                    rx_byte_cnt;                         /**< Rx byte counter */
    bool                       phy_suspended;                       /**< Rx byte counter */

    iso7816_hal_atr_fsm_t      atr_state;                           /**< ATR finite state machine. */
    uint16_t                   T;                                   /**< T (each bit denotes T=0, T=1....T=15.). */
    uint8_t                    last_T;                              /**< Last known T. */
    uint8_t                    Y;                                   /**< Last known Yi. */
    uint8_t                    K;                                   /**< Number of Historical bytes. */
    uint16_t                   Fi;                                  /**< Cached Fi. */
    uint8_t                    Di;                                  /**< Cached Di. */
    uint8_t                    WI;                                  /**< Waiting interval WI. */
    bool                       neogatiable;                         /**< Ability for the SIM card to enter negotiable mode. */
    bool                       use_Fi_Di;                           /**< Use Fi/Di (true), or default (false). */
    bool                       ignore_atr;                          /**< Ignore ATR parameters: default false. */
    bool                       wt_enabled;                          /**< Indicates if WT is enabled. */
    bool                       clk_stop_supported;                  /**< Indicates if clock-stop is supported. */
    uint8_t                    clk_stop_lv;                         /**< Clock stop level. */
    uint8_t                    atr [ISO7816_HAL_ATR_MAX_LEN];       /**< ATR string. */
    uint8_t                    atr_len;                             /**< ATR string length. */

    iso7816_hal_pps_fsm_t      pps_state;                           /**< PPS finite state machine. */
    uint8_t                    pps_req[ISO7816_HAL_PPS_MAX_LEN];    /**< PPS request. */
    uint8_t                    pps_rsp[ISO7816_HAL_PPS_MAX_LEN];    /**< PPS response. */
    uint8_t                    pps_n;                               /**< Number of PPS1~PPS3 received. */

    iso7816_hal_tpdu_fsm_t     tpdu_state;                          /**< TPDU exchange finite state machine. */
    iso7816_hal_tpdu_t         tpdu;                                /**< Back up of TPDU request from application. */
    uint8_t                    tpdu_hdr[ISO7816_HAL_TPDU_HDR_LEN];  /**< TPDU header length. */
    iso7816_hal_evt_tpdu_rsp_t tpdu_rsp;                            /**< TPDU response. */
    bool                       tpdu_pended;                         /**< Indicate whether a TPDU has been pended because of HPY suspension. */

    uint8_t                    rx_buff;                             /**< 1-byte Rx buffer. */
		
		bool											 enable_pps;													/**< Enable PPS negocitation. */
} iso7816_hal_t;

/** @} */ // End tag for iso7816_hal_types

static iso7816_hal_t m_iso7816_hal;                                 /**< Driver context. */
static const uint8_t m_pps_def_req[ISO7816_HAL_PPS_MAX_LEN]         /**< Default PPS request. */
    = {ISO7816_HAL_PPSS, 0x10, 0x01, 0x00};


/**
 * @defgroup iso7816_hal_lut Look-up tables
 * @{
 */

/**@brief Fi lookup table (see ISO7816-3, table 7) */
const uint16_t iso7816_hal_Fi_lut [16]
    = { 372,  372,  558,  744, 1116, 1488, 1860,    0,
          0,  512,  768, 1024, 1536, 2048,    0,    0};


/**@brief Di lookup table (see ISO7816-3, table 8) */
const uint16_t iso7816_hal_Di_lut [16]
    = { 0,  1,  2,  4,  8, 16, 32, 64,
       12, 20,  0,  0,  0,  0,  0,  0};


/**@brief Maximum of Di index supported by this implementation */
#ifdef NRF51
const uint16_t iso7816_hal_MaxDi_idx_lut [3][16]
    = {
        {4, 4, 4, 5, 5, 6, 6, 1, 1, 4, 5, 5, 6, 6, 1, 1},
        {3, 3, 3, 4, 4, 5, 5, 1, 1, 3, 4, 4, 5, 5, 1, 1},
        {2, 2, 2, 3, 3, 4, 4, 1, 1, 2, 3, 3, 4, 4, 1, 1}
    };
#elif defined(NRF52)
    #ifdef USE_EASYDMA
    const uint16_t iso7816_hal_MaxDi_idx_lut [3][16]        // Measured MAX: 250 kbps
        = {
            {7, 7, 7, 7, 7, 7, 7, 1, 1, 7, 7, 7, 7, 7, 1, 1},
            {6, 6, 7, 7, 7, 7, 7, 1, 1, 7, 7, 7, 7, 7, 1, 1},
            {5, 5, 6, 6, 7, 7, 7, 1, 1, 6, 6, 7, 7, 7, 1, 1}
        };
    #else
    const uint16_t iso7816_hal_MaxDi_idx_lut [3][16]        // Measured MAX: 62.5 kbps
        = {
            {5, 5, 6, 6, 7, 7, 7, 1, 1, 6, 6, 7, 7, 7, 1, 1},
            {4, 4, 5, 5, 6, 6, 6, 1, 1, 5, 5, 6, 6, 7, 1, 1},
            {3, 3, 4, 4, 5, 5, 5, 1, 1, 4, 4, 5, 5, 6, 1, 1}
        };
    #endif
#else
    #error Platform not supported.
#endif


/**@brief 3-bit bit-count lookup table */
const uint8_t iso7816_hal_bcnt_lut [8]
    = {0, 1, 1, 2, 1, 2, 2, 3};

/** @} */ // End tag for iso7816_hal_lut


/** @defgroup iso7816_hal_func Functions */

static uint32_t on_iso7816_phy_evt (iso7816_phy_evt_t * p_event);

static void set_tx (void);
static void set_rx (uint8_t * buff, uint8_t len);
static void disable_txrx (void);

static void atr_parser (uint8_t data);
static void pps_parser (uint8_t data);

static void try_recovery (iso7816_hal_evt_id_t evt_when_failed);

static void sys_fsm (iso7816_hal_sys_fsm_evt_t evt, uint8_t * data, uint16_t len);
static uint32_t tpdu_fsm (iso7816_hal_tpdu_fsm_evt_t evt, void * data, uint16_t len);


/**@brief ISO7816 PHY layer event handler */
static uint32_t on_iso7816_phy_evt (iso7816_phy_evt_t * p_event)
{
    uint32_t err_code = NRF_SUCCESS;
    iso7816_hal_evt_t l_evt;

    switch (p_event->evt_id)
    {
        case ISO7816_PHY_EVT_RX_DATA:
            sys_fsm (ISO7816_HAL_SYS_FSM_EVT_RX, p_event->evt.rx_data.data, p_event->evt.rx_data.len);
            break;

        case ISO7816_PHY_EVT_TX_DONE:
            sys_fsm (ISO7816_HAL_SYS_FSM_EVT_TX, 0, p_event->evt.tx_done.len);
            break;

        case ISO7816_PHY_EVT_TIMEOUT:
            sys_fsm (ISO7816_HAL_SYS_FSM_EVT_TIMEOUT, 0, 0);
            break;

        case ISO7816_PHY_EVT_CLK_STOPPED:
            sys_fsm (ISO7816_HAL_SYS_FSM_EVT_SUSPEND, 0, 0);
            break;

        case ISO7816_PHY_EVT_CLK_RESUMED:
            sys_fsm (ISO7816_HAL_SYS_FSM_EVT_RESUME, 0, 0);
            break;

        case ISO7816_PHY_EVT_WARM_RESET:
            /* Re-enable initialization. */
            m_iso7816_hal.sys_state = ISO7816_HAL_SYS_FSM_INIT;

            err_code = iso7816_phy_timer_start (ISO7816_HAL_ATR_MAX_WAIT);
            VERIFY_SUCCESS(err_code);
            break;

        case ISO7816_PHY_EVT_COLD_RESET:
            /* Start SE-detection timer. */
            err_code = iso7816_phy_timer_start (ISO7816_HAL_ATR_MAX_WAIT);
            VERIFY_SUCCESS(err_code);
            break;

        case ISO7816_PHY_EVT_RST_TIMEOUT:
            /* The RST signal cannot bee pulled up. */
            l_evt.evt_id = ISO7816_HAL_EVT_NOT_INITIALIZED;
            m_iso7816_hal.evt_cb(&l_evt);
            break;

        default:
            break;
    }

    return err_code;
}


/**@brief Function for setting PHY to Tx mode */
static void set_tx (void)
{
    uint32_t err_code;

    err_code = iso7816_phy_tx_enable ();
    VERIFY_SUCCESS_VOID(err_code);
}


/**@brief Function for setting PHY to Rx mode */
static void set_rx (uint8_t * buff, uint8_t len)
{
    uint32_t err_code;

    err_code = iso7816_phy_rx_enable (buff, len);
    VERIFY_SUCCESS_VOID(err_code);
}


/**@brief Disable PHY Tx/Rx. */
static void disable_txrx (void)
{
    uint32_t err_code;

    err_code = iso7816_phy_txrx_disable ();
    VERIFY_SUCCESS_VOID(err_code);
}


/**@brief Function for parsing ATR string. */
static void atr_parser (uint8_t data)
{
    uint8_t  i, x;
    uint32_t err_code = NRF_SUCCESS;
    iso7816_hal_evt_t l_evt;

    m_iso7816_hal.atr [m_iso7816_hal.rx_byte_cnt] = data;

    /* Check entry state */ 
    if (m_iso7816_hal.atr_state != ISO7816_HAL_ATR_FSM_RDY)
    {
        m_iso7816_hal.rx_byte_cnt++;
    }

    switch (m_iso7816_hal.atr_state)
    {
        case ISO7816_HAL_ATR_FSM_INIT:      // TS: initial character
            err_code = iso7816_phy_set_coding (data == ISO7816_HAL_ATR_DIRECT);
            if (err_code != NRF_SUCCESS)
            {
                l_evt.evt_id = ISO7816_HAL_EVT_NOT_INITIALIZED;
                m_iso7816_hal.evt_cb(&l_evt);

                m_iso7816_hal.atr_state = ISO7816_HAL_ATR_FSM_FAIL;
                return;
            }

            m_iso7816_hal.atr_state = ISO7816_HAL_ATR_FSM_FORMAT;
            break;

        case ISO7816_HAL_ATR_FSM_FORMAT:    // T0: encodes Y1 and K
            m_iso7816_hal.Y = (data >> 4) & 0xF;
            m_iso7816_hal.K = data & 0xF;

            m_iso7816_hal.atr_state = ISO7816_HAL_ATR_FSM_GLOBAL_1;
            break;

        case ISO7816_HAL_ATR_FSM_GLOBAL_1:
            for (i = 0; i < 4; i++)
            {
                if ((m_iso7816_hal.Y & (1 << i)) != 0)
                {
                    m_iso7816_hal.Y &= ~(1 << i);

                    switch (i)
                    {
                        case 0:     // TA1: encodes Fi and Di
                            /* Prepare PPS request */
                            if (m_iso7816_hal.ignore_atr)
                            {
                                m_iso7816_hal.Fi = ISO7816_PHY_Fd;
                                m_iso7816_hal.Di = ISO7816_PHY_Dd;
                                m_iso7816_hal.pps_req [2] = 0x11;
                            }
                            else
                            {
                                uint8_t idx_Fi, idx_Di;
                                
                                idx_Fi = (data >> 4) & 0xF;
                                idx_Di = data & 0xF;

                                if (idx_Di > iso7816_hal_MaxDi_idx_lut [m_iso7816_hal.clk_freq][idx_Fi])
                                {
                                    idx_Di = iso7816_hal_MaxDi_idx_lut [m_iso7816_hal.clk_freq][idx_Fi];
                                }

                                m_iso7816_hal.Fi = iso7816_hal_Fi_lut [idx_Fi];
                                m_iso7816_hal.Di = iso7816_hal_Di_lut [idx_Di];
                                m_iso7816_hal.pps_req [2] = (idx_Fi << 4) | idx_Di;
                            }
								
                            m_iso7816_hal.pps_req [3] = m_iso7816_hal.pps_req [0] ^ m_iso7816_hal.pps_req [1] ^ m_iso7816_hal.pps_req [2];  // xsum
                            break;

                        case 2:     // TC1: encodes extra guard time, N
                            if (data < 255)
                            {
                                iso7816_phy_set_N (data);
                            }
                            break;

                        case 3:     // TD1: encodes Y2 and T
                            m_iso7816_hal.Y       = (data >> 4) & 0xF;
                            m_iso7816_hal.last_T  = data & 0xF;
                            m_iso7816_hal.T      |= 1 << m_iso7816_hal.last_T;

                            m_iso7816_hal.atr_state = ISO7816_HAL_ATR_FSM_GLOBAL_2;
                            break;

                        default:    // TB1 is deprecated
                            break;
                    }

                    if ((m_iso7816_hal.Y == 0) && (i != 3))
                    {
                        /* TD1 not found */
                        m_iso7816_hal.atr_state = (m_iso7816_hal.K != 0)? ISO7816_HAL_ATR_FSM_HISTORICAL: ISO7816_HAL_ATR_FSM_RDY;
                    }

                    i = 3;          // terminate the for-loop, will enter again on next byte.
                }
            }
            break;

        case ISO7816_HAL_ATR_FSM_GLOBAL_2:
            for (i = 0; i < 4; i++)
            {
                if ((m_iso7816_hal.Y & (1 << i)) != 0)
                {
                    m_iso7816_hal.Y &= ~(1 << i);

                    switch (i)
                    {
                        case 0:         // TA2: encodes negotiation parameters
                            m_iso7816_hal.neogatiable = ((data & 0x80) == 0);
                            m_iso7816_hal.use_Fi_Di   = ((data & 0x10) == 0);
                            m_iso7816_hal.last_T      = data & 0xF;
                            m_iso7816_hal.T          |= 1 << m_iso7816_hal.last_T;
                            break;

                        case 2:         // Specific to T=0: waiting interval
                            m_iso7816_hal.WI = data;
                            iso7816_phy_set_WT (data * 960);
                            break;

                        case 3:
                            m_iso7816_hal.Y       = (data >> 4) & 0xF;
                            m_iso7816_hal.last_T  = data & 0xF;
                            m_iso7816_hal.T      |= 1 << m_iso7816_hal.last_T;

                            m_iso7816_hal.atr_state = ISO7816_HAL_ATR_FSM_STRUCTURAL;
                            break;

                        default:        // TB2 is deprecated
                            break;
                    }

                    if ((m_iso7816_hal.Y == 0) && (i != 3))
                    {
                        /* TD2 not found */
                        m_iso7816_hal.atr_state = (m_iso7816_hal.K != 0)? ISO7816_HAL_ATR_FSM_HISTORICAL: ISO7816_HAL_ATR_FSM_RDY;
                    }

                    i = 3;          // terminate the for-loop, will enter again on next byte.
                }
            }
            break;

        case ISO7816_HAL_ATR_FSM_STRUCTURAL:
            for (i = 0; i < 4; i++)
            {
                if ((m_iso7816_hal.Y & (1 << i)) != 0)
                {
                    m_iso7816_hal.Y &= ~(1 << i);

                    switch (i)
                    {
                        case 0:         // TAi
                            if (m_iso7816_hal.last_T == 15)
                            {
                                m_iso7816_hal.clk_stop_supported = ((data & 0xC0) != 0);
                                m_iso7816_hal.clk_stop_lv        = ((data >> 6) & 1) ^ (1 ^ ISO7816_HAL_PREFERRED_IDLE_CLK_LV);
                            }
                            break;

                        case 1:         // TBi: ignored
                        case 2:         // TCi: ignored
                            break;

                        case 3:
                            m_iso7816_hal.Y       = (data >> 4) & 0xF;
                            m_iso7816_hal.last_T  = data & 0xF;
                            m_iso7816_hal.T      |= 1 << m_iso7816_hal.last_T;
                            break;

                        default:
                            break;
                    }

                    if ((m_iso7816_hal.Y == 0) && (i != 3))
                    {
                        /* TDi not found */
                        m_iso7816_hal.atr_state = (m_iso7816_hal.K != 0)? ISO7816_HAL_ATR_FSM_HISTORICAL: ISO7816_HAL_ATR_FSM_RDY;
                    }

                    i = 3;          // terminate the for-loop, will enter again on next byte.
                }
            }
            break;

        case ISO7816_HAL_ATR_FSM_HISTORICAL:
            if (--m_iso7816_hal.K == 0)
            {
                /* requires TCK check if NOT "only T=0 is indicated" */
                m_iso7816_hal.atr_state = (m_iso7816_hal.T != 0x0001)? ISO7816_HAL_ATR_FSM_TCK: ISO7816_HAL_ATR_FSM_RDY;
            }
            break;

        case ISO7816_HAL_ATR_FSM_TCK:
            /* check TCK */
            x = 0;
            for (i = 1; i < m_iso7816_hal.rx_byte_cnt; i++)
            {
                x ^= m_iso7816_hal.atr [i];
            }

            m_iso7816_hal.atr_state = (x == 0)? ISO7816_HAL_ATR_FSM_RDY: ISO7816_HAL_ATR_FSM_FAIL;
            break;

        case ISO7816_HAL_ATR_FSM_FAIL:
        case ISO7816_HAL_ATR_FSM_RDY:
        default:
            break;
    }
}


/**@brief PPS response parser */
static void pps_parser (uint8_t data)
{
    uint8_t j, x;

    m_iso7816_hal.pps_rsp [m_iso7816_hal.rx_byte_cnt++] = data;

    switch (m_iso7816_hal.pps_state)
    {
        case ISO7816_HAL_PPS_FSM_WAITING:
            m_iso7816_hal.pps_state = (data == ISO7816_HAL_PPSS)? ISO7816_HAL_PPS_FSM_PPS0: ISO7816_HAL_PPS_FSM_FAIL;
            break;

        case ISO7816_HAL_PPS_FSM_PPS0:
            m_iso7816_hal.pps_state = (((data >> 4) & 0x7) != 0)? ISO7816_HAL_PPS_FSM_PPSX: ISO7816_HAL_PPS_FSM_PCK;
            m_iso7816_hal.pps_n = 0;
            break;

        case ISO7816_HAL_PPS_FSM_PPSX:
            if (++m_iso7816_hal.pps_n >= iso7816_hal_bcnt_lut [(m_iso7816_hal.pps_rsp[1] >> 4) & 0x7])
            {
                m_iso7816_hal.pps_state = ISO7816_HAL_PPS_FSM_PCK;
            }
            break;

        case ISO7816_HAL_PPS_FSM_PCK:
            /* Check-sum */
            x = 0;
            for (j = 0; j < m_iso7816_hal.rx_byte_cnt; j++)
            {
                x ^= m_iso7816_hal.pps_rsp[j];
            }

            if (x != 0)
            {
                m_iso7816_hal.pps_state = ISO7816_HAL_PPS_FSM_FAIL;
                break;
            }

            /* T */
            if (((m_iso7816_hal.pps_rsp[1] ^ m_iso7816_hal.pps_req[1]) & 0x0F) != 0)
            {
                m_iso7816_hal.pps_state = ISO7816_HAL_PPS_FSM_FAIL;
                break;
            }

            /* PPS1 */
            if ((m_iso7816_hal.pps_rsp[1] & 0x10) != 0)     // PPS1 exist?
            {
                if (m_iso7816_hal.pps_rsp[2] != m_iso7816_hal.pps_req[2])
                {
                    m_iso7816_hal.pps_state = ISO7816_HAL_PPS_FSM_FAIL;
                    break;
                }
            }
            else
            {
                // Use default: Fd and Dd
                m_iso7816_hal.Fi = ISO7816_PHY_Fd;
                m_iso7816_hal.Di = ISO7816_PHY_Dd;
            }

            /* PPS2 */
            if ((m_iso7816_hal.pps_rsp[1] & 0x20) != 0)     // PPS2 exist?
            {
                m_iso7816_hal.pps_state = ISO7816_HAL_PPS_FSM_FAIL;
                break;
            }
            
            /* PPS3 */
            if ((m_iso7816_hal.pps_rsp[1] & 0x40) != 0)     // PPS3 exist?
            {
                m_iso7816_hal.pps_state = ISO7816_HAL_PPS_FSM_FAIL;
                break;
            }

            m_iso7816_hal.pps_state = ISO7816_HAL_PPS_FSM_RDY;
            break;

        default:
            break;
    }
}

static void try_recovery (iso7816_hal_evt_id_t evt_when_failed)
{
    uint32_t err_code = NRF_SUCCESS;
    iso7816_hal_evt_t l_evt;

    m_iso7816_hal.sys_state = ISO7816_HAL_SYS_FSM_OFF;
    m_iso7816_hal.atr_state = ISO7816_HAL_ATR_FSM_INIT;
    m_iso7816_hal.pps_state = ISO7816_HAL_PPS_FSM_WAITING;

    err_code = iso7816_phy_warm_reset ();

    if (err_code != NRF_SUCCESS)
    {
        /* Cannot recover. */
        l_evt.evt_id = ISO7816_HAL_EVT_NOT_INITIALIZED;
        m_iso7816_hal.evt_cb(&l_evt);
    }
}


/**@brief Finite state machine (system) */
static void sys_fsm (iso7816_hal_sys_fsm_evt_t evt, uint8_t *data, uint16_t len)
{
    uint32_t err_code = NRF_SUCCESS;
    iso7816_hal_evt_t l_evt;
    uint16_t i;

    switch (m_iso7816_hal.sys_state)
    {
        case ISO7816_HAL_SYS_FSM_INIT:      // Initialization, receiving ATR
            switch (evt)
            {
                case ISO7816_HAL_SYS_FSM_EVT_RX:
                    atr_parser (*data);

                    if (m_iso7816_hal.atr_state == ISO7816_HAL_ATR_FSM_RDY)
                    {
                        m_iso7816_hal.atr_len = m_iso7816_hal.rx_byte_cnt;

                        /* The ATR has been successfully received. */
											
												if(m_iso7816_hal.enable_pps == true)
												{
													m_iso7816_hal.sys_state = ISO7816_HAL_SYS_FSM_ATR_RECVD;
													set_tx();
													err_code = iso7816_phy_send (m_iso7816_hal.pps_req, 4);
													VERIFY_SUCCESS_VOID(err_code);
												
												}else{
													/* No PPS negociation needs, HAL initialization ready*/
													m_iso7816_hal.sys_state  = ISO7816_HAL_SYS_FSM_RDY;

													/* Apply Fi and Di */
													iso7816_phy_set_Fi_Di (m_iso7816_hal.Fi, m_iso7816_hal.Di);
													m_iso7816_hal.wt_enabled = false;

													err_code = tpdu_fsm (ISO7816_HAL_TPDU_FSM_EVT_SYS_RDY, 0, 0);
													if (err_code != NRF_SUCCESS)
													{
															try_recovery (ISO7816_HAL_EVT_FAULT);
															break;
													}

													l_evt.evt_id = ISO7816_HAL_EVT_RDY;
													m_iso7816_hal.evt_cb(&l_evt);
												}
                        m_iso7816_hal.rx_byte_cnt = 0;
                    }
                    break;

                case ISO7816_HAL_SYS_FSM_EVT_TIMEOUT:
                    /* Timeout, during or before the ATR receive procedure */
                    err_code = iso7816_phy_deactivate ();
                    VERIFY_SUCCESS_VOID(err_code);

                    l_evt.evt_id = ISO7816_HAL_EVT_NOT_INITIALIZED;
                    m_iso7816_hal.evt_cb(&l_evt);
                    break;

                default:
                    break;
            }
            break;

        case ISO7816_HAL_SYS_FSM_ATR_RECVD:
            switch (evt)
            {
                case ISO7816_HAL_SYS_FSM_EVT_RX:
                    for (i = 0; i < len; i++)
                    {
                        pps_parser (data[i]);
                    }

                    if (m_iso7816_hal.pps_state == ISO7816_HAL_PPS_FSM_RDY)
                    {
                        /* The correct PPS response has been received. */
                        m_iso7816_hal.sys_state  = ISO7816_HAL_SYS_FSM_RDY;

                        /* Apply Fi and Di */
                        iso7816_phy_set_Fi_Di (m_iso7816_hal.Fi, m_iso7816_hal.Di);
                        m_iso7816_hal.wt_enabled = false;

                        err_code = tpdu_fsm (ISO7816_HAL_TPDU_FSM_EVT_SYS_RDY, 0, 0);
                        if (err_code != NRF_SUCCESS)
                        {
                            try_recovery (ISO7816_HAL_EVT_FAULT);
                            break;
                        }

                        l_evt.evt_id = ISO7816_HAL_EVT_RDY;
                        m_iso7816_hal.evt_cb(&l_evt);
                    }
                    else if (m_iso7816_hal.pps_state == ISO7816_HAL_PPS_FSM_FAIL)
                    {
                        /* PPS procedure failed, trying to recover by warm resetting... */
                        try_recovery (ISO7816_HAL_EVT_NOT_INITIALIZED);
                    }
                    break;

                case ISO7816_HAL_SYS_FSM_EVT_TX:
                    set_rx(m_iso7816_hal.pps_rsp, ISO7816_HAL_PPS_EXPECTED_LEN);
                    break;

                case ISO7816_HAL_SYS_FSM_EVT_TIMEOUT:
                    /* Timeout: recover by default parameters. */
                    m_iso7816_hal.ignore_atr = true;

                    try_recovery (ISO7816_HAL_EVT_NOT_INITIALIZED);
                    break;

                default:
                    break;
            }
            break;

        case ISO7816_HAL_SYS_FSM_RDY:
            switch (evt)
            {
                case ISO7816_HAL_SYS_FSM_EVT_RX:
                    err_code = tpdu_fsm (ISO7816_HAL_TPDU_FSM_EVT_RX_DATA, data, len);
                    if (err_code != NRF_SUCCESS)
                    {
                        try_recovery (ISO7816_HAL_EVT_FAULT);
                    }
                    break;

                case ISO7816_HAL_SYS_FSM_EVT_TX:
                    err_code = tpdu_fsm (ISO7816_HAL_TPDU_FSM_EVT_TX_RDY, 0, len);
                    if (err_code != NRF_SUCCESS)
                    {
                        try_recovery (ISO7816_HAL_EVT_FAULT);
                    }
                    break;

                case ISO7816_HAL_SYS_FSM_EVT_TIMEOUT:
                    if (m_iso7816_hal.wt_enabled)
                    {
                        m_iso7816_hal.ignore_atr = true;
                        try_recovery (ISO7816_HAL_EVT_FAULT);
                    }
                    err_code = tpdu_fsm (ISO7816_HAL_TPDU_FSM_EVT_TIMEOUT, 0, 0);
                    if (err_code != NRF_SUCCESS)
                    {
                        try_recovery (ISO7816_HAL_EVT_FAULT);
                    }
                    break;

                case ISO7816_HAL_SYS_FSM_EVT_SUSPEND:
                    m_iso7816_hal.sys_state = ISO7816_HAL_SYS_FSM_SUSPENDED;
                    break;

                default:
                    break;
            }
            break;

        case ISO7816_HAL_SYS_FSM_SUSPENDED:
            if (evt == ISO7816_HAL_SYS_FSM_EVT_RESUME)
            {
                m_iso7816_hal.phy_suspended = false;
                m_iso7816_hal.sys_state     = ISO7816_HAL_SYS_FSM_RDY;

                /* TPDU resend. */
                if (m_iso7816_hal.tpdu_pended)
                {
                    err_code = iso7816_phy_send (m_iso7816_hal.tpdu_hdr, ISO7816_HAL_TPDU_HDR_LEN);
                    VERIFY_SUCCESS_VOID (err_code);

                    m_iso7816_hal.tpdu_pended = false;
                }
            }
            break;
            
        default:
            m_iso7816_hal.sys_state = ISO7816_HAL_SYS_FSM_OFF;
            break;
    }
}


/**@brief TPDU exchange state machine. */
static uint32_t tpdu_fsm (iso7816_hal_tpdu_fsm_evt_t evt, void * data, uint16_t len)
{
    uint32_t          err_code = NRF_SUCCESS;
    iso7816_hal_evt_t l_evt;
    uint16_t          i;

    /* Check for timeout. */
    if ((m_iso7816_hal.tpdu_state != ISO7816_HAL_TPDU_FSM_DISABLED) &&
        (m_iso7816_hal.tpdu_state != ISO7816_HAL_TPDU_FSM_INIT) &&
        (m_iso7816_hal.tpdu_state != ISO7816_HAL_TPDU_FSM_IDLE) &&
        (evt == ISO7816_HAL_TPDU_FSM_EVT_TIMEOUT))
    {
        /* Reset state */
        m_iso7816_hal.tpdu_state = ISO7816_HAL_TPDU_FSM_IDLE;

        l_evt.evt_id = ISO7816_HAL_EVT_TPDU_TIMEOUT;
        m_iso7816_hal.evt_cb(&l_evt);
        return err_code;
    }

    switch (m_iso7816_hal.tpdu_state)
    {
        case ISO7816_HAL_TPDU_FSM_INIT:
            /* Right after PHY initialization */
            if (evt == ISO7816_HAL_TPDU_FSM_EVT_SYS_RDY)
            {
                m_iso7816_hal.tpdu_state = ISO7816_HAL_TPDU_FSM_IDLE;
                disable_txrx ();
            }
            break;

        case ISO7816_HAL_TPDU_FSM_IDLE:
            /* TPDU exchange request */
            if (evt == ISO7816_HAL_TPDU_FSM_EVT_REQ)
            {
                volatile iso7816_hal_tpdu_t *tpdu;

                /* Parameter checks */
                if (
                        ((m_iso7816_hal.tpdu.req != ISO7816_HAL_TPDU_REQ_GET) && (m_iso7816_hal.tpdu.req != ISO7816_HAL_TPDU_REQ_PUT))  // Neither GET nor PUT
                     || ((m_iso7816_hal.tpdu.req == ISO7816_HAL_TPDU_REQ_PUT) && (m_iso7816_hal.tpdu.len >= 256))                       // PUT but len >= 256
                   )
                {
                    err_code = NRF_ERROR_INVALID_PARAM;
                    break;
                }

                tpdu = &m_iso7816_hal.tpdu;
                memcpy ((void *)tpdu, data, sizeof(iso7816_hal_tpdu_t));
                tpdu->reserved           = tpdu->ins ^ 0xFF;    // for procedure byte checking
                m_iso7816_hal.wt_enabled = false;

                /* Build TPDU header */
                m_iso7816_hal.tpdu_hdr [0] = tpdu->cla;
                m_iso7816_hal.tpdu_hdr [1] = tpdu->ins;
                m_iso7816_hal.tpdu_hdr [2] = tpdu->p1;
                m_iso7816_hal.tpdu_hdr [3] = tpdu->p2;
                m_iso7816_hal.tpdu_hdr [4] = tpdu->len & 0x00FF;

                /* Send header */
                if (!m_iso7816_hal.phy_suspended)   // Is PHY suspended ?
                {
                    set_tx();

                    err_code = iso7816_phy_send (m_iso7816_hal.tpdu_hdr, ISO7816_HAL_TPDU_HDR_LEN);
                    VERIFY_SUCCESS (err_code);
                }
                else
                {
                    m_iso7816_hal.tpdu_pended = true;
                }

                m_iso7816_hal.tpdu_state = ISO7816_HAL_TPDU_FSM_HEADER;
            }
            break;

        case ISO7816_HAL_TPDU_FSM_HEADER:
            if (evt == ISO7816_HAL_TPDU_FSM_EVT_TX_RDY)
            {
                m_iso7816_hal.tpdu_state = ISO7816_HAL_TPDU_FSM_LISTEN;
                set_rx (&m_iso7816_hal.rx_buff, 1);
            }
            break;

        case ISO7816_HAL_TPDU_FSM_LISTEN:
            if (evt == ISO7816_HAL_TPDU_FSM_EVT_RX_DATA)
            {
                /* Assumption: only one procedure byte has been received */
                uint8_t procd = *(uint8_t *)data;

                if (procd == ISO7816_HAL_TPDU_PROCD_NULL)
                {
                    /* NULL byte received */
                    /* No action required, and wait for another procedure byte. */
                    break;
                }
                else if (
                            ((procd & ISO7816_HAL_TPDU_PROCD_SW1_MASK) == ISO7816_HAL_TPDU_PROCD_SW1_OK)
                         || ((procd & ISO7816_HAL_TPDU_PROCD_SW1_MASK) == ISO7816_HAL_TPDU_PROCD_SW1_ERR)
                        )
                {
                    /* SW1 received, waiting for SW2 */
                    m_iso7816_hal.tpdu_rsp.sw1 = procd;
                    m_iso7816_hal.tpdu_state   = ISO7816_HAL_TPDU_FSM_SW1;
                }
                else if (procd == m_iso7816_hal.tpdu.ins)
                {
                    /* ACK (INS) received, send/receive all data */
                    switch (m_iso7816_hal.tpdu.req)
                    {
                        case ISO7816_HAL_TPDU_REQ_GET:
                            m_iso7816_hal.tpdu_state = ISO7816_HAL_TPDU_FSM_RX_DATA;
                            set_rx (m_iso7816_hal.tpdu.data, m_iso7816_hal.tpdu.len);
                            break;

                        case ISO7816_HAL_TPDU_REQ_PUT:
                            set_tx ();

                            err_code = iso7816_phy_send (m_iso7816_hal.tpdu.data, m_iso7816_hal.tpdu.len);
                            VERIFY_SUCCESS (err_code);

                            m_iso7816_hal.tpdu_state = ISO7816_HAL_TPDU_FSM_TX_DATA;
                            m_iso7816_hal.tpdu.data += m_iso7816_hal.tpdu.len;
                            m_iso7816_hal.tpdu.len   = 0;
                            break;

                        default:
                            err_code = NRF_ERROR_INTERNAL;
                            break;
                    }
                }
                else if (procd == m_iso7816_hal.tpdu.reserved)
                {
                    /* ACK (INS^0xFF) received, send/receive one byte. */
                    switch (m_iso7816_hal.tpdu.req)
                    {
                        case ISO7816_HAL_TPDU_REQ_GET:
                            m_iso7816_hal.wt_enabled = true;
                            m_iso7816_hal.tpdu_state = ISO7816_HAL_TPDU_FSM_RX_1BYTE;
                            break;

                        case ISO7816_HAL_TPDU_REQ_PUT:
                            set_tx ();

                            m_iso7816_hal.wt_enabled = false;
                            err_code = iso7816_phy_send (m_iso7816_hal.tpdu.data, 1);
                            VERIFY_SUCCESS (err_code);

                            m_iso7816_hal.tpdu_state = ISO7816_HAL_TPDU_FSM_TX_DATA;
                            m_iso7816_hal.tpdu.data++;
                            m_iso7816_hal.tpdu.len--;
                            break;

                        default:
                            err_code = NRF_ERROR_INTERNAL;
                            break;
                    }
                }
                else
                {
                    err_code = NRF_ERROR_INTERNAL;
                }
            }
            break;

        case ISO7816_HAL_TPDU_FSM_TX_DATA:
            if (evt == ISO7816_HAL_TPDU_FSM_EVT_TX_RDY)
            {
                m_iso7816_hal.wt_enabled = true;
                m_iso7816_hal.tpdu_state = ISO7816_HAL_TPDU_FSM_LISTEN;

                set_rx (&m_iso7816_hal.rx_buff, 1);
            }
            break;

        case ISO7816_HAL_TPDU_FSM_RX_DATA:
            if (evt == ISO7816_HAL_TPDU_FSM_EVT_RX_DATA)
            {
                /* Back-up buffer. */
                for (i = 0; i < len; i++)
                {
                    *m_iso7816_hal.tpdu.data++ = ((uint8_t *)data)[i];
                }

                m_iso7816_hal.tpdu.len -= len;

                if (m_iso7816_hal.tpdu.len == 0)
                {
                    m_iso7816_hal.tpdu_state = ISO7816_HAL_TPDU_FSM_LISTEN;
                }
            }
            break;

        case ISO7816_HAL_TPDU_FSM_RX_1BYTE:
            if (evt == ISO7816_HAL_TPDU_FSM_EVT_RX_DATA)
            {
                m_iso7816_hal.tpdu_state   = ISO7816_HAL_TPDU_FSM_LISTEN;
                *m_iso7816_hal.tpdu.data++ = *(uint8_t *)data;
                m_iso7816_hal.tpdu.len--;
            }
            break;

        case ISO7816_HAL_TPDU_FSM_SW1:
            if (evt == ISO7816_HAL_TPDU_FSM_EVT_RX_DATA)
            {
                m_iso7816_hal.wt_enabled   = false;
                m_iso7816_hal.tpdu_rsp.sw2 = *(uint8_t *)data;
                m_iso7816_hal.tpdu_state   = ISO7816_HAL_TPDU_FSM_IDLE;

                disable_txrx ();

                /* Notify application */
                if (m_iso7816_hal.evt_cb != 0)
                {
                    l_evt.evt_id           = ISO7816_HAL_EVT_TPDU_RESPONSE;
                    l_evt.evt.tpdu_rsp.sw1 = m_iso7816_hal.tpdu_rsp.sw1;
                    l_evt.evt.tpdu_rsp.sw2 = m_iso7816_hal.tpdu_rsp.sw2;
                    m_iso7816_hal.evt_cb(&l_evt);
                }
            }
            break;

        default:
            break;
    }

    return err_code;
}

uint32_t iso7816_hal_init (iso7816_hal_init_t *init_param)
{
    VERIFY_PARAM_NOT_NULL(init_param);
    VERIFY_PARAM_NOT_NULL(init_param->evt_cb);
    uint32_t err_code = NRF_SUCCESS;
    iso7816_phy_init_t phy_init_param;

    //
    // 1. PHY
    //
    memset (&phy_init_param, 0, sizeof(iso7816_phy_init_t));
    phy_init_param.clock_freq  = (iso7816_phy_freq_t) init_param->clock_freq;
    phy_init_param.vcc_pin     = init_param->vcc_pin;
    phy_init_param.clk_pin     = init_param->clk_pin;
    phy_init_param.rst_pin     = init_param->rst_pin;
    phy_init_param.io_pin      = init_param->io_pin;
    phy_init_param.dbg_act_pin = init_param->dbg_act_pin;
    phy_init_param.dbg_ppi_pin = init_param->dbg_ppi_pin;
    phy_init_param.evt_cb      = on_iso7816_phy_evt;

    err_code = iso7816_phy_init (&phy_init_param);
    VERIFY_SUCCESS(err_code);

    err_code = iso7816_phy_cold_reset ();
    VERIFY_SUCCESS(err_code);

    //
    // 2. HAL
    //
    memset (&m_iso7816_hal, 0, sizeof(iso7816_hal_t));
    m_iso7816_hal.evt_cb    = init_param->evt_cb;
    m_iso7816_hal.sys_state = ISO7816_HAL_SYS_FSM_OFF;
    m_iso7816_hal.clk_freq  = init_param->clock_freq;

    /* ATR */
    m_iso7816_hal.atr_state  = ISO7816_HAL_ATR_FSM_INIT;
    m_iso7816_hal.atr[0]     = ISO7816_HAL_INVALID_BYTE;
    m_iso7816_hal.T          = 0x0001;
    m_iso7816_hal.last_T     = 0xFF;
    m_iso7816_hal.wt_enabled = true;

    m_iso7816_hal.ignore_atr = false;
		
		m_iso7816_hal.Di				 = ISO7816_PHY_Dd;
		m_iso7816_hal.Fi				 = ISO7816_PHY_Fd;
		

    /* PPS */
		m_iso7816_hal.enable_pps = init_param->enable_pps;
    m_iso7816_hal.pps_state = ISO7816_HAL_PPS_FSM_WAITING;

    memcpy (m_iso7816_hal.pps_req, m_pps_def_req, sizeof(m_pps_def_req));
    m_iso7816_hal.pps_req[3] = m_iso7816_hal.pps_req[0] ^ m_iso7816_hal.pps_req[1] ^ m_iso7816_hal.pps_req[2];  // xsum

    /* TPDU exchange */
    m_iso7816_hal.tpdu_state = ISO7816_HAL_TPDU_FSM_INIT;

    m_iso7816_hal.sys_state = ISO7816_HAL_SYS_FSM_INIT;
    return err_code;
}


uint32_t iso7816_hal_send_tpdu (iso7816_hal_tpdu_t *tpdu)
{
    uint32_t err_code = NRF_SUCCESS;

    if (m_iso7816_hal.tpdu_state != ISO7816_HAL_TPDU_FSM_IDLE)
    {
        err_code = NRF_ERROR_BUSY;
    }
    else
    {
        err_code = tpdu_fsm (ISO7816_HAL_TPDU_FSM_EVT_REQ, tpdu, sizeof(iso7816_hal_tpdu_t));
    }
    return err_code;
}


uint32_t iso7816_hal_suspend (void)
{
    uint32_t err_code = NRF_SUCCESS;
    volatile iso7816_hal_sys_fsm_t *sys_state;

    if (m_iso7816_hal.clk_stop_supported)
    {
        err_code = iso7816_phy_clk_stop (m_iso7816_hal.clk_stop_lv);
        VERIFY_SUCCESS(err_code);

        m_iso7816_hal.phy_suspended = true;

        /* Wait until PHY is suspended. */
        sys_state = &m_iso7816_hal.sys_state;
        while (*sys_state != ISO7816_HAL_SYS_FSM_SUSPENDED) ;
    }

    return err_code;
}


uint32_t iso7816_hal_resume (void)
{
    uint32_t err_code = NRF_SUCCESS;
    volatile iso7816_hal_sys_fsm_t *sys_state;

    if (m_iso7816_hal.clk_stop_supported)
    {
        err_code = iso7816_phy_clk_resume ();
        VERIFY_SUCCESS(err_code);

        /* Wait until PHY is resumed. */
        sys_state = &m_iso7816_hal.sys_state;
        while (*sys_state != ISO7816_HAL_SYS_FSM_RDY) ;
    }

    return err_code;
}

uint32_t iso7816_hal_uninit (void)
{
    uint32_t err_code = NRF_SUCCESS;

    err_code = iso7816_phy_uninit ();
    VERIFY_SUCCESS(err_code);

    m_iso7816_hal.sys_state = ISO7816_HAL_SYS_FSM_OFF;
    return err_code;
}


void iso7816_hal_log_tpdu_req (iso7816_hal_tpdu_t *tpdu)
{
    uint32_t i;

    /* Label */
    NRF_LOG("Tx: ");

    /* Header */
    NRF_LOG("[");
    NRF_LOG_HEX_CHAR (tpdu->cla); NRF_LOG(" ");
    NRF_LOG_HEX_CHAR (tpdu->ins); NRF_LOG(" ");
    NRF_LOG_HEX_CHAR (tpdu->p1 ); NRF_LOG(" ");
    NRF_LOG_HEX_CHAR (tpdu->p2 ); NRF_LOG(" ");
    NRF_LOG_HEX_CHAR (tpdu->len);
    NRF_LOG("] ");

    /* Type */
    switch (tpdu->req)
    {
        case ISO7816_HAL_TPDU_REQ_GET:
            NRF_LOG("GET\r\n");
            break;

        case ISO7816_HAL_TPDU_REQ_PUT:
            NRF_LOG("PUT [");
            for (i = 0; i < tpdu->len - 1; i++)
            {
                NRF_LOG_HEX_CHAR (tpdu->data[i]); NRF_LOG(" ");
            }
            NRF_LOG_HEX_CHAR (tpdu->data[i]); NRF_LOG("]\r\n");
            break;

        default:
            NRF_LOG("UNKNOWN\r\n");
            break;
    }
}


void iso7816_hal_log_tpdu_rsp (iso7816_hal_tpdu_t *tpdu, uint8_t sw1, uint8_t sw2)
{
    uint32_t i;

    /* Label */
    NRF_LOG("Rx: ");

    /* Type */
    if (tpdu->req == ISO7816_HAL_TPDU_REQ_GET)
    {
        NRF_LOG("[");
        for (i = 0; i < tpdu->len - 1; i++)
        {
            NRF_LOG_HEX_CHAR (tpdu->data[i]); NRF_LOG(" ");
        }
        NRF_LOG_HEX_CHAR (tpdu->data[i]); NRF_LOG("] ");
    }

    ;
    /* Trailer */
    NRF_LOG("[");
    NRF_LOG_HEX_CHAR (sw1); NRF_LOG(" ");
    NRF_LOG_HEX_CHAR (sw2);
    NRF_LOG("]\r\n");
}


void iso7816_hal_get_atr (uint8_t **data, uint16_t *len)
{
    *data = m_iso7816_hal.atr;
    *len  = m_iso7816_hal.atr_len;
}

/** @} */ // End tag for iso7816_hal

