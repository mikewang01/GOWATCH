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

#include "iso7816_2.h"
#include "nrf_gpio.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include <string.h>
#include "main.h"
#define  NRF_LOG_PRINTF Y_SPRINTF

/**
 * @defgroup iso7816_private Private
 * @ingroup  nrf_iso7816
 * @{
 */

/**
 * @defgroup iso7816_private_const Constants
 * @{
 */

#define ISO7816_DBG_ENABLE          0               /**< Enable debug signal (for timing verfication). */
#define ISO7816_DBG_PPI_ENABLE      0               /**< Enable debug signal (PPI)(for timing verfication). */

#define ISO7816_INVALID_BYTE        0xFF            /**< Invalid byte. */

#define ISO7816_ATR_INVERSE         0x03             /**< First byte of ATR, indicates inverse coding. */
#define ISO7816_ATR_DIRECT          0x3B            /**< First byte of ATR, indicates direct coding. */
#define ISO7816_ATR_MAX_LEN         32              /**< Maximum size of ATR. */
#define ISO7816_Fd                  372             /**< Default F. */
#define ISO7816_Dd                  1               /**< Default D. */
#define ISO7816_WT                  9600            /**< Default WT. */
#define ISO7816_ATR_MAX_WAIT        40000           /**< Maximum waiting time for the first character after ATR */

#define ISO7816_PPS_MAX_LEN         6               /**< Maximum size of PPS request/response message. */
#define ISO7816_PPSS                0xFF            /**< First byte of PPS messages. */
#define ISO7816_PPS_EXPECTED_LEN    4

#define ISO7816_TPDU_HDR_LEN        5               /**< TPDU header length. */
#define ISO7816_TPDU_PROCD_NULL     0x60            /**< TPDU procedure byte: NULL. */
#define ISO7816_TPDU_PROCD_SW1_MASK 0xF0            /**< TPDU procedure byte: mask to check SW1. */
#define ISO7816_TPDU_PROCD_SW1_OK   0x90            /**< TPDU procedure byte: first half of SW1 - OK. */
#define ISO7816_TPDU_PROCD_SW1_ERR  0x60            /**< TPDU procedure byte: first half of SW1 - ERROR. */

#define ISO7816_PERIOD_COMPENSATION 0               /**< Period compensation, for NRF_TIMER */
#define ISO7816_EDGE_COMPENSATION   60              /**< Falling edge timing compensation */
#define ISO7816_EDGE_COMPENSATION_1 60               /**< Falling edge timing compensation */
#define ISO7816_MAX_WAIT            32767           /**< maximum waiting time for function wait() */

#define ISO7816_TIMER_CH_ID_1P5     1               /**< Timer channel ID: 1.5 ETU timer */
#define ISO7816_TIMER_CH_ID_1       2               /**< Timer channel ID: 1 ETU timer */

#define ISO7816_GPIOTE_HIGH_SPD     false

/** @} */ // End tag for iso7816_private_const


/**
 * @defgroup iso7816_private_macro Macros
 * @{
 */

#if ISO7816_DBG_ENABLE
/**@brief Toggle debug pin: m_dbg_activity_pin */
#define DBG_SIGNAL() \
{\
	if (m_dbg_activity_pin != ISO7816_INVALID_PIN_NUMBER)\
	{\
		nrf_gpio_pin_toggle(m_dbg_activity_pin);\
	}\
}
#else
#define DBG_SIGNAL()
#endif

/** @} */ // End tag for iso7816_private_macro


/**
 * @defgroup iso7816_private_enums Enumerations
 * @{
 */

/**@brief Finite state machine (system). */
typedef enum {
    ISO7816_SYS_FSM_OFF = 0,            /**< ISO7816 module is turned off */
    ISO7816_SYS_FSM_INIT,               /**< Initialization in progress, waiting for ATR */
    ISO7816_SYS_FSM_ATR_RECVD,          /**< ATR received, parse and exchange PPS whenever necessary */
    ISO7816_SYS_FSM_RDY                 /**< System ready for Tx/Rx */
} iso7816_sys_fsm_t;


/**@brief Events for system finite state machine. */
typedef enum {
    ISO7816_SYS_FSM_EVT_INIT = 0,       /**< Initialization (constructor). */
    ISO7816_SYS_FSM_EVT_RX,             /**< Indicates that a byte is received. */
    ISO7816_SYS_FSM_EVT_TX,             /**< Indicates that data is transmitted. */
    ISO7816_SYS_FSM_EVT_TIMEOUT,        /**< System FSM timer timeout */
    ISO7816_SYS_FSM_EVT_CLEANUP         /**< Clean-up (destructor). */
} iso7816_sys_fsm_evt_t;


/**@brief Finite state machine (I/O). */
typedef enum {
    ISO7816_IO_FSM_DISABLED = 0,        /**< I/O line disabled */
    ISO7816_IO_FSM_RX_DATA,             /**< DATA bits reception in progress */
    ISO7816_IO_FSM_RX_PARITY,           /**< Receiving parity bits */
    ISO7816_IO_FSM_RX_PAUSE,            /**< Sending PAUSE bit (ACK bit) */
    ISO7816_IO_FSM_RX_ERROR,            /**< Sending ERROR bit (NAK bit) */
    ISO7816_IO_FSM_RX_WAIT,             /**< Waiting for end of PAUSE/ERROR bit */
    ISO7816_IO_FSM_TX_DATA,             /**< Sending DATA bits */
    ISO7816_IO_FSM_TX_PARITY,           /**< Sending PARITY bit */
    ISO7816_IO_FSM_TX_WAIT_0,           /**< Waiting for PAUSE/ERROR bit #1 (ACK/NAK) */
    ISO7816_IO_FSM_TX_WAIT_1,           /**< Waiting for PAUSE/ERROR bit #2 (ACK/NAK) */
    ISO7816_IO_FSM_TX_START,            /**< Start bit, used in resend or continue-send */
    ISO7816_IO_FSM_IDLE                 /**< I/O line idle, ready for Tx / Rx */
} iso7816_io_fsm_t;


/**@brief Events for I/O finite state machine. */
typedef enum {
    ISO7816_IO_FSM_EVT_ACTIVATE = 0,    /**< Activate, after system initialization */
    ISO7816_IO_FSM_EVT_RX_START_BIT,    /**< On receive of a START bit. */
    ISO7816_IO_FSM_EVT_TX_REQ,          /**< Higher layer requested the transmission of a byte. */
    ISO7816_IO_FSM_EVT_BIT_TIMER,       /**< Tx/Rx bit timer expired. */
    ISO7816_IO_FSM_EVT_DEACTIVATE       /**< Deactivate, before system clean-up. */
} iso7816_io_fsm_evt_t;


/**@brief Finite state machine (ATR). */
typedef enum {
    ISO7816_ATR_FSM_INIT = 0,           /**< Initial: Expects Format byte T0 */
    ISO7816_ATR_FSM_FORMAT,             /**< Expects Format byte T0 */
    ISO7816_ATR_FSM_GLOBAL_1,           /**< Expects global values in TA1, TC1 and TD1 */
    ISO7816_ATR_FSM_GLOBAL_2,           /**< Expects global values in TA2, TC2 and TD2 */
    ISO7816_ATR_FSM_STRUCTURAL,         /**< Expects structural values in TAi, TCi and TDi */
    ISO7816_ATR_FSM_HISTORICAL,         /**< Expects historical bytes */
    ISO7816_ATR_FSM_TCK,                /**< Expects TCK */
    ISO7816_ATR_FSM_RDY,                /**< ATR Ready */
    ISO7816_ATR_FSM_FAIL                /**< ATR failed */
} iso7816_atr_fsm_t;


/**@brief Finite state machine (PPS). */
typedef enum {
    ISO7816_PPS_FSM_WAITING = 0,        /**< Receiving PPSS */
    ISO7816_PPS_FSM_PPS0,               /**< Receiving PPS0 */
    ISO7816_PPS_FSM_PPSX,               /**< Receiving PPS1~PPS3 */
    ISO7816_PPS_FSM_PCK,                /**< Receiving PCK */
    ISO7816_PPS_FSM_RDY,                /**< PPS ready */
    ISO7816_PPS_FSM_FAIL                /**< PPS failed */
} iso7816_pps_fsm_t;


/**@brief Finite state machine (TPDU exchange). */
typedef enum {
    ISO7816_TPDU_FSM_INIT = 0,          /**< Initialization in progress. */
    ISO7816_TPDU_FSM_IDLE,              /**< Idle state, can receive request from application. */
    ISO7816_TPDU_FSM_HEADER,            /**< Sending header. */
    ISO7816_TPDU_FSM_LISTEN,            /**< Listening to procedure bytes. */
    ISO7816_TPDU_FSM_TX_DATA,           /**< Sending data (1 or multiple bytes) to card. */
    ISO7816_TPDU_FSM_RX_DATA,           /**< Receiving data from card. */
    ISO7816_TPDU_FSM_RX_1BYTE,          /**< Receiving 1-byte from card. */
    ISO7816_TPDU_FSM_SW1                /**< Received SW1, waiting for SW2. */
} iso7816_tpdu_fsm_t;


/**@brief Events for TPDU exchange finite state machine. */
typedef enum {
    ISO7816_TPDU_FSM_EVT_REQ,           /**< Request from application. */
    ISO7816_TPDU_FSM_EVT_TX_RDY,        /**< Data transmitted. */
    ISO7816_TPDU_FSM_EVT_RX_DATA,       /**< On receive of a byte, be it procedure byte or data. */
    ISO7816_TPDU_FSM_EVT_SYS_RDY        /**< ISO7816 module initialized. */
} iso7816_tpdu_fsm_evt_t;

/** @} */ // End tag for iso7816_private_enums

/**
 * @defgroup iso7816_private_templates Callback templates
 * @{
 */

/**@brief Timer callback function
 * @note  Return true to stop timer.
 */
typedef void (* timer_handler_t)(void);

/** @} */ // End tag for iso7816_private_templates


/**
 * @defgroup iso7816_private_var Variables
 * @{
 */

static iso7816_sys_fsm_t      m_sys_state = ISO7816_SYS_FSM_OFF;        /**< System finite state machine */
static iso7816_io_fsm_t       m_io_state  = ISO7816_IO_FSM_DISABLED;    /**< I/O finite state machine */
static iso7816_io_fsm_t       m_io_state_prev __attribute__((unused)) = ISO7816_IO_FSM_DISABLED;/**< I/O finite state machine */

static uint8_t                m_vcc_pin;                                /**< Pin number of VCC */
static uint8_t                m_clk_pin;                                /**< Pin number of CLK */
static uint8_t                m_rst_pin;                                /**< Pin number of RST */
static uint8_t                m_io_pin;                                 /**< Pin number of I/O */
static uint8_t                m_dbg_activity_pin __attribute__((unused));                       /**< Pin number of "Activity Debug" */
static uint8_t                m_dbg_ppi_pin __attribute__((unused));                            /**< Pin number of "PPI Debug" */

static uint8_t                m_io_in_val;                              /**< Value of I/O, read right after bit timer */
static uint8_t                m_io_out_val;                             /**< Value of I/O, write right after bit timer */

static iso7816_evt_handler_t  m_evt_cb;                                 /**< Event callback */

static iso7816_freq_t         m_clk_freq;                               /**< Clock frequency backup */
static uint16_t               m_1_etu;                                  /**< ETU, in number of high frequency clocks */
static uint16_t               m_0p5_etu;                                /**< 0.5 ETU, in number of high frequency clocks */
static uint8_t                m_bit_cnt __attribute__((unused));                                /**< Bit counter for Tx/Rx */
static uint8_t              * m_tx_data __attribute__((unused));                                /**< Pointer to Tx data */
static uint8_t                m_rx_data;                                /**< Received byte */
static uint8_t                m_parity __attribute__((unused));                                 /**< Locally computed parity */
static uint8_t                m_inv = 0;                                /**< Indicates inverse decoding */
static uint8_t                m_tx_byte_cnt __attribute__((unused));                            /**< Tx byte counter */
static uint8_t                m_rx_byte_cnt;                            /**< Rx byte counter */
static uint8_t                m_rx_wait_bit;                            /**< Number of Rx wait bit */

static uint8_t                m_atr [ISO7816_ATR_MAX_LEN];              /**< ATR */
static iso7816_atr_fsm_t      m_atr_state = ISO7816_ATR_FSM_INIT;       /**< ATR finite state machine */
static uint16_t               m_T;                                      /**< Last known T */
static uint8_t                m_Y;                                      /**< Last known Yi */
static uint8_t                m_K;                                      /**< Number of Historical bytes. */
static uint16_t               m_Fi;                                     /**< F proposed by SIM card. */
static uint8_t                m_Di;                                     /**< D proposed by SIM card. */
static uint8_t                m_N;                                      /**< Extra guard time (N) proposed by SIM card. */
static bool                   m_neogatiable __attribute__((unused));                            /**< Ability for the SIM card to enter negotiable mode */
static bool                   m_use_Fi_Di __attribute__((unused));                              /**< Use Fi/Di (true), or default (false). */
static uint16_t               m_WI;                                     /**< Waiting internal: default 10. */
static bool                   m_ignore_atr;                             /**< Ignore ATR parameters: default false */
static bool                   m_wt_enabled;                             /**< Indicates if WT is enabled */
static bool                   m_FD_updated __attribute__((unused));                             /**< Indicates whether F and D are updated */

static iso7816_pps_fsm_t      m_pps_state = ISO7816_PPS_FSM_WAITING;    /**< PPS finite state machine. */
static uint8_t                m_pps_req[ISO7816_PPS_MAX_LEN]
    = {ISO7816_PPSS, 0x10, 0x01, 0x00};                                 /**< PPS request. */
static uint8_t                m_pps_rsp[ISO7816_PPS_MAX_LEN];           /**< PPS response. */
static uint8_t                m_pps_n;                                  /**< Number of PPS1~PPS3 received */

static iso7816_tpdu_fsm_t     m_tpdu_state = ISO7816_TPDU_FSM_INIT;     /**< TPDU exchange finite state machine. */
static iso7816_tpdu_t         m_tpdu;                                   /**< Back up of TPDU request from application. */
static uint8_t                m_tpdu_hdr[ISO7816_TPDU_HDR_LEN];         /**< TPDU header length */
static iso7816_evt_tpdu_rsp_t m_tpdu_rsp;                               /**< TPDU response */


static nrf_drv_timer_t   m_high_freq_timer = NRF_DRV_TIMER_INSTANCE(1); /**< High frequency timer */
static nrf_ppi_channel_t m_high_freq_ppi;                               /**< High frequency PPI */

static nrf_drv_timer_t   m_low_freq_timer  = NRF_DRV_TIMER_INSTANCE(2); /**< Low frequency timer */
static timer_handler_t   m_low_freq_cb;                                 /**< Low frequency callback function */
static nrf_ppi_channel_t m_low_freq_ppi __attribute__((unused));                                /**< Low frequency PPI */
static nrf_ppi_channel_t m_low_freq_ppi_clr __attribute__((unused));                            /**< Low frequency PPI */

static uint16_t          m_last_wait;                                   /**< Last known waiting time */
static uint32_t          m_total_wait;                                  /**< Total wait time */
static int16_t           m_edge_comp;                                   /**< Edge compensation of the first byte */

#if ISO7816_DBG_PPI_ENABLE
static nrf_ppi_channel_t m_io_dbg_ppi_edge;                             /**< Debug PPI */
static nrf_ppi_channel_t m_io_dbg_ppi_1p5;                              /**< Debug PPI */
static nrf_ppi_channel_t m_io_dbg_ppi_1;                                /**< Debug PPI */
static nrf_ppi_channel_t m_io_dbg_ppi_0p5;                              /**< Debug PPI */
#endif

#define  TX_MODE  1
#define  RX_MODE  0
static uint8_t current_mode = RX_MODE;

/** @} */ // End tag for iso7816_private_var


/**
 * @defgroup iso7816_private_lut Look-up tables
 * @{
 */

/**@brief Compare configuration register lookup table
 * @note  Index is of data type @ref iso7816_freq_t
 */
const uint8_t iso7816_cc_lut [ISO7816_FREQ_END]
= {
    8,  // 1MHz
    4,  // 2MHz
    2   // 4MHz
};

/**@brief Timer frequency lookup table
 * @note  For use in initialization and clock reconfiguration only */
const nrf_timer_frequency_t iso7816_timer_freq_lut [ISO7816_FREQ_END]
= {
    NRF_TIMER_FREQ_1MHz,  // 1MHz
    NRF_TIMER_FREQ_2MHz,  // 2MHz
    NRF_TIMER_FREQ_4MHz   // 4MHz
};

/**@brief Lower frequency timer config */
static nrf_drv_timer_config_t m_low_freq_timer_cfg
= {
    NRF_TIMER_FREQ_16MHz,
    NRF_TIMER_MODE_TIMER,
    NRF_TIMER_BIT_WIDTH_16,
    2,
    NULL
};


/**@brief 4-bit inverse-code lookup table */
const uint8_t iso7816_inverse_code_lut [16]
    = {0xF, 0x7, 0xB, 0x3, 0xD, 0x5, 0x9, 0x1,
       0xE, 0x6, 0xA, 0x2, 0xC, 0x4, 0x8, 0x0
      };


/**@brief 4-bit parity lookup table */
const uint8_t iso7816_parity_lut [16]
    = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};


/**@brief Fi lookup table (see ISO7816-3, table 7) */
const uint16_t iso7816_Fi_lut [16]
    = { 372,  372,  558,  744, 1116, 1488, 1860,    0,
        0,  512,  768, 1024, 1536, 2048,    0,    0
      };


/**@brief Di lookup table (see ISO7816-3, table 8) */
const uint16_t iso7816_Di_lut [16]
    = { 0,  1,  2,  4,  8, 16, 32, 64,
        12, 20,  0,  0,  0,  0,  0,  0
      };

/**@brief Maximum of Di index supported by this implementation */
const uint16_t iso7816_MaxDi_idx_lut [3][16]
= {
    {4, 4, 4, 5, 5, 6, 6, 1, 1, 4, 5, 5, 6, 6, 1, 1},
    {3, 3, 3, 4, 4, 5, 5, 1, 1, 3, 4, 4, 5, 5, 1, 1},
    {2, 2, 2, 3, 3, 4, 4, 1, 1, 2, 3, 3, 4, 4, 1, 1}
};


/**@brief 3-bit bit-count lookup table */
const uint8_t iso7816_bcnt_lut [8]
    = {0, 1, 1, 2, 1, 2, 2, 3};

/** @} */ // End tag for iso7816_private_lut

/** @defgroup iso7816_private_func Functions */
/** @} */ // End tag for iso7816_private


/* Private function prototypes */
#if 1
static uint32_t clk_cleanup (void);
static void update_io_state (iso7816_io_fsm_t new_state);
static uint8_t inverse_code (uint8_t data);
static uint8_t parity (uint8_t data);
static void gpiote_event_handler (nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void bit_timer_handler (void);
static void sys_timer_handler (void);
static void dummy_timer_handler (nrf_timer_event_t event_type, void * p_context);
static void timer_handler (nrf_timer_event_t event_type, void * p_context);
static void low_freq_timer_restart (void);
static void low_freq_timer_clear_events (void);
static void low_freq_timer_enable (uint8_t ch);
static void timers_set_etu (uint16_t F, uint8_t D);
static uint32_t io_init (void);
static void io_set_input (void);
static bool io_wait_for_start_bit (void);
static void io_set_output (void);
static uint32_t io_get_value (void);
static void io_set_value (uint8_t val);
static void io_disable (void);
static void gpio_init (void);
static uint32_t dbg_init (void);
static uint32_t timers_init (uint16_t F, uint8_t D);
static uint32_t clk_init (iso7816_freq_t clk_freq);
static void clk_enable (void);
static void clk_disable (void);
static void clear_timer (void);
static void wait (uint16_t n);
static void sleep (uint16_t n, timer_handler_t cb);
static void sleep_and_stop (uint16_t n, timer_handler_t cb);
static uint32_t cold_reset (void);
static void warm_reset_0 (void);
static void warm_reset_1 (void);
static uint32_t on_init (iso7816_init_t *init_param);
static uint32_t iso7816_send (uint8_t *data, uint16_t len);
static uint32_t sys_fsm (iso7816_sys_fsm_evt_t evt, void *data, uint8_t len);
static uint32_t io_fsm (iso7816_io_fsm_evt_t evt, void *data, uint8_t len);
static uint32_t tpdu_fsm (iso7816_tpdu_fsm_evt_t evt, void *data, uint8_t len);
void _config_iso7816_input(void);
void _config_iso7816_output(void);
static int _iso7816_uart_tx(uint8_t ch);
#endif


/**
 * @defgroup iso7816_private_func_others Others
 * @ingroup  iso7816_private_func
 * @{
 */

/**@brief GPIOTE event handler */
static void gpiote_event_handler (nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    DBG_SIGNAL();
    if ((pin == m_io_pin) && (action == NRF_GPIOTE_POLARITY_HITOLO)) {
        low_freq_timer_enable (ISO7816_TIMER_CH_ID_1P5);
        nrf_drv_gpiote_in_event_disable (m_io_pin);

#if ISO7816_DBG_PPI_ENABLE
        if (m_dbg_ppi_pin != ISO7816_INVALID_PIN_NUMBER) {
            NRF_PPI->CHENCLR = PPI_CHENCLR_CH0_Clear << ((uint32_t) m_io_dbg_ppi_1);
        }
#endif

        // io_fsm (ISO7816_IO_FSM_EVT_RX_START_BIT, 0, 0);
    }
    DBG_SIGNAL();
}


/**@brief Configure GPIOs */
static void gpio_init ()
{
    nrf_gpio_cfg_output (m_vcc_pin);
    nrf_gpio_pin_clear  (m_vcc_pin);    // Vcc
    nrf_gpio_cfg_output (m_clk_pin);
    nrf_gpio_pin_clear  (m_clk_pin);    // CLK
    nrf_gpio_cfg_output (m_rst_pin);
    nrf_gpio_pin_clear (m_rst_pin);    // RST
    //io_set_input ();                                                    // I/O
}


/**@brief Configure GPIO debug pins */
static uint32_t dbg_init ()
{
    uint32_t err_code = NRF_SUCCESS;
#if ISO7816_DBG_ENABLE


    /* Debug pin */
    if (m_dbg_activity_pin != ISO7816_INVALID_PIN_NUMBER) {
        nrf_gpio_cfg_output (m_dbg_activity_pin);
        nrf_gpio_pin_clear  (m_dbg_activity_pin);
    }
#endif

#if ISO7816_DBG_PPI_ENABLE
    uint32_t evt_addr, task_addr;

    if (m_dbg_ppi_pin != ISO7816_INVALID_PIN_NUMBER) {
        nrf_gpio_cfg_output (m_dbg_ppi_pin);
        nrf_gpio_pin_clear  (m_dbg_ppi_pin);

        //
        // Debug I/O: falling edge of I/O
        ////////////////////////////////////////////////////////

        /* Initialize GPIOTE channel for I/O debug */
        nrf_drv_gpiote_out_config_t gpiote_config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);
        err_code = nrf_drv_gpiote_out_init (m_dbg_ppi_pin, &gpiote_config);
        if (err_code != NRF_SUCCESS) {
            return err_code;
        }

        err_code = nrf_drv_ppi_channel_alloc (&m_io_dbg_ppi_edge);
        if (err_code != NRF_SUCCESS) {
            return err_code;
        }

        evt_addr  = nrf_drv_gpiote_in_event_addr_get (m_io_pin);
        task_addr = nrf_drv_gpiote_out_task_addr_get (m_dbg_ppi_pin);
        err_code  = nrf_drv_ppi_channel_assign (m_io_dbg_ppi_edge, evt_addr, task_addr);
        if (err_code != NRF_SUCCESS) {
            return err_code;
        }

        err_code = nrf_drv_ppi_channel_enable (m_io_dbg_ppi_edge);
        if (err_code != NRF_SUCCESS) {
            return err_code;
        }

        //
        // Debug I/O: 1.5 bit timer expiry
        ///////////////////////////////////////////////////////////////////////

        err_code = nrf_drv_ppi_channel_alloc (&m_io_dbg_ppi_1p5);
        if (err_code != NRF_SUCCESS) {
            return err_code;
        }

        evt_addr  = nrf_drv_timer_event_address_get (&m_low_freq_timer, NRF_TIMER_EVENT_COMPARE1);
        task_addr = nrf_drv_gpiote_out_task_addr_get (m_dbg_ppi_pin);
        err_code  = nrf_drv_ppi_channel_assign (m_io_dbg_ppi_1p5, evt_addr, task_addr);
        if (err_code != NRF_SUCCESS) {
            return err_code;
        }

        //
        // Debug I/O: 1.0 bit timer expiry
        ///////////////////////////////////////////////////////////////////////
        err_code = nrf_drv_ppi_channel_alloc (&m_io_dbg_ppi_1);
        if (err_code != NRF_SUCCESS) {
            return err_code;
        }

        evt_addr  = nrf_drv_timer_event_address_get (&m_low_freq_timer, NRF_TIMER_EVENT_COMPARE2);
        task_addr = nrf_drv_gpiote_out_task_addr_get (m_dbg_ppi_pin);
        err_code  = nrf_drv_ppi_channel_assign (m_io_dbg_ppi_1, evt_addr, task_addr);
        if (err_code != NRF_SUCCESS) {
            return err_code;
        }

        /* Enable GPIOTE for CLK signal */
        nrf_drv_gpiote_out_task_enable (m_dbg_ppi_pin);
    }
#endif

    return err_code;
}

/** @} */ // End tag for iso7816_private_func_others


/**
 * @defgroup iso7816_private_func_timers Timers
 * @ingroup  iso7816_private_func
 * @{
 */

/**@brief Bit timer handler */

struct bytes_buffer {
    uint8_t p[256];
    uint16_t size;
    uint8_t  *p_used;
    uint8_t  signal;
};
static struct bytes_buffer iso_7816_buffer;
/*retarget uart int handler to this function*/

/*retarget uart int handler to this function*/
static void bit_timer_handler ()
{
    if((iso_7816_buffer.p_used - iso_7816_buffer.p) < iso_7816_buffer.size) {
        /*it is still in */


    }
    if (m_io_out_val != 0xFF) {
        /*send data*/
        io_set_value (m_io_out_val);
    } else {
        /*rev data*/
        m_io_in_val = nrf_gpio_pin_read (m_io_pin);
    }
    DBG_SIGNAL();
    io_fsm (ISO7816_IO_FSM_EVT_BIT_TIMER, 0, 0);
    DBG_SIGNAL();
}

void rx_sucess_int (uint8_t data)
{
    if(current_mode == RX_MODE) {
        DBG_SIGNAL();
        io_fsm (ISO7816_IO_FSM_EVT_RX_START_BIT, &data, 1);
        DBG_SIGNAL();
    }
}


void tx_sucess_int ()
{

    if(current_mode == TX_MODE) {
        io_fsm (ISO7816_IO_FSM_EVT_BIT_TIMER, 0, 0);
    }

}

/**@brief System timer handler */
static void sys_timer_handler ()
{
    DBG_SIGNAL();
    sys_fsm (ISO7816_SYS_FSM_EVT_TIMEOUT, 0, 0);
    DBG_SIGNAL();
}

/**@brief System timer handler */
static void tx_timer_handler ()
{

    _iso7816_uart_tx(*iso_7816_buffer.p_used);
		iso_7816_buffer.p_used++;
	  update_io_state (ISO7816_IO_FSM_TX_DATA);
#if 0
    if(iso_7816_buffer.p_used -  iso_7816_buffer.p < iso_7816_buffer.size) {
        _iso7816_uart_tx(*iso_7816_buffer.p_used);
        update_io_state (ISO7816_IO_FSM_TX_DATA);
    }
    iso_7816_buffer.p_used++;
    /*buffer has been empty*/
    if(iso_7816_buffer.p_used -  iso_7816_buffer.p > iso_7816_buffer.size) {
        /* no bytes left: no extra guard time required */;
        iso_7816_buffer.signal = false;
        update_io_state (ISO7816_IO_FSM_IDLE);
        /*switch uart to default rxd stae*/
        //	io_set_value (1);
        io_set_input ();                    // I/O as input
        _config_iso7816_input();

    }
#endif
}

/**@brief Dummy timer handler */
static void dummy_timer_handler (nrf_timer_event_t event_type, void * p_context)
{
    // do nothing
    ;
}


/**@brief Dummy timer handler */
static void timer_handler (nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type) {
    case NRF_TIMER_EVENT_COMPARE0:
        m_low_freq_cb ();
        break;

    case NRF_TIMER_EVENT_COMPARE1:
    case NRF_TIMER_EVENT_COMPARE2:
    case NRF_TIMER_EVENT_COMPARE3:
        bit_timer_handler ();
        break;

    default:
        break;
    }
}


/**@brief Restart low frequency timer */
__attribute__((unused)) static void low_freq_timer_restart ()
{
    m_low_freq_timer.p_reg->TASKS_STOP  = 1;
    m_low_freq_timer.p_reg->TASKS_CLEAR = 1;
    m_low_freq_timer.p_reg->TASKS_START = 1;
}


/**@brief Clear timer events */
static inline void low_freq_timer_clear_events ()
{
    m_low_freq_timer.p_reg->EVENTS_COMPARE[0] = 0;
    m_low_freq_timer.p_reg->EVENTS_COMPARE[1] = 0;
    m_low_freq_timer.p_reg->EVENTS_COMPARE[2] = 0;
}


/**@brief Enable low frequency timer */
static void low_freq_timer_enable (uint8_t ch)
{
    if (ch < 3) {
        m_low_freq_timer.p_reg->SHORTS = NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK << ch;
        low_freq_timer_clear_events ();
        m_low_freq_timer.p_reg->INTENCLR = (~(NRF_TIMER_INT_COMPARE0_MASK << ch))
                                           & (NRF_TIMER_INT_COMPARE0_MASK | NRF_TIMER_INT_COMPARE1_MASK
                                              |  NRF_TIMER_INT_COMPARE2_MASK | NRF_TIMER_INT_COMPARE3_MASK);
        m_low_freq_timer.p_reg->INTENSET = NRF_TIMER_INT_COMPARE0_MASK << ch;
    } else {
        m_low_freq_timer.p_reg->SHORTS &= ~(NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK | NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK
                                            | NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK | NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK);
        m_low_freq_timer.p_reg->INTENCLR = NRF_TIMER_INT_COMPARE0_MASK | NRF_TIMER_INT_COMPARE1_MASK
                                           | NRF_TIMER_INT_COMPARE2_MASK | NRF_TIMER_INT_COMPARE3_MASK;
        m_low_freq_timer.p_reg->TASKS_STOP  = 1;
        m_low_freq_timer.p_reg->TASKS_CLEAR = 1;
        low_freq_timer_clear_events ();
    }

#if ISO7816_DBG_PPI_ENABLE
    if (m_dbg_ppi_pin != ISO7816_INVALID_PIN_NUMBER) {
        switch (ch) {
        case ISO7816_TIMER_CH_ID_1P5:
            NRF_PPI->CHENSET = PPI_CHENSET_CH0_Set << ((uint32_t) m_io_dbg_ppi_1p5);
            break;

        case ISO7816_TIMER_CH_ID_1:
            NRF_PPI->CHENSET = PPI_CHENSET_CH0_Set << ((uint32_t) m_io_dbg_ppi_1);
            break;

        default:
            NRF_PPI->CHENSET = PPI_CHENSET_CH0_Set << ((uint32_t) m_io_dbg_ppi_1p5);
            NRF_PPI->CHENSET = PPI_CHENSET_CH0_Set << ((uint32_t) m_io_dbg_ppi_1);
            break;
        }
    }
#endif
}


/**@brief Enable first byte edge compensation */
static void timer_1st_byte_comp_enable ()
{
    m_low_freq_timer.p_reg->CC[ISO7816_TIMER_CH_ID_1P5] = m_1_etu + m_0p5_etu - m_edge_comp + ISO7816_PERIOD_COMPENSATION;
}


/**@brief Disable first byte edge compensation */
__attribute__((unused)) static void timer_1st_byte_comp_disable ()
{
    m_low_freq_timer.p_reg->CC[ISO7816_TIMER_CH_ID_1P5] = m_1_etu + m_0p5_etu + ISO7816_PERIOD_COMPENSATION;
}


/**@brief Configure ETU timers */
static void timers_set_etu (uint16_t F, uint8_t D)
{
    /* Precompute ETU */
    m_1_etu   = F / D;
    m_0p5_etu = m_1_etu >> 1;

    // Falling edge compensation (circuit behavior of NRF_TIMER2)
    m_edge_comp = 4;

    nrf_drv_timer_extended_compare (&m_low_freq_timer,
                                    (nrf_timer_cc_channel_t) 1,
                                    m_1_etu + m_0p5_etu - m_edge_comp + ISO7816_PERIOD_COMPENSATION,
                                    (nrf_timer_short_mask_t) 0,
                                    false);

    nrf_drv_timer_extended_compare (&m_low_freq_timer,
                                    (nrf_timer_cc_channel_t) 2,
                                    m_1_etu + ISO7816_PERIOD_COMPENSATION,
                                    (nrf_timer_short_mask_t) 0,
                                    false);

    m_low_freq_timer.p_reg->TASKS_STOP  = 1;
    m_low_freq_timer.p_reg->TASKS_CLEAR = 1;
    low_freq_timer_clear_events ();

    m_FD_updated  = false;
}

/**@brief Initialize timers */
static uint32_t timers_init (uint16_t F, uint8_t D)
{
    uint32_t err_code;
    nrf_drv_timer_uninit(&m_high_freq_timer);
    err_code = nrf_drv_timer_init(&m_high_freq_timer, NULL, dummy_timer_handler);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }
    *(uint32_t *)0x40009C0C = 1;

    m_low_freq_timer_cfg.frequency = iso7816_timer_freq_lut [m_clk_freq];
    nrf_drv_timer_uninit(&m_low_freq_timer);
    err_code = nrf_drv_timer_init(&m_low_freq_timer, &m_low_freq_timer_cfg, timer_handler);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }
    *(uint32_t *)0x4000AC0C = 1;

#if ISO7816_GPIOTE_HIGH_SPD
    timers_set_etu (F, D);
#endif
    return NRF_SUCCESS;
}


/**@brief Uninit timers */
static uint32_t timers_cleanup ()
{
    *(uint32_t *)0x40009C0C = 1;
    nrf_drv_timer_uninit(&m_high_freq_timer);

    *(uint32_t *)0x4000AC0C = 1;
    nrf_drv_timer_uninit(&m_low_freq_timer);

    return NRF_SUCCESS;
}


/**@brief Clean-up low frequency timer */
static uint32_t low_freq_timer_cleanup ()
{
    nrf_drv_timer_disable (&m_low_freq_timer);
    return NRF_SUCCESS;
}


/**@brief Clear timer */
static void clear_timer ()
{
    m_low_freq_timer.p_reg->TASKS_CLEAR = 1;
}


/**@brief Wait for n cycles, blocking, stop afterwards */
static void wait (uint16_t n)
{
    m_last_wait = n;

    m_low_freq_timer.p_reg->INTENCLR = NRF_TIMER_INT_COMPARE0_MASK | NRF_TIMER_INT_COMPARE1_MASK
                                       | NRF_TIMER_INT_COMPARE2_MASK | NRF_TIMER_INT_COMPARE3_MASK;
    low_freq_timer_clear_events();

    nrf_drv_timer_extended_compare (&m_low_freq_timer,
                                    (nrf_timer_cc_channel_t)0,
                                    m_last_wait,
                                    (nrf_timer_short_mask_t) (NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK | NRF_TIMER_SHORT_COMPARE0_STOP_MASK),
                                    false);
    m_low_freq_timer.p_reg->EVENTS_COMPARE[0] = 0;
    nrf_drv_timer_resume (&m_low_freq_timer);

    /* Wait until expired */
    while (m_low_freq_timer.p_reg->EVENTS_COMPARE[0] == 0) ;
    m_low_freq_timer.p_reg->EVENTS_COMPARE[0] = 0;
}


/**@brief Sleep for n cycles, with background task running */
static void sleep (uint16_t n, timer_handler_t cb)
{
    m_last_wait              = n;
    m_low_freq_cb            = cb;

    m_low_freq_timer.p_reg->INTENCLR = NRF_TIMER_INT_COMPARE0_MASK | NRF_TIMER_INT_COMPARE1_MASK
                                       | NRF_TIMER_INT_COMPARE2_MASK | NRF_TIMER_INT_COMPARE3_MASK;
    low_freq_timer_clear_events();

    nrf_drv_timer_extended_compare (&m_low_freq_timer,
                                    (nrf_timer_cc_channel_t)0,
                                    m_last_wait,            // rising edge and falling edge
                                    (nrf_timer_short_mask_t) NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                    true);
    nrf_drv_timer_resume (&m_low_freq_timer);
}


/**@brief Sleep for n cycles, with background task running. Stop afterwards. */
static void sleep_and_stop (uint16_t n, timer_handler_t cb)
{
    m_last_wait              = n;
    m_low_freq_cb            = cb;

    m_low_freq_timer.p_reg->INTENCLR = NRF_TIMER_INT_COMPARE0_MASK | NRF_TIMER_INT_COMPARE1_MASK
                                       | NRF_TIMER_INT_COMPARE2_MASK | NRF_TIMER_INT_COMPARE3_MASK;
    low_freq_timer_clear_events();

    nrf_drv_timer_extended_compare (&m_low_freq_timer,
                                    (nrf_timer_cc_channel_t)0,
                                    m_last_wait,
                                    (nrf_timer_short_mask_t) (NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK | NRF_TIMER_SHORT_COMPARE0_STOP_MASK),
                                    true);
    nrf_drv_timer_resume (&m_low_freq_timer);
}

/** @} */ // End tag for iso7816_private_func_timers


/**
 * @defgroup iso7816_private_func_std ISO7816-3 implementation
 * @ingroup  iso7816_private_func
 * @{
 */
 
/**@brief Inverse coding */
__attribute__((unused)) static uint8_t inverse_code (uint8_t data) 
{
    uint8_t l_data = (iso7816_inverse_code_lut[data & 0xF] << 4)
                     | (iso7816_inverse_code_lut[(data >> 4) & 0xF]);
    return l_data;
}


/**@brief Parity computation */
__attribute__((unused)) static uint8_t parity (uint8_t data)
{
    uint8_t parity = iso7816_parity_lut[data & 0xF] ^ iso7816_parity_lut[(data >> 4) & 0xF];
    return (parity ^ m_inv);
}


/**@brief ATR parser */
static uint32_t atr_parser (uint8_t data)
{
    uint8_t i, x;

    // back up in m_atr[]
    m_atr [m_rx_byte_cnt] = data;
    if (m_atr_state != ISO7816_ATR_FSM_RDY) {
        m_rx_byte_cnt++;
    }

    switch (m_atr_state) {
    case ISO7816_ATR_FSM_INIT:      // TS: initial character
        m_atr_state = ISO7816_ATR_FSM_FORMAT;
        break;

    case ISO7816_ATR_FSM_FORMAT:    // T0: encodes Y1 and K
        m_atr_state = ISO7816_ATR_FSM_GLOBAL_1;
        m_Y = (data >> 4) & 0xF;
        m_K = data & 0xF;

    case ISO7816_ATR_FSM_GLOBAL_1:
        for (i = 0; i < 4; i++) {
            if ((m_Y & (1 << i)) != 0) {
                m_Y &= ~(1 << i);

                switch (i) {
                case 0:     // TA1: encodes Fi and Di
                    /* Prepare PPS request */
                    if (m_ignore_atr) {
                        m_Fi = ISO7816_Fd;
                        m_Di = ISO7816_Dd;
                        m_pps_req [2] = 0x11;
                    } else {
                        uint8_t idx_Fi, idx_Di;

                        idx_Fi = (data >> 4) & 0xF;
                        idx_Di = data & 0xF;

                        if (idx_Di > iso7816_MaxDi_idx_lut [m_clk_freq][idx_Fi]) {
                            idx_Di = iso7816_MaxDi_idx_lut [m_clk_freq][idx_Fi];
                        }

                        m_Fi = iso7816_Fi_lut [idx_Fi];
                        m_Di = iso7816_Di_lut [idx_Di];
                        m_pps_req [2] = (idx_Fi << 4) | idx_Di;
                    }

                    m_pps_req [3] = m_pps_req [0] ^ m_pps_req [1] ^ m_pps_req [2];  // xsum
                    break;

                case 2:     // TC1: encodes extra guard time, N
                    if (data < 255) {
                        m_N = data;
                    }
                    break;

                case 3:     // Next
                    m_atr_state = ISO7816_ATR_FSM_GLOBAL_2;
                    m_Y = (data >> 4) & 0xF;
                    m_T |= 1 << (data & 0xF);
                    break;

                default:
                    break;
                }

                if ((m_Y == 0) && (i != 3)) {
                    /* TD1 not found */
                    m_atr_state = (m_K != 0)? ISO7816_ATR_FSM_HISTORICAL: ISO7816_ATR_FSM_RDY;
                }

                i = 3;              // terminate the for-loop
            }
        }
        break;

    case ISO7816_ATR_FSM_GLOBAL_2:
        for (i = 0; i < 4; i++) {
            if ((m_Y & (1 << i)) != 0) {
                m_Y &= ~(1 << i);

                switch (i) {
                case 0:         // TA2: encodes negotiation parameters
                    m_neogatiable = ((data & 0x80) == 0)? true: false;
                    m_use_Fi_Di    = ((data & 0x10) == 0)? true: false;
                    m_T           |= 1 << (data & 0xF);
                    break;

                case 2:         // Specific to T=0: waiting interval
                    m_WI = data;
                    break;

                case 3:
                    m_atr_state = ISO7816_ATR_FSM_STRUCTURAL;
                    m_Y         = (data >> 4) & 0xF;
                    m_T        |= 1 << (data & 0xF);
                    break;

                default:
                    break;
                }

                if ((m_Y == 0) && (i != 3)) {
                    /* TD2 not found */
                    m_atr_state = (m_K != 0)? ISO7816_ATR_FSM_HISTORICAL: ISO7816_ATR_FSM_RDY;
                }

                i = 3;              // terminate the for-loop
            }
        }
        break;

    case ISO7816_ATR_FSM_STRUCTURAL:
        for (i = 0; i < 4; i++) {
            if ((m_Y & (1 << i)) != 0) {
                m_Y &= ~(1 << i);

                switch (i) {
                case 0:         // TAi: ignored
                case 1:         // TBi: ignored
                case 2:         // TCi: ignored
                    break;

                case 3:
                    m_Y  = (data >> 4) & 0xF;
                    m_T |= 1 << (data & 0xF);
                    break;

                default:
                    break;
                }

                if ((m_Y == 0) && (i != 3)) {
                    /* TDi not found */
                    m_atr_state = (m_K != 0)? ISO7816_ATR_FSM_HISTORICAL: ISO7816_ATR_FSM_RDY;
                }

                i = 3;              // terminate the for-loop
            }
        }
        break;

    case ISO7816_ATR_FSM_HISTORICAL:
        if (--m_K == 0) {
            /* requires TCK check if NOT "only T=0 is indicated" */
            m_atr_state = (m_T != 0x0001)? ISO7816_ATR_FSM_TCK: ISO7816_ATR_FSM_RDY;
        }
        break;

    case ISO7816_ATR_FSM_TCK:
        /* check TCK */
        x = 0;
        for (i = 1; i < m_rx_byte_cnt; i++) {
            x ^= m_atr [i];
        }

        m_atr_state = (x == 0)? ISO7816_ATR_FSM_RDY: ISO7816_ATR_FSM_FAIL;
        break;

    case ISO7816_ATR_FSM_FAIL:
    case ISO7816_ATR_FSM_RDY:
    default:
        break;
    }

    return NRF_SUCCESS;
}


/**@brief PPS response parser */
static uint32_t pps_parser (uint8_t data)
{
    uint8_t j, x;

    m_pps_rsp [m_rx_byte_cnt++] = data;

    switch (m_pps_state) {
    case ISO7816_PPS_FSM_WAITING:
        m_pps_state = (data == ISO7816_PPSS)? ISO7816_PPS_FSM_PPS0: ISO7816_PPS_FSM_FAIL;
        break;

    case ISO7816_PPS_FSM_PPS0:
        m_pps_state = (((data >> 4) & 0x7) != 0)? ISO7816_PPS_FSM_PPSX: ISO7816_PPS_FSM_PCK;
        m_pps_n = 0;
        break;

    case ISO7816_PPS_FSM_PPSX:
        if (++m_pps_n >= iso7816_bcnt_lut [(m_pps_rsp[1] >> 4) & 0x7]) {
            m_pps_state = ISO7816_PPS_FSM_PCK;
        }
        break;

    case ISO7816_PPS_FSM_PCK:
        /* Check-sum */
        x = 0;
        for (j = 0; j < m_rx_byte_cnt; j++) {
            x ^= m_pps_rsp[j];
        }

        if (x != 0) {
            m_pps_state = ISO7816_PPS_FSM_FAIL;
            break;
        }

        /* T */
        if (((m_pps_rsp[1] ^ m_pps_req[1]) & 0x0F) != 0) {
            m_pps_state = ISO7816_PPS_FSM_FAIL;
            break;
        }

        /* PPS1 */
        if ((m_pps_rsp[1] & 0x10) != 0) {           // PPS1 exist?
            if (m_pps_rsp[2] != m_pps_req[2]) {
                m_pps_state = ISO7816_PPS_FSM_FAIL;
                break;
            }
        } else {
            // Use default: Fd and Dd
            m_Fi = ISO7816_Fd;
            m_Di = ISO7816_Dd;
        }

        /* PPS2 */
        if ((m_pps_rsp[1] & 0x20) != 0) {           // PPS2 exist?
            m_pps_state = ISO7816_PPS_FSM_FAIL;
            break;
        }

        /* PPS3 */
        if ((m_pps_rsp[1] & 0x40) != 0) {           // PPS3 exist?
            m_pps_state = ISO7816_PPS_FSM_FAIL;
            break;
        }

        m_pps_state = ISO7816_PPS_FSM_RDY;
        break;

    default:
        break;
    }

    return NRF_SUCCESS;
}


/**@brief Cold Reset */
static uint32_t cold_reset ()
{
    nrf_gpio_pin_clear (m_rst_pin);     // RST=0
    nrf_gpio_pin_set (m_vcc_pin);       // Vcc=1
    DBG_SIGNAL();
    clk_enable ();                      // CLK oscillate
    DBG_SIGNAL();
    io_set_value (1);
    io_set_input ();                    // I/O as input

    // PKChan: wait (400);
    wait (400 * 12);

    if (io_get_value() != 1) {
        return NRF_ERROR_TIMEOUT;
    } else {
        /*wait for atr response so uart rev int is need to be initilize here*/

        //   _config_iso7816_output();
        //  _config_iso7816_input();
        //		_config_iso7816_output();
        _config_iso7816_input();
        nrf_gpio_pin_set (m_rst_pin);   // RST=1
        DBG_SIGNAL();


        /*=============================config uart here and ready to recieve data========================================*/
        /**/
        /*
        wait (400);
        timer_1st_byte_comp_enable ();
        io_wait_for_start_bit ();
        */
        DBG_SIGNAL();
        //sleep_and_stop (ISO7816_MAX_WAIT, sys_timer_handler);
        m_total_wait = 0;
    }

    return NRF_SUCCESS;
}


/**@brief Warm reset, phase 1
 * @note  Run in interrupt.
 */
static void warm_reset_0 ()
{
    nrf_gpio_pin_clear (m_rst_pin);     // RST=0
    io_set_input ();
    DBG_SIGNAL();

    // reset states
    m_io_state      = ISO7816_IO_FSM_IDLE;
    m_io_state_prev = ISO7816_IO_FSM_IDLE;
    m_atr_state     = ISO7816_ATR_FSM_INIT;
    m_pps_state     = ISO7816_PPS_FSM_WAITING;
    m_tpdu_state    = ISO7816_TPDU_FSM_INIT;

    m_Fi = ISO7816_Fd;
    m_Di = ISO7816_Dd;
    timers_set_etu (m_Fi, m_Di);

    m_rx_byte_cnt = 0;
    m_wt_enabled  = true;

    clear_timer ();
    sleep_and_stop (400, warm_reset_1);
}


/**@brief Warm reset, phase 2
 * @note  Run in interrupt.
 */
static void warm_reset_1 ()
{
    nrf_gpio_pin_set (m_rst_pin);       // RST=1
    timer_1st_byte_comp_enable ();
    io_wait_for_start_bit ();
    DBG_SIGNAL();
    sleep_and_stop (ISO7816_MAX_WAIT, sys_timer_handler);
    m_total_wait = 0;
}


/**@brief Deactivation and cleanup */
static uint32_t deactivate ()
{
    /* Signals */
    nrf_gpio_pin_clear (m_rst_pin);     // RST=0
    clk_disable();                      // Disable clock
    nrf_gpio_pin_clear (m_clk_pin);     // CLK=0
    io_disable();                       // I/O=0, and disable
    nrf_gpio_pin_clear (m_vcc_pin);     // Vcc=0
    DBG_SIGNAL();
    /* Finalize values */
    m_sys_state     = ISO7816_SYS_FSM_OFF;
    m_io_state      = ISO7816_IO_FSM_DISABLED;
    m_io_state_prev = ISO7816_IO_FSM_DISABLED;
    m_atr_state     = ISO7816_ATR_FSM_INIT;
    m_pps_state     = ISO7816_PPS_FSM_WAITING;
    m_tpdu_state    = ISO7816_TPDU_FSM_INIT;
    m_inv           = 0;

    /* Release SDK resources */
    clk_cleanup ();
    low_freq_timer_cleanup ();
    timers_cleanup ();

    return NRF_SUCCESS;
}


/**@brief Send RAW data through I/O */
static uint32_t iso7816_send (uint8_t *data, uint16_t len)
{
    uint32_t err_code = NRF_SUCCESS;
//    int i;
    update_io_state (ISO7816_IO_FSM_IDLE);
    err_code = io_fsm (ISO7816_IO_FSM_EVT_TX_REQ, data, len);
    return err_code;
}


/**
 * @defgroup iso7816_private_func_std_clk_signal CLK signal
 * @{
 */

/**@brief Initialize CLK */
static uint32_t clk_init (iso7816_freq_t clk_freq)
{
    uint32_t err_code;
    uint32_t evt_addr, task_addr;
    nrf_drv_gpiote_out_config_t gpiote_config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);

    /* Initialize GPIOTE channel for CLK */
    nrf_drv_gpiote_out_uninit(m_clk_pin);
    err_code = nrf_drv_gpiote_out_init (m_clk_pin, &gpiote_config);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    /* Set up CLK Timer COMPARE #0 */
    nrf_drv_timer_extended_compare (&m_high_freq_timer,
                                    (nrf_timer_cc_channel_t)0,
                                    iso7816_cc_lut [clk_freq],
                                    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                    false);
    nrf_drv_ppi_uninit();
    nrf_drv_ppi_init();
    /* Connect PPI and enable */
    err_code = nrf_drv_ppi_channel_alloc (&m_high_freq_ppi);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    evt_addr  = nrf_drv_timer_event_address_get (&m_high_freq_timer, NRF_TIMER_EVENT_COMPARE0);
    task_addr = nrf_drv_gpiote_out_task_addr_get (m_clk_pin);
    err_code  = nrf_drv_ppi_channel_assign (m_high_freq_ppi, evt_addr, task_addr);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = nrf_drv_ppi_channel_enable (m_high_freq_ppi);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    /* Enable GPIOTE for CLK signal */
    nrf_drv_gpiote_out_task_enable (m_clk_pin);

    return NRF_SUCCESS;
}


/**@brief Enable CLK output */
static void clk_enable ()
{
    nrf_drv_timer_enable (&m_high_freq_timer);
}


/**@brief Disable CLK output */
static void clk_disable ()
{
    nrf_drv_timer_disable (&m_high_freq_timer);
}


/**@brief Cleanup CLK */
static uint32_t clk_cleanup ()
{
    nrf_drv_timer_disable (&m_high_freq_timer);
    nrf_drv_gpiote_out_uninit (m_clk_pin);
    nrf_drv_ppi_channel_free (m_high_freq_ppi);
    return NRF_SUCCESS;
}

/** @} */ // End tag for iso7816_private_func_std_clk_signal


/**
 * @defgroup iso7816_private_func_std_io_signal I/O signal.
 * @{
 */

/**@brief Initialize I/O */
static uint32_t io_init ()
{
    uint32_t err_code;
//    uint32_t evt_addr, task_addr;
    nrf_drv_gpiote_in_config_t io_gpiote_param;

    io_gpiote_param.sense       = NRF_GPIOTE_POLARITY_HITOLO;
    io_gpiote_param.pull        = NRF_GPIO_PIN_PULLUP;
    io_gpiote_param.is_watcher  = false;
    io_gpiote_param.hi_accuracy = true;
    err_code = nrf_drv_gpiote_in_init (m_io_pin,
                                       &io_gpiote_param,
                                       gpiote_event_handler);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

#if ISO7816_GPIOTE_HIGH_SPD
    err_code = nrf_drv_ppi_channel_alloc (&m_low_freq_ppi);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    evt_addr  = nrf_drv_gpiote_in_event_addr_get (m_io_pin);
    task_addr = nrf_drv_timer_task_address_get (&m_low_freq_timer, NRF_TIMER_TASK_START);
    err_code  = nrf_drv_ppi_channel_assign (m_low_freq_ppi, evt_addr, task_addr);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = nrf_drv_ppi_channel_enable (m_low_freq_ppi);
    if (err_code != NRF_SUCCESS) {
        return false;
    }

    err_code = nrf_drv_ppi_channel_alloc (&m_low_freq_ppi_clr);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    evt_addr  = nrf_drv_gpiote_in_event_addr_get (m_io_pin);
    task_addr = nrf_drv_timer_task_address_get (&m_low_freq_timer, NRF_TIMER_TASK_CLEAR);
    err_code  = nrf_drv_ppi_channel_assign (m_low_freq_ppi_clr, evt_addr, task_addr);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = nrf_drv_ppi_channel_enable (m_low_freq_ppi_clr);
    if (err_code != NRF_SUCCESS) {
        return false;
    }
#endif

    return err_code;
}


/**@brief Set I/O pin as input */
static void io_set_input ()
{
    nrf_gpio_cfg_input (m_io_pin, NRF_GPIO_PIN_PULLUP);
}


/**@brief Disable I/O pin (set input and pull low) */
static void io_disable ()
{
    nrf_drv_gpiote_in_uninit (m_io_pin);
    nrf_gpio_cfg_input (m_io_pin, NRF_GPIO_PIN_PULLDOWN);
}


/**@brief I/O to wait for start bit */
static bool io_wait_for_start_bit ()
{
    nrf_drv_gpiote_in_event_enable (m_io_pin, true);
    return true;
}


/**@brief Set I/O pin as output */
static void io_set_output ()
{
    nrf_gpio_cfg_output (m_io_pin);
    nrf_gpio_pin_clear (m_io_pin);
}


/**@brief Get I/O value */
static uint32_t io_get_value ()
{
    return nrf_gpio_pin_read (m_io_pin);
}


/**@brief Get I/O value (cached) */
__attribute__((unused)) static uint32_t io_get_cached_value ()
{
    return m_io_in_val;
}


/**@brief Set I/O value */
static void io_set_value (uint8_t val)
{
    nrf_gpio_pin_write(m_io_pin, val);
}

/** @} */ // End tag for iso7816_private_func_std_io_signal
/** @} */ // End tag for iso7816_private_func_std



/**@brief Initialization and activation
 * @ingroup iso7816_private_func_others
 */
static uint32_t on_init (iso7816_init_t *init_param)
{
    uint32_t err_code;

    //
    // Initial values
    //
    /* Generic */
    m_vcc_pin          = init_param->vcc_pin;
    m_clk_pin          = init_param->clk_pin;
    m_rst_pin          = init_param->rst_pin;
    m_io_pin           = init_param->io_pin;
    m_dbg_activity_pin = init_param->dbg_activity_pin;
    m_dbg_ppi_pin      = init_param->dbg_ppi_pin;
    m_evt_cb           = init_param->evt_cb;
    m_clk_freq         = init_param->clock_freq;
    m_inv              = 0;
    m_low_freq_cb      = 0;
    m_tx_byte_cnt      = 0;
    m_rx_byte_cnt      = 0;
    m_io_out_val       = 0xFF;

    /* ATR */
    m_atr_state   = ISO7816_ATR_FSM_INIT;
    m_atr[0]      = ISO7816_INVALID_BYTE;
    m_T           = 0x0001;
    m_Y           = 0;
    m_K           = 0;
    m_use_Fi_Di   = false;
    m_Fi          = ISO7816_Fd;
    m_Di          = ISO7816_Dd;
    m_N           = 0;
    m_WI          = ISO7816_WT - 12 - m_N;
    m_neogatiable = false;
    m_ignore_atr  = false;
    m_wt_enabled  = true;

    /* PPS */
    m_pps_state   = ISO7816_PPS_FSM_WAITING;
    m_pps_req [2] = 0x01;
    m_pps_req [3] = m_pps_req [0] ^ m_pps_req [1] ^ m_pps_req [2];  // xsum

    /* TPDU exchange */
    m_tpdu_state = ISO7816_TPDU_FSM_INIT;

    /* Initialize SDK resources */
    gpio_init ();

    /*add uart=========================================== init here*/
#if 1
    err_code = timers_init (ISO7816_Fd, ISO7816_Dd);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }
    /* Initialize I/O signal */
    err_code = io_init ();
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

#endif

    /* Initialize CLK signal */
    err_code = clk_init (init_param->clock_freq);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }


    /* Initialize I/O dbg signal */
    err_code = dbg_init ();
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    /* Cold reset */
    err_code = cold_reset ();
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    m_io_state      = ISO7816_IO_FSM_IDLE;
    m_io_state_prev = ISO7816_IO_FSM_IDLE;
    return NRF_SUCCESS;
}


/**
 * @defgroup iso7816_private_func_fsm Finite state machines
 * @ingroup  iso7816_private_func
 * @{
 */

/**@brief Finite state machine (system) */
static uint32_t sys_fsm (iso7816_sys_fsm_evt_t evt, void *data, uint8_t len)
{
    uint32_t err_code = NRF_SUCCESS;
    iso7816_evt_t l_evt;
    uint32_t l_wt, l_max_wait;
    uint8_t i;
    switch (m_sys_state) {
    case ISO7816_SYS_FSM_OFF:
        if (evt == ISO7816_SYS_FSM_EVT_INIT) {
            iso7816_init_t *init_param = (iso7816_init_t *) data;

            m_sys_state = ISO7816_SYS_FSM_INIT;
            err_code = on_init (init_param);
        } else {
            err_code = NRF_ERROR_INVALID_STATE;
        }
        break;

    case ISO7816_SYS_FSM_INIT:
        switch (evt) {
        case ISO7816_SYS_FSM_EVT_RX:
            atr_parser (*(uint8_t *)data);
            if (m_atr_state == ISO7816_ATR_FSM_RDY) {
                NRF_LOG_PRINTF("ATR: ");
                for(i = 0; i < m_rx_byte_cnt; i++) {
                    NRF_LOG_PRINTF("%X ", m_atr[i]);
                }
                NRF_LOG_PRINTF("\r\n");


                m_sys_state = ISO7816_SYS_FSM_RDY;

                /* Precompute ETU */
                m_FD_updated = true;
                m_wt_enabled = false;

                err_code = tpdu_fsm (ISO7816_TPDU_FSM_EVT_SYS_RDY, 0, 0);
                /*after rev, switch io fsm to default state*/
                update_io_state (ISO7816_IO_FSM_IDLE);
                if (m_evt_cb != 0) {
                    NRF_LOG_PRINTF("evt id rdy\r\n");
                    l_evt.evt_id = ISO7816_EVT_ID_RDY;
                    m_evt_cb(&l_evt);
                }

                if (err_code == NRF_SUCCESS) {
                    m_rx_byte_cnt = 0;
                }
            }
            break;

        case ISO7816_SYS_FSM_EVT_TIMEOUT:
            //NRF_LOG_PRINTF("time out\r\n");
            m_total_wait += m_last_wait;
            if (m_rx_byte_cnt == 0) {
                l_max_wait = ISO7816_ATR_MAX_WAIT;  // waiting for the initial byte
            } else {
                l_max_wait = m_WI * m_1_etu;        // Inter-character timeout (during ATR)
            }

            if (m_total_wait >= l_max_wait) {
                /* timeout after 40000 cycles */
                deactivate();

                if (m_evt_cb != 0) {
                    l_evt.evt_id = ISO7816_EVT_ID_DEACTIVATED;
                    m_evt_cb(&l_evt);
                }
            } else {
                /* continue waiting */
                if ((l_wt = l_max_wait - m_total_wait) >= ISO7816_MAX_WAIT) {
                    l_wt = ISO7816_MAX_WAIT;
                    sleep (l_wt, sys_timer_handler);
                } else {
                    sleep_and_stop (l_wt, sys_timer_handler);
                }
            }
            break;

        default:
            break;
        }
        break;

    case ISO7816_SYS_FSM_ATR_RECVD:
        switch (evt) {
        case ISO7816_SYS_FSM_EVT_RX:
            pps_parser (*(uint8_t *)data);
            NRF_LOG_PRINTF("%X ", *(uint8_t *)data);
            if (m_pps_state == ISO7816_PPS_FSM_RDY) {
                NRF_LOG_PRINTF("\r\nPPS RSP: ");
                for(i = 0; i < m_rx_byte_cnt; i++) {
                    NRF_LOG_PRINTF("%X ", m_pps_rsp[i]);
                }
                NRF_LOG_PRINTF("\r\n");
                m_sys_state = ISO7816_SYS_FSM_RDY;

                /* Precompute ETU */
                m_FD_updated = true;
                m_wt_enabled = false;

                err_code = tpdu_fsm (ISO7816_TPDU_FSM_EVT_SYS_RDY, 0, 0);

                if (m_evt_cb != 0) {
                    NRF_LOG_PRINTF("evt id rdy\r\n");
                    l_evt.evt_id = ISO7816_EVT_ID_RDY;
                    m_evt_cb(&l_evt);
                }
            } else if (m_pps_state == ISO7816_PPS_FSM_FAIL) {
                warm_reset_0 ();
                m_sys_state = ISO7816_SYS_FSM_INIT;
            }
            break;

        case ISO7816_SYS_FSM_EVT_TX:
            clear_timer ();
            break;

        case ISO7816_SYS_FSM_EVT_TIMEOUT:
            m_total_wait += m_last_wait;
            l_max_wait = m_WI * m_1_etu;        // Inter-character timeout

            if (m_total_wait >= l_max_wait) {
                /* Timeout: use default parameters */
                m_ignore_atr = true;

                warm_reset_0 ();
                m_sys_state = ISO7816_SYS_FSM_INIT;
            } else {
                /* continue waiting */
                if ((l_wt = l_max_wait - m_total_wait) >= ISO7816_MAX_WAIT) {
                    l_wt = ISO7816_MAX_WAIT;
                    sleep (l_wt, sys_timer_handler);
                } else {
                    sleep_and_stop (l_wt, sys_timer_handler);
                }
            }
            break;

        default:
            break;
        }
        break;

    case ISO7816_SYS_FSM_RDY:
        switch (evt) {
        case ISO7816_SYS_FSM_EVT_CLEANUP:
            m_sys_state = ISO7816_SYS_FSM_OFF;
            break;

        case ISO7816_SYS_FSM_EVT_RX:
            err_code = tpdu_fsm (ISO7816_TPDU_FSM_EVT_RX_DATA, data, len);
            break;

        case ISO7816_SYS_FSM_EVT_TX:
            err_code = tpdu_fsm (ISO7816_TPDU_FSM_EVT_TX_RDY, 0, 0);
            break;

        case ISO7816_SYS_FSM_EVT_TIMEOUT:
            m_total_wait += m_last_wait;
            l_max_wait = m_WI * m_1_etu;        // Inter-character timeout

            if (m_total_wait >= l_max_wait) {
                if (m_wt_enabled) {
                    m_ignore_atr = true;

                    warm_reset_0 ();
                    m_sys_state = ISO7816_SYS_FSM_INIT;
                } else {
                    DBG_SIGNAL();
                }
            } else {
                /* continue waiting */
                if ((l_wt = l_max_wait - m_total_wait) >= ISO7816_MAX_WAIT) {
                    l_wt = ISO7816_MAX_WAIT;
                    sleep (l_wt, sys_timer_handler);
                } else {
                    sleep_and_stop (l_wt, sys_timer_handler);
                }
            }
            break;

        default:
            break;
        }
        break;

    default:
        m_sys_state = ISO7816_SYS_FSM_OFF;
        break;
    }

    return err_code;
}


/**@brief Update I/O state machine */
static void update_io_state (iso7816_io_fsm_t new_state)
{
    m_io_state_prev = m_io_state;
    m_io_state      = new_state;
}

__STATIC_INLINE void nrf_uart_enable()
{
    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled;
}

__STATIC_INLINE void nrf_uart_disable()
{
    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Disabled;
}

static uint8_t  uart_para_init = false;
void _config_iso7816_output()
{

    // nrf_uart_disable();
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NRF_GPIO->PIN_CNF[m_io_pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                  | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)
                                  | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
                                  | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                  | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

    //io_set_output ();
    //
    // UART Interrupt setting
    // Enable RX interrupt, Disable TX Interrupt.
    //
    NRF_UART0->TASKS_STARTRX = 0;
    NRF_UART0->TASKS_STARTTX = 0;
    NRF_UART0->INTENSET =0;
    NRF_UART0->INTENCLR = 0XFF;

   if(uart_para_init == false){

    NRF_UART0->BAUDRATE = (_BAUDRATE_10753 << UART_BAUDRATE_BAUDRATE_Pos);
    //NRF_UART0->ENABLE   = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
    NRF_UART0->CONFIG   = (UART_CONFIG_PARITY_Included << UART_CONFIG_PARITY_Pos);
		uart_para_init = true;
	 }
	  NRF_UART0->PSELTXD  = (m_io_pin);
    NRF_UART0->PSELRXD  = (GPIO_UART_RX);
    NRF_UART0->INTENSET = (UART_INTENSET_TXDRDY_Enabled << UART_INTENSET_TXDRDY_Pos) | UART_INTENSET_ERROR_Msk;
    NRF_UART0->INTENCLR = (UART_INTENSET_RXDRDY_Enabled << UART_INTENCLR_RXDRDY_Pos);

    current_mode = TX_MODE;
    //nrf_uart_enable();

    NVIC_SetPriority(UART0_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(UART0_IRQn);
    NRF_UART0->ERRORSRC = 0;
    NRF_UART0->TASKS_STARTTX = 1;

    // wait(2500);
}


void _config_iso7816_input()
{
    //nrf_uart_disable();
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NRF_UART0->TASKS_STARTRX = 0;
    NRF_UART0->TASKS_STARTTX = 0;

    NRF_GPIO->PIN_CNF[m_io_pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                  | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)
                                  | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                  | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                  | (GPIO_PIN_CNF_DIR_Input  << GPIO_PIN_CNF_DIR_Pos);

    //
    // UART Interrupt setting
    // Enable RX interrupt, Disable TX Interrupt.
    //
    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->INTENSET =0;
    NRF_UART0->INTENCLR = 0XFF;

    NRF_UART0->PSELTXD  = (GPIO_UART_TX);
    NRF_UART0->PSELRXD  = (m_io_pin);
		
   if(uart_para_init == false){

    NRF_UART0->BAUDRATE = (_BAUDRATE_10753 << UART_BAUDRATE_BAUDRATE_Pos);
    //NRF_UART0->ENABLE   = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
    NRF_UART0->CONFIG   = (UART_CONFIG_PARITY_Included << UART_CONFIG_PARITY_Pos);
		uart_para_init = true;
	 }
    NVIC_SetPriority(UART0_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(UART0_IRQn);

    //NRF_UART0->INTENSET = (UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos) | (UART_INTENSET_TXDRDY_Disabled << UART_INTENSET_TXDRDY_Pos);
    NRF_UART0->INTENSET = (UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos) | UART_INTENSET_ERROR_Msk;
    NRF_UART0->INTENCLR = (UART_INTENSET_TXDRDY_Enabled << UART_INTENCLR_TXDRDY_Pos);
    NRF_UART0->ERRORSRC = 0;
    if(NRF_UART0->ENABLE == 0) {
        nrf_uart_enable();
    }
    current_mode = RX_MODE;
    NRF_UART0->TASKS_STARTRX = 1;
//NRF_UART0->INTENCLR = (UART_INTENCLR_RXDRDY_Enabled << UART_INTENCLR_RXDRDY_Pos) | (UART_INTENCLR_TXDRDY_Disabled << UART_INTENCLR_RXDRDY_Pos);
}

static int _iso7816_uart_tx(uint8_t ch)
{
//    I32U time_out = 10000;
    NRF_UART0->TXD = (uint8_t)ch;

    return 1;
}
/**@brief Finite state machine (I/O) */
static uint32_t io_fsm (iso7816_io_fsm_evt_t evt, void *data, uint8_t len)
{
    uint32_t err_code = NRF_SUCCESS;
//    int i;

    switch (m_io_state) {
    case ISO7816_IO_FSM_IDLE:
        /* Check if start bit is detected. */
        switch (evt) {

        case  ISO7816_IO_FSM_EVT_BIT_TIMER:
             // sleep_and_stop (20000, tx_timer_handler);
                if((iso_7816_buffer.p_used -  iso_7816_buffer.p < iso_7816_buffer.size)) {
									  update_io_state (ISO7816_IO_FSM_TX_DATA);
                    sleep_and_stop (5000, tx_timer_handler);
                } else {
                    iso_7816_buffer.signal = false;
                    update_io_state (ISO7816_IO_FSM_IDLE);
                    /*switch uart to default rxd stae*/
                    //	io_set_value (1);
                    io_set_input ();                    // I/O as input
                    _config_iso7816_input();
                }
            break;
        case  ISO7816_IO_FSM_EVT_RX_START_BIT:
            err_code = sys_fsm (ISO7816_SYS_FSM_EVT_RX, data, 1);
            update_io_state (ISO7816_IO_FSM_RX_DATA);
            break;
        case ISO7816_IO_FSM_EVT_TX_REQ:
            if (io_get_value() != 0) {
                memset(&iso_7816_buffer, 0, sizeof(iso_7816_buffer));
                memcpy(iso_7816_buffer.p, data, len);
                iso_7816_buffer.p_used = iso_7816_buffer.p;
                iso_7816_buffer.signal = true;
                iso_7816_buffer.size = len;
                _config_iso7816_output();
                // sleep_and_stop (20000, tx_timer_handler);
                if((iso_7816_buffer.p_used -  iso_7816_buffer.p < iso_7816_buffer.size)) {
                    sleep_and_stop (10000, tx_timer_handler);
									  update_io_state (ISO7816_IO_FSM_TX_DATA);
                } else {
                    iso_7816_buffer.signal = false;
                    update_io_state (ISO7816_IO_FSM_IDLE);
                    /*switch uart to default rxd stae*/
                    //	io_set_value (1);
                    io_set_input ();                    // I/O as input
                    _config_iso7816_input();
                }


            } else {
                err_code = NRF_ERROR_BUSY;
            }
            break;
//            break;
        default:
            break;
        }
        break;

    case ISO7816_IO_FSM_RX_DATA:
        /* Check if bit timer expired. */
        if(evt == ISO7816_IO_FSM_EVT_RX_START_BIT) {
            err_code = sys_fsm (ISO7816_SYS_FSM_EVT_RX, data, 1);
        }
        //update_io_state (ISO7816_IO_FSM_RX_WAIT);
        break;



    case ISO7816_IO_FSM_RX_PAUSE:
        /* Check if bit timer expired. */
        if (evt == ISO7816_IO_FSM_EVT_BIT_TIMER) {
            err_code = sys_fsm (ISO7816_SYS_FSM_EVT_RX, &m_rx_data, 1);
            io_wait_for_start_bit();
            update_io_state (ISO7816_IO_FSM_RX_WAIT);
        }
        break;

    case ISO7816_IO_FSM_RX_ERROR:
        /* Check if bit timer expired. */
        if (evt == ISO7816_IO_FSM_EVT_BIT_TIMER) {
            m_rx_wait_bit--;
            switch (m_rx_wait_bit) {
            case 1:
                io_set_output();
                break;

            case 0:
                io_set_input();
                io_wait_for_start_bit();
                update_io_state (ISO7816_IO_FSM_RX_WAIT);
                break;

            default:
                break;
            }
        }
        break;

    case ISO7816_IO_FSM_RX_WAIT:        // Warning! 0.5 ETU only, don't schedule many tasks here
        if (evt == ISO7816_IO_FSM_EVT_BIT_TIMER) {
            low_freq_timer_enable (ISO7816_TIMER_CH_ID_1);
            update_io_state (ISO7816_IO_FSM_IDLE);
        }
        break;

    case ISO7816_IO_FSM_TX_DATA:

        if((iso_7816_buffer.p_used -  iso_7816_buffer.p < iso_7816_buffer.size)) {
            sleep_and_stop (5000, tx_timer_handler);
					  update_io_state (ISO7816_IO_FSM_TX_DATA);
        } else {
            iso_7816_buffer.signal = false;
            update_io_state (ISO7816_IO_FSM_IDLE);
            /*switch uart to default rxd stae*/
            //	io_set_value (1);
            io_set_input ();                    // I/O as input
            _config_iso7816_input();
        }

        break;

    case ISO7816_IO_FSM_TX_WAIT_0:



    case ISO7816_IO_FSM_TX_START:
        /* Check if bit timer expired. */

        break;

    default:
        break;
    }
    return err_code;
}

static uint8_t wait_for_response = false;
/**@brief TPDU exchange state machine. */
#define  	ONE_MORE_LOOP_START()    start:
#define   EXECUTE_ONE_MORE_LOOP()  goto  start
static uint32_t tpdu_fsm (iso7816_tpdu_fsm_evt_t evt, void *data, uint8_t len)
{
    uint32_t err_code = NRF_SUCCESS;
   // static uint8_t wait_for_onother_sw_char = false;
    ONE_MORE_LOOP_START()
    switch (m_tpdu_state) {
    case ISO7816_TPDU_FSM_INIT:
        /* Right after lower layer initialization */
        if (evt == ISO7816_TPDU_FSM_EVT_SYS_RDY) {
            m_tpdu_state = ISO7816_TPDU_FSM_IDLE;
        }

    case ISO7816_TPDU_FSM_IDLE:
        /* TPDU exchange request */
        if (evt == ISO7816_TPDU_FSM_EVT_REQ) {
            /* Parameter checks */
            if (
                ((m_tpdu.req != ISO7816_TPDU_REQ_GET) && (m_tpdu.req != ISO7816_TPDU_REQ_PUT))  // Neither GET nor PUT
                || ((m_tpdu.req == ISO7816_TPDU_REQ_PUT) && (m_tpdu.len >= 256))                   // PUT but len >= 256
            ) {
                err_code = NRF_ERROR_INVALID_PARAM;
                break;
            }

            memcpy (&m_tpdu, data, sizeof(iso7816_tpdu_t));
            m_tpdu.reserved = m_tpdu.ins ^ 0xFF;            // for procedure byte checking
            m_wt_enabled = false;

            /* Build TPDU header */
            m_tpdu_hdr [0] = m_tpdu.cla;
            m_tpdu_hdr [1] = m_tpdu.ins;
            m_tpdu_hdr [2] = m_tpdu.p1;
            m_tpdu_hdr [3] = m_tpdu.p2;
            m_tpdu_hdr [4] = m_tpdu.len & 0x00FF;

            /* Send header */
            err_code = iso7816_send (m_tpdu_hdr, ISO7816_TPDU_HDR_LEN);
            if (err_code == NRF_SUCCESS) {
                m_tpdu_state = ISO7816_TPDU_FSM_HEADER;
                wait_for_response = true;
            }
            break;
        } else if(evt == ISO7816_TPDU_FSM_EVT_RX_DATA) {
            /*this is not active cha so ignore it*/
            if(wait_for_response == false) {
                update_io_state (ISO7816_IO_FSM_IDLE);
                break;
            } else {
                m_tpdu_state = ISO7816_TPDU_FSM_LISTEN;
            }
        }
    case ISO7816_TPDU_FSM_HEADER:
        if (evt == ISO7816_TPDU_FSM_EVT_TX_RDY) {
            m_tpdu_state = ISO7816_TPDU_FSM_LISTEN;
        } else if(evt == ISO7816_TPDU_FSM_EVT_RX_DATA) {
            m_tpdu_state = ISO7816_TPDU_FSM_LISTEN;
        } else {
            break;
        }


    case ISO7816_TPDU_FSM_LISTEN:

        if (evt == ISO7816_TPDU_FSM_EVT_RX_DATA) {
            // Assumption: only one procedure byte is received
            uint8_t procd = *(uint8_t *)data;
            wait_for_response = false;
            if (procd == ISO7816_TPDU_PROCD_NULL) {
                /* NULL byte received */
                /* No action required, and wait for another procedure byte. */
                break;
            } else if (
                ((procd & ISO7816_TPDU_PROCD_SW1_MASK) == ISO7816_TPDU_PROCD_SW1_OK)
                || ((procd & ISO7816_TPDU_PROCD_SW1_MASK) == ISO7816_TPDU_PROCD_SW1_ERR)
            ) {
                /* SW1 received, waiting for SW2 */
                m_tpdu_rsp.sw1 = procd;
                m_tpdu_state   = ISO7816_TPDU_FSM_SW1;
            } else if (procd == m_tpdu.ins) {
                /* ACK (INS) received, send/receive all data */
                switch (m_tpdu.req) {
                case ISO7816_TPDU_REQ_GET:
                    m_tpdu_state = ISO7816_TPDU_FSM_RX_DATA;
                    break;

                case ISO7816_TPDU_REQ_PUT:
                    wait_for_response = true;
                    //wait_for_onother_sw_char = false;
                    err_code = iso7816_send (m_tpdu.data, m_tpdu.len);
                    if (err_code == NRF_SUCCESS) {
                        m_tpdu_state = ISO7816_TPDU_FSM_TX_DATA;
                        m_tpdu.data += m_tpdu.len;
                        m_tpdu.len   = 0;
                    }

                default:
                    err_code = NRF_ERROR_INTERNAL;
                    break;
                }
            } else if (procd == m_tpdu.reserved) {
                /* ACK (INS^0xFF) received, send/receive one byte. */
                switch (m_tpdu.req) {
                case ISO7816_TPDU_REQ_GET:
                    m_wt_enabled = true;
                    m_tpdu_state = ISO7816_TPDU_FSM_RX_1BYTE;
                    break;

                case ISO7816_TPDU_REQ_PUT:
                    m_wt_enabled = false;
                    err_code = iso7816_send (m_tpdu.data, 1);
                    if (err_code == NRF_SUCCESS) {
                        m_tpdu_state = ISO7816_TPDU_FSM_TX_DATA;
                        m_tpdu.data++;
                        m_tpdu.len--;
                    }
                    break;
 
                default:
                    err_code = NRF_ERROR_INTERNAL;
                    break;
                }
            } else {
                err_code = NRF_ERROR_INTERNAL;
            }
        }

        break;



    case ISO7816_TPDU_FSM_TX_DATA:
        if (evt == ISO7816_TPDU_FSM_EVT_TX_RDY) {
            m_tpdu_state = ISO7816_TPDU_FSM_LISTEN;
        } else if(evt == ISO7816_TPDU_FSM_EVT_RX_DATA) {
            m_tpdu_state = ISO7816_TPDU_FSM_LISTEN;
            EXECUTE_ONE_MORE_LOOP();
        }
        break;

    case ISO7816_TPDU_FSM_RX_DATA:
        if (evt == ISO7816_TPDU_FSM_EVT_RX_DATA) {
            *m_tpdu.data++ = *(uint8_t *)data;
            if (--m_tpdu.len == 0) {
                m_tpdu_state = ISO7816_TPDU_FSM_LISTEN;
            }
        }
        break;

    case ISO7816_TPDU_FSM_RX_1BYTE:
        if (evt == ISO7816_TPDU_FSM_EVT_RX_DATA) {
            m_tpdu_state   = ISO7816_TPDU_FSM_LISTEN;
            *m_tpdu.data++ = *(uint8_t *)data;
            m_tpdu.len--;
        }
        break;

    case ISO7816_TPDU_FSM_SW1:
        if (evt == ISO7816_TPDU_FSM_EVT_RX_DATA) {
            iso7816_evt_t l_evt;

            m_wt_enabled = false;
            m_tpdu_rsp.sw2 = *(uint8_t *)data;
            m_tpdu_state   = ISO7816_TPDU_FSM_IDLE;
            update_io_state (ISO7816_IO_FSM_IDLE);
            /* Notify application */
            if (m_evt_cb != 0) {
                l_evt.evt_id           = ISO7816_EVT_ID_TPDU_RESPONSE;
                l_evt.evt.tpdu_rsp.sw1 = m_tpdu_rsp.sw1;
                l_evt.evt.tpdu_rsp.sw2 = m_tpdu_rsp.sw2;
                m_evt_cb(&l_evt);
            }
        }
        break;

    default:
        break;
    }

    return err_code;
}

/** @} */ // End tag for iso7816_private_func_fsm


uint32_t iso7816_init (iso7816_init_t *init_param)
{
    return sys_fsm (ISO7816_SYS_FSM_EVT_INIT, init_param, sizeof(iso7816_init_t));
}


uint32_t iso7816_send_TPDU (iso7816_tpdu_t *tpdu)
{
    uint32_t err_code = NRF_SUCCESS;


    if (m_tpdu_state != ISO7816_TPDU_FSM_IDLE) {
        NRF_LOG_PRINTF("iso7816_send_TPDU current state = %d", m_tpdu_state);
        err_code = NRF_ERROR_BUSY;
    } else {
        err_code = tpdu_fsm (ISO7816_TPDU_FSM_EVT_REQ, tpdu, sizeof(iso7816_tpdu_t));
    }
    return err_code;
}


uint32_t iso7816_warm_reset ()
{
    warm_reset_0 ();
    return NRF_SUCCESS;
}


uint32_t iso7816_cleanup ()
{
		nrf_uart_disable();
    return deactivate();
	
}


/** @}
 *  @endcond
 */

