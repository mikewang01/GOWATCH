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
#include "nrf_gpio.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_uart.h"
#include "nrf_delay.h"
#include "sdk_common.h"
#include "app_util_platform.h"
#include <string.h>


/**
 * @defgroup iso7816_phy_uart ISO7816-3 USART-PHY.
 * @{
 */

/**
 * @defgroup iso7816_phy_uart_const Constants
 * @{
 */

#define ISO7816_PHY_WT                      9600            /**< Default WT. */
#define ISO7816_PHY_tg                      1860            /**< Waiting time before clock stop. */
#define ISO7816_PHY_th                      700             /**< Waiting time after clock resume. */

#define ISO7816_PHY_DBG_ENABLE              0               /**< Enable debug signal (for timing verfication). */
#define ISO7816_PHY_DBG_PPI_ENABLE          0               /**< Enable debug signal (PPI)(for timing verfication). */

#define ISO7816_PHY_COLD_RESET_EXTRA_DELAY  4400            /**< Post-cold-reset extra delay. */

#if TIMER2_CONFIG_BIT_WIDTH == TIMER_BITMODE_BITMODE_08Bit
    #error Bit mode not supported.
#elif TIMER2_CONFIG_BIT_WIDTH == TIMER_BITMODE_BITMODE_16Bit
    #define ISO7816_PHY_MAX_WAIT                0xFFFF      /**< maximum waiting time for function wait(), over 16-bit timer. */
#elif TIMER2_CONFIG_BIT_WIDTH == TIMER_BITMODE_BITMODE_24Bit
    #define ISO7816_PHY_MAX_WAIT                0xFFFFFF    /**< maximum waiting time for function wait(), over 24-bit timer. */
#elif TIMER2_CONFIG_BIT_WIDTH == TIMER_BITMODE_BITMODE_32Bit
    #define ISO7816_PHY_MAX_WAIT                0xFFFFFFFF  /**< maximum waiting time for function wait(), over 32-bit timer. */
#else
    #error TIMER2 not enabled.
#endif

#define ISO7816_PHY_TIMER_CH_GENERIC        0               /**< Timer channel ID: General purpose shared timer. */
#define ISO7816_PHY_TIMER_CH_RX_END         1               /**< Timer channel ID: End of Rx. */
#define ISO7816_PHY_TIMER_CH_TX_END         2               /**< Timer channel ID: End of Tx. */

#define ISO7816_PHY_UART_BAUDRATE_RATIO     268             /**< Magic settings in NRF_UARTE->BAUDRATE */
#define ISO7816_PHY_UART_DUMMY_PIN          31              /**< Dummy pin for UART, as a place holder for unused Tx/Rx pins. */
#define ISO7816_PHY_UART_MAX_TX             255             /**< Maximum number of Tx bytes. */

/** @} */ // End tag for iso7816_phy_uart_const


/**
 * @defgroup iso7816_phy_uart_enums Enumerations
 * @{
 */

/**@brief System state machine. */
typedef enum
{
    ISO7816_PHY_SYS_FSM_DISABLED = 0,       /**< System disabled. */
    ISO7816_PHY_SYS_FSM_COLD_RESET,         /**< Cold reset in progress. */
    ISO7816_PHY_SYS_FSM_WARM_RESET,         /**< Warm reset in progress. */
    ISO7816_PHY_SYS_FSM_RUNNING,            /**< System running, ready for Tx/Rx. */
    ISO7816_PHY_SYS_FSM_DEACTIVATE,         /**< Deactivation in progress. */
    ISO7816_PHY_SYS_FSM_CLK_STOP,           /**< Clock stop in progress. */
    ISO7816_PHY_SYS_FSM_IDLE                /**< Clock stopped. */
} iso7816_phy_sys_fsm_t;

/** @} */ // End tag for iso7816_phy_uart_enums


/**
 * @defgroup iso7816_phy_uart_macro Macros
 * @{
 */

#if ISO7816_PHY_DBG_ENABLE
/**@brief Toggle debug pin: dbg_act_pin */
#define DBG_SIGNAL() \
    {\
        if (m_iso7816_phy.dbg_act_pin != ISO7816_PHY_INVALID_PIN_NUMBER)\
        {\
            nrf_gpio_pin_toggle(m_iso7816_phy.dbg_act_pin);\
        }\
    }
#else
#define DBG_SIGNAL()
#endif

/** @} */ // End tag for iso7816_phy_uart_macro


/**
 * @defgroup iso7816_phy_uart_types Types
 * @{
 */

/**@brief Timer callback function */
typedef void (* timer_handler_t)(void);


/**@brief Driver context */
typedef struct
{
    iso7816_phy_evt_handler_t evt_cb;               /**< Event handler. */

    iso7816_phy_sys_fsm_t sys_state;                /**< System state machine. */

    uint8_t vcc_pin;                                /**< Pin number of VCC */
    uint8_t clk_pin;                                /**< Pin number of CLK */
    uint8_t rst_pin;                                /**< Pin number of RST */
    uint8_t io_pin;                                 /**< Pin number of I/O */
    uint8_t dbg_act_pin;                            /**< Pin number of "Activity Debug" */
    uint8_t dbg_ppi_pin;                            /**< Pin number of "PPI Debug" */

    bool tx_enabled;                                /**< Whether I/O is enabled as Tx. */
    bool rx_enabled;                                /**< Whether I/O is enabled as Rx. */

    iso7816_phy_freq_t clk_freq;                    /**< Clock frequency backup. */
    uint16_t           _1_etu;                      /**< ETU, in number of high frequency clocks. */
    uint8_t            idle_lv;                     /**< CLK idle level. */
    uint32_t           baudrate;                    /**< UART baudrate */
		nrf_drv_uart_t   	 iso7816_uart_instance;              /**< High frequency timer */
    nrf_drv_uart_config_t uart_config;              /**< A backup of UART configuration. */

    uint8_t * tx_data;                              /**< Pointer to Tx data. */
    uint16_t  tx_byte_cnt;                          /**< Tx byte counter. */
    uint16_t  tx_req_cnt;                           /**< Requested number of Tx bytes. */

    uint8_t * rx_buff;                              /**< Pointer to RX buffer. */
    uint8_t   rx_buff_len;                          /**< RX buffer size. */
    uint8_t   rx_data_internal;                     /**< Internal RX data, for use in cold / warm reset. */

    bool      rx_queued;                            /**< Indicates that RX has been requested from higher layer. */
    uint8_t * rx_q_buff;                            /**< Pointer to queued RX buffer. */
    uint8_t   rx_q_buff_len;                        /**< Queued RX buffer size. */
    bool      rx_on_going;                          /**< Indicates that RX is on-going. */

    bool      rx_data_rdy;                          /**< Flag indicating whether Rx data is ready or not. */
    uint8_t * rx_data_bak;                          /**< Backup of rx_data->p_data in NRF_DRV_UART_EVT_RX_DONE. */
    uint16_t  rx_bytes_bak;                         /**< Backup of rx_data->bytes in NRF_DRV_UART_EVT_RX_DONE. */

    uint16_t Fi;                                    /**< F proposed by SIM card. */
    uint8_t  Di;                                    /**< D proposed by SIM card. */
    uint32_t WT;                                    /**< Waiting time: default 9600. */
    uint8_t  N;                                     /**< Extra guard time (N) proposed by SIM card. */

    nrf_drv_timer_t   high_freq_timer;              /**< High frequency timer */
    nrf_ppi_channel_t high_freq_ppi;                /**< High frequency PPI */

    nrf_drv_timer_t   low_freq_timer;               /**< Low frequency timer */
    timer_handler_t   low_freq_cb;                  /**< Low frequency callback function */

    nrf_ppi_channel_t io_ppi_rx_end;                /**< PPI: UART RX_END ==> low_freq_timer start */
    nrf_ppi_channel_t io_ppi_tx_end_suspend;        /**< PPI: UART TX_END ==> suspend UART */
    nrf_ppi_channel_t io_ppi_tx_end_timer_start;    /**< PPI: UART TX_END ==> timer start */
    nrf_ppi_channel_t io_ppi_timer_resume;          /**< PPI: timer ==> resume UART */

    uint32_t last_wait;                             /**< Last known waiting time */
    uint32_t total_wait;                            /**< Total wait time */
    uint32_t max_wait;                              /**< Total wait time */

#if ISO7816_PHY_DBG_PPI_ENABLE
    nrf_ppi_channel_t io_dbg_ppi_rx_end;            /**< Debug PPI */
    nrf_ppi_channel_t io_dbg_ppi_rx_end_timer;      /**< Debug PPI */
    nrf_ppi_channel_t io_dbg_ppi_tx_end;            /**< Debug PPI */
    nrf_ppi_channel_t io_dbg_ppi_tx_end_timer;      /**< Debug PPI */
#endif
} iso7816_phy_t;

/** @} */ // End tag for iso7816_phy_uart_types


static iso7816_phy_t     m_iso7816_phy;             /**< Driver context. */

static nrf_drv_timer_t   m_high_freq_timer = NRF_DRV_TIMER_INSTANCE(1); /**< High frequency timer */
static nrf_drv_timer_t   m_low_freq_timer  = NRF_DRV_TIMER_INSTANCE(2); /**< Low frequency timer */
static nrf_drv_uart_t		 m_7816_uart_instance = NRF_DRV_UART_INSTANCE(0);

/**
 * @defgroup iso7816_phy_uart_lut Look-up tables
 * @{
 */

/**@brief Compare configuration register lookup table
 * @note  Index is of data type @ref iso7816_phy_freq_t
 */
static const uint8_t m_cc_lut [ISO7816_PHY_FREQ_END]
    = {
        8,  // 1MHz
        4,  // 2MHz
        2   // 4MHz
    };


/**@brief Timer frequency lookup table
 * @note  For use in initialization and clock reconfiguration only */
static const nrf_timer_frequency_t m_timer_freq_lut [ISO7816_PHY_FREQ_END]
    = {
        NRF_TIMER_FREQ_1MHz,  // 1MHz
        NRF_TIMER_FREQ_2MHz,  // 2MHz
        NRF_TIMER_FREQ_4MHz   // 4MHz
    };

/** @} */ // End tag for iso7816_phy_uart_lut


/** @defgroup iso7816_phy_uart_func Functions */

static uint32_t clk_init    (iso7816_phy_freq_t clk_freq);
static void     clk_enable  (void);
static void     clk_disable (void);
static uint32_t clk_cleanup (void);

static void     io_set_input (void);
static void     io_disable   (void);
static uint32_t io_get_value (void);

static void     gpio_init   (void);
static uint32_t dbg_init    (void);
static uint32_t dbg_cleanup (void);

static void     high_freq_timer_handler (nrf_timer_event_t event_type, void * p_context);
static uint32_t high_freq_timer_init    (void);
static void     high_freq_timer_cleanup (void);

static void     long_timer_handler     (void);
static void     low_freq_timer_handler (nrf_timer_event_t event_type, void * p_context);
static uint32_t low_freq_timer_init    (void);
static void     low_freq_timer_clear   (void);
static void     low_freq_timer_cleanup (void);

static uint32_t timers_init    (void);
static void     timers_cleanup (void);

static void     on_uart_evt    (nrf_drv_uart_event_t * p_evt, void * p_context);
static uint32_t uart_init      (void);
static uint32_t uart_enable_rx (void);
static uint32_t uart_enable_tx (void);
static uint32_t uart_rx        (void);
static uint32_t uart_disable   (void);

static void sleep          (uint32_t n, timer_handler_t cb);
static void sleep_and_stop (uint32_t n, timer_handler_t cb);

static void cold_reset_cont (void);
static void warm_reset_cont (void);
static void clk_stop_cont   (void);
static void clk_resume_cont (void);


/**
 * @defgroup iso7816_phy_uart_func_clk_signal CLK signal
 * @ingroup  iso7816_phy_uart_func
 * @{
 */

/**@brief Initialize CLK */
static uint32_t clk_init (iso7816_phy_freq_t clk_freq)
{
    uint32_t err_code;
    uint32_t evt_addr, task_addr;
    nrf_drv_gpiote_out_config_t gpiote_config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);

    /* Initialize GPIOTE channel for CLK */
    err_code = nrf_drv_gpiote_out_init (m_iso7816_phy.clk_pin, &gpiote_config);
    VERIFY_SUCCESS(err_code);
    
    /* Set up CLK Timer COMPARE #0 */
    nrf_drv_timer_extended_compare (&m_iso7816_phy.high_freq_timer,
                                    (nrf_timer_cc_channel_t)0,
                                    m_cc_lut [clk_freq],
                                    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                    false);

    /* Connect PPI and enable */
    err_code = nrf_drv_ppi_channel_alloc (&m_iso7816_phy.high_freq_ppi);
    VERIFY_SUCCESS(err_code);
    
    evt_addr  = nrf_drv_timer_event_address_get (&m_iso7816_phy.high_freq_timer, NRF_TIMER_EVENT_COMPARE0);
    task_addr = nrf_drv_gpiote_out_task_addr_get (m_iso7816_phy.clk_pin);
    err_code  = nrf_drv_ppi_channel_assign (m_iso7816_phy.high_freq_ppi, evt_addr, task_addr);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_drv_ppi_channel_enable (m_iso7816_phy.high_freq_ppi);
    VERIFY_SUCCESS(err_code);

    /* Enable GPIOTE for CLK signal */
    nrf_drv_gpiote_out_task_enable (m_iso7816_phy.clk_pin);

    return NRF_SUCCESS;
}


/**@brief Enable CLK output */
static void clk_enable ()
{
    nrf_drv_timer_enable (&m_iso7816_phy.high_freq_timer);
}


/**@brief Disable CLK output */
static void clk_disable ()
{
    nrf_drv_timer_disable (&m_iso7816_phy.high_freq_timer);
}


/**@brief Cleanup CLK */
static uint32_t clk_cleanup ()
{
    nrf_drv_timer_disable (&m_iso7816_phy.high_freq_timer);
    nrf_drv_gpiote_out_uninit (m_iso7816_phy.clk_pin);
    nrf_drv_ppi_channel_free (m_iso7816_phy.high_freq_ppi);
	  nrf_drv_ppi_channel_free (m_iso7816_phy.io_ppi_rx_end);
	  nrf_drv_ppi_channel_free (m_iso7816_phy.io_ppi_timer_resume);
  	nrf_drv_ppi_channel_free (m_iso7816_phy.io_ppi_tx_end_suspend);
	  nrf_drv_ppi_channel_free (m_iso7816_phy.io_ppi_tx_end_timer_start);
    return NRF_SUCCESS;
}

/** @} */ // End tag for iso7816_phy_uart_func_clk_signal


/**
 * @defgroup iso7816_phy_uart_func_io_signal I/O signal.
 * @ingroup  iso7816_phy_uart_func
 * @{
 */

/**@brief Set I/O pin as input */
static void io_set_input ()
{
   // nrf_gpio_cfg_input (m_iso7816_phy.io_pin, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg(m_iso7816_phy.io_pin, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
}


/**@brief Disable I/O pin (set input and pull low) */
static void io_disable ()
{
    nrf_gpio_cfg_input (m_iso7816_phy.io_pin, NRF_GPIO_PIN_PULLDOWN);
}


/**@brief Get I/O value */
static uint32_t io_get_value ()
{
    return nrf_gpio_pin_read(m_iso7816_phy.io_pin);
}

/** @} */ // End tag for iso7816_phy_uart_func_io_signal


/**
 * @defgroup iso7816_phy_uart_func_others Others
 * @ingroup  iso7816_phy_uart_func
 * @{
 */


/**@brief Configure GPIOs */
static void gpio_init ()
{
    /* Vcc */
    nrf_gpio_cfg_output (m_iso7816_phy.vcc_pin);
    nrf_gpio_pin_clear (m_iso7816_phy.vcc_pin);
    
    /* CLK */
	 nrf_gpio_cfg(m_iso7816_phy.clk_pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, GPIO_PIN_CNF_PULL_Pullup,NRF_GPIO_PIN_S0S1,NRF_GPIO_PIN_NOSENSE);
   // nrf_gpio_cfg_output (m_iso7816_phy.clk_pin);
    nrf_gpio_pin_clear (m_iso7816_phy.clk_pin);
    
    /* RST */
    nrf_gpio_cfg_output (m_iso7816_phy.rst_pin);
    nrf_gpio_pin_clear (m_iso7816_phy.rst_pin);
    
    // I/O
    io_set_input ();
}


/**@brief Configure GPIO debug pins */
static uint32_t dbg_init ()
{
    uint32_t err_code = NRF_SUCCESS;

#if ISO7816_PHY_DBG_ENABLE
    /* Debug pin */
    if (m_iso7816_phy.dbg_act_pin != ISO7816_PHY_INVALID_PIN_NUMBER)
    {
        nrf_gpio_cfg_output (m_iso7816_phy.dbg_act_pin);
        nrf_gpio_pin_clear  (m_iso7816_phy.dbg_act_pin);
    }
#endif

#if ISO7816_PHY_DBG_PPI_ENABLE
    uint32_t evt_addr, task_addr;

    if (m_iso7816_phy.dbg_ppi_pin != ISO7816_PHY_INVALID_PIN_NUMBER)
    {
        nrf_gpio_cfg_output (m_iso7816_phy.dbg_ppi_pin);
        nrf_gpio_pin_clear  (m_iso7816_phy.dbg_ppi_pin);

        /* Initialize GPIOTE channel for I/O debug */

        //
        // Debug I/O: RX_END from UART
        ////////////////////////////////////////////////////////
        nrf_drv_gpiote_out_config_t gpiote_config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);
        err_code = nrf_drv_gpiote_out_init (m_iso7816_phy.dbg_ppi_pin, &gpiote_config);
        VERIFY_SUCCESS(err_code);

        err_code = nrf_drv_ppi_channel_alloc (&m_iso7816_phy.io_dbg_ppi_rx_end);
        VERIFY_SUCCESS(err_code);

        evt_addr  = nrf_uarte_event_address_get(NRF_UARTE0, NRF_UARTE_EVENT_ENDRX);
        task_addr = nrf_drv_gpiote_out_task_addr_get (m_iso7816_phy.dbg_ppi_pin);
        err_code  = nrf_drv_ppi_channel_assign (m_iso7816_phy.io_dbg_ppi_rx_end, evt_addr, task_addr);
        VERIFY_SUCCESS(err_code);

        err_code = nrf_drv_ppi_channel_enable (m_iso7816_phy.io_dbg_ppi_rx_end);
        VERIFY_SUCCESS(err_code);

        //
        // Debug I/O: Timer expiry after RX_END from UART
        ////////////////////////////////////////////////////////
        err_code = nrf_drv_ppi_channel_alloc (&m_iso7816_phy.io_dbg_ppi_rx_end_timer);
        VERIFY_SUCCESS(err_code);

        evt_addr  = nrf_drv_timer_event_address_get (&m_iso7816_phy.low_freq_timer, NRF_TIMER_EVENT_COMPARE1);
        task_addr = nrf_drv_gpiote_out_task_addr_get (m_iso7816_phy.dbg_ppi_pin);
        err_code  = nrf_drv_ppi_channel_assign (m_iso7816_phy.io_dbg_ppi_rx_end_timer, evt_addr, task_addr);
        VERIFY_SUCCESS(err_code);

        err_code = nrf_drv_ppi_channel_enable (m_iso7816_phy.io_dbg_ppi_rx_end_timer);
        VERIFY_SUCCESS(err_code);

        //
        // Debug I/O: TX_END from UART
        ////////////////////////////////////////////////////////
        err_code = nrf_drv_ppi_channel_alloc (&m_iso7816_phy.io_dbg_ppi_tx_end);
        VERIFY_SUCCESS(err_code);

        evt_addr  = nrf_uart_event_address_get(NRF_UART0, NRF_UART_EVENT_TXDRDY);
        task_addr = nrf_drv_gpiote_out_task_addr_get (m_iso7816_phy.dbg_ppi_pin);
        err_code  = nrf_drv_ppi_channel_assign (m_iso7816_phy.io_dbg_ppi_tx_end, evt_addr, task_addr);
        VERIFY_SUCCESS(err_code);

        err_code = nrf_drv_ppi_channel_enable (m_iso7816_phy.io_dbg_ppi_tx_end);
        VERIFY_SUCCESS(err_code);

        //
        // Debug I/O: Timer expiry after TX_END from UART
        ////////////////////////////////////////////////////////
        err_code = nrf_drv_ppi_channel_alloc (&m_iso7816_phy.io_dbg_ppi_tx_end_timer);
        VERIFY_SUCCESS(err_code);

        evt_addr  = nrf_drv_timer_event_address_get (&m_iso7816_phy.low_freq_timer, NRF_TIMER_EVENT_COMPARE2);
        task_addr = nrf_drv_gpiote_out_task_addr_get (m_iso7816_phy.dbg_ppi_pin);
        err_code  = nrf_drv_ppi_channel_assign (m_iso7816_phy.io_dbg_ppi_tx_end_timer, evt_addr, task_addr);
        VERIFY_SUCCESS(err_code);

        err_code = nrf_drv_ppi_channel_enable (m_iso7816_phy.io_dbg_ppi_tx_end_timer);
        VERIFY_SUCCESS(err_code);

        /* Enable GPIOTE for debug pins */
        nrf_drv_gpiote_out_task_enable (m_iso7816_phy.dbg_ppi_pin);
    }
#endif
		
    return err_code;
}


/**@brief DBG signal cleanup. */
static uint32_t dbg_cleanup ()
{
    uint32_t err_code = NRF_SUCCESS;

#if ISO7816_PHY_DBG_ENABLE
    /* Debug pin */
    if (m_iso7816_phy.dbg_act_pin != ISO7816_PHY_INVALID_PIN_NUMBER)
    {
        nrf_gpio_pin_clear  (m_iso7816_phy.dbg_act_pin);
        nrf_gpio_input_disconnect (m_iso7816_phy.dbg_act_pin);
    }
#endif

#if ISO7816_PHY_DBG_PPI_ENABLE
    if (m_iso7816_phy.dbg_ppi_pin != ISO7816_PHY_INVALID_PIN_NUMBER)
    {
        nrf_drv_gpiote_out_task_disable (m_iso7816_phy.dbg_ppi_pin);

        //
        // Debug I/O: RX_END from UART
        ////////////////////////////////////////////////////////
        err_code = nrf_drv_ppi_channel_disable (m_iso7816_phy.io_dbg_ppi_rx_end);
        VERIFY_SUCCESS(err_code);

        err_code = nrf_drv_ppi_channel_free (m_iso7816_phy.io_dbg_ppi_rx_end);
        VERIFY_SUCCESS(err_code);

        nrf_drv_gpiote_out_uninit (m_iso7816_phy.dbg_ppi_pin);

        //
        // Debug I/O: Timer expiry after RX_END from UART
        ////////////////////////////////////////////////////////
        err_code = nrf_drv_ppi_channel_disable (m_iso7816_phy.io_dbg_ppi_rx_end_timer);
        VERIFY_SUCCESS(err_code);

        err_code = nrf_drv_ppi_channel_free (m_iso7816_phy.io_dbg_ppi_rx_end_timer);
        VERIFY_SUCCESS(err_code);

        //
        // Debug I/O: TX_END from UART
        ////////////////////////////////////////////////////////
        err_code = nrf_drv_ppi_channel_disable (m_iso7816_phy.io_dbg_ppi_tx_end);
        VERIFY_SUCCESS(err_code);

        err_code = nrf_drv_ppi_channel_free (m_iso7816_phy.io_dbg_ppi_tx_end);
        VERIFY_SUCCESS(err_code);

        //
        // Debug I/O: Timer expiry after TX_END from UART
        ////////////////////////////////////////////////////////
        err_code = nrf_drv_ppi_channel_disable (m_iso7816_phy.io_dbg_ppi_tx_end_timer);
        VERIFY_SUCCESS(err_code);

        err_code = nrf_drv_ppi_channel_free (m_iso7816_phy.io_dbg_ppi_tx_end_timer);
        VERIFY_SUCCESS(err_code);

        //
        // Clear debug pin
        //
        nrf_gpio_pin_clear  (m_iso7816_phy.dbg_ppi_pin);
        nrf_gpio_input_disconnect (m_iso7816_phy.dbg_ppi_pin);
    }
#endif
		
    return err_code;
}

/** @} */ // End tag for iso7816_phy_uart_func_others


/**
 * @defgroup iso7816_phy_uart_func_timers Timers
 * @ingroup  iso7816_phy_uart_func
 * @{
 */

/**
 * @defgroup iso7816_phy_uart_func_high_freq_timer High frequency timer.
 * @{
 */

/**@brief Dummy timer handler */
static void high_freq_timer_handler (nrf_timer_event_t event_type, void * p_context)
{
    ;
}


/**@brief High frequency timer initialization */
static uint32_t high_freq_timer_init ()
{
    uint32_t err_code = NRF_SUCCESS;

    err_code = nrf_drv_timer_init(&m_iso7816_phy.high_freq_timer, NULL, high_freq_timer_handler);
    VERIFY_SUCCESS(err_code);

    *(uint32_t *)0x40009C0C = 1;
    return err_code;
}


/**@brief High frequency timer clean-up */
static void high_freq_timer_cleanup ()
{
    *(uint32_t *)0x40009C0C = 1;
    nrf_drv_timer_uninit(&m_iso7816_phy.high_freq_timer);
}

/** @} */ // End tag for iso7816_phy_uart_func_high_freq_timer


/**
 * @defgroup iso7816_phy_uart_func_low_freq_timer Low frequency timer.
 * @{
 */

/**@brief Long timer handler */
static void long_timer_handler ()
{
    iso7816_phy_evt_t evt;
    uint32_t t;

    m_iso7816_phy.total_wait += m_iso7816_phy.last_wait;

    if (m_iso7816_phy.total_wait >= m_iso7816_phy.max_wait)
    {
        /* Timed out */
        evt.evt_id = ISO7816_PHY_EVT_TIMEOUT;
        m_iso7816_phy.evt_cb(&evt);
        DBG_SIGNAL();
    }
    else
    {
        t = m_iso7816_phy.max_wait - m_iso7816_phy.total_wait;

        if (t > ISO7816_PHY_MAX_WAIT)
        {
            sleep (ISO7816_PHY_MAX_WAIT, long_timer_handler);
        }
        else
        {
            sleep_and_stop (t, long_timer_handler);
        }
    }
}

/**@brief Low frequency timer handler */
static void low_freq_timer_handler (nrf_timer_event_t event_type, void * p_context)
{    
    uint32_t err_code = NRF_SUCCESS;
    iso7816_phy_evt_t l_evt;

    DBG_SIGNAL();
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            m_iso7816_phy.low_freq_cb ();
            break;

        case NRF_TIMER_EVENT_COMPARE1:
            if (m_iso7816_phy.rx_data_rdy)
            {
                m_iso7816_phy.rx_data_rdy = false;

                /* Post-RX timer ends, then restart WT timer. */
                err_code = iso7816_phy_timer_start (m_iso7816_phy.WT * m_iso7816_phy.Fi);
                VERIFY_SUCCESS_VOID(err_code);

                l_evt.evt_id           = ISO7816_PHY_EVT_RX_DATA;
                l_evt.evt.rx_data.data = m_iso7816_phy.rx_data_bak;
                l_evt.evt.rx_data.len  = m_iso7816_phy.rx_bytes_bak;
                m_iso7816_phy.evt_cb(&l_evt);
            }
            break;

        default:
            break;
    }
}


/**@brief Low frequency timer initialization. */
static uint32_t low_freq_timer_init ()
{
    uint32_t err_code = NRF_SUCCESS;
    uint32_t evt_addr, task_addr;
    nrf_drv_timer_config_t config = NRF_DRV_TIMER_DEFAULT_CONFIG;

    config.frequency = m_timer_freq_lut[m_iso7816_phy.clk_freq];

    err_code = nrf_drv_timer_init(&m_iso7816_phy.low_freq_timer, &config, low_freq_timer_handler);
    VERIFY_SUCCESS(err_code);

    *(uint32_t *)0x4000AC0C = 1;

    //
    // PPI: UART RX_END to low_freq_timer start.
    //
    err_code = nrf_drv_ppi_channel_alloc (&m_iso7816_phy.io_ppi_rx_end);
    VERIFY_SUCCESS(err_code);

    evt_addr  = nrf_uarte_event_address_get(NRF_UARTE0, NRF_UARTE_EVENT_ENDRX);
    task_addr = nrf_drv_timer_task_address_get (&m_iso7816_phy.low_freq_timer, NRF_TIMER_TASK_START);
    err_code  = nrf_drv_ppi_channel_assign (m_iso7816_phy.io_ppi_rx_end, evt_addr, task_addr);
    VERIFY_SUCCESS(err_code);

    task_addr = nrf_drv_timer_task_address_get (&m_iso7816_phy.low_freq_timer, NRF_TIMER_TASK_CLEAR);
    err_code  = nrf_drv_ppi_channel_fork_assign (m_iso7816_phy.io_ppi_rx_end, task_addr);
    VERIFY_SUCCESS(err_code);

    //
    // PPI: UART TX_END to UART suspend.
    //
    err_code = nrf_drv_ppi_channel_alloc (&m_iso7816_phy.io_ppi_tx_end_suspend);
    VERIFY_SUCCESS(err_code);

    evt_addr  = nrf_uart_event_address_get(NRF_UART0, NRF_UART_EVENT_TXDRDY);   // Legacy UART w/o EasyDMA
    task_addr = (uint32_t) &NRF_UARTE0->RESERVED0[3];                           // SUSPEND
    err_code  = nrf_drv_ppi_channel_assign (m_iso7816_phy.io_ppi_tx_end_suspend, evt_addr, task_addr);
    VERIFY_SUCCESS(err_code);

    //
    // PPI: UART TX_END to low_freq_timer start.
    //
    err_code = nrf_drv_ppi_channel_alloc (&m_iso7816_phy.io_ppi_tx_end_timer_start);
    VERIFY_SUCCESS(err_code);

    evt_addr  = nrf_uart_event_address_get(NRF_UART0, NRF_UART_EVENT_TXDRDY);   // Legacy UART w/o EasyDMA
    task_addr = nrf_drv_timer_task_address_get (&m_iso7816_phy.low_freq_timer, NRF_TIMER_TASK_START);
    err_code  = nrf_drv_ppi_channel_assign (m_iso7816_phy.io_ppi_tx_end_timer_start, evt_addr, task_addr);
    VERIFY_SUCCESS(err_code);

    task_addr = nrf_drv_timer_task_address_get (&m_iso7816_phy.low_freq_timer, NRF_TIMER_TASK_CLEAR);
    err_code  = nrf_drv_ppi_channel_fork_assign (m_iso7816_phy.io_ppi_tx_end_timer_start, task_addr);
    VERIFY_SUCCESS(err_code);

    //
    // PPI: Timer EVENTS_COMPARE[2] to UART RESUME.
    //
    err_code = nrf_drv_ppi_channel_alloc (&m_iso7816_phy.io_ppi_timer_resume);
    VERIFY_SUCCESS(err_code);

    evt_addr  = nrf_drv_timer_event_address_get(&m_iso7816_phy.low_freq_timer, NRF_TIMER_EVENT_COMPARE2);
    task_addr = nrf_drv_timer_task_address_get (&m_iso7816_phy.low_freq_timer, NRF_TIMER_TASK_SHUTDOWN);
    err_code  = nrf_drv_ppi_channel_assign (m_iso7816_phy.io_ppi_timer_resume, evt_addr, task_addr);
    VERIFY_SUCCESS(err_code);

    task_addr = (uint32_t) &NRF_UARTE0->RESERVED0[4];    // RESUME
    err_code  = nrf_drv_ppi_channel_fork_assign (m_iso7816_phy.io_ppi_timer_resume, task_addr);
    VERIFY_SUCCESS(err_code);

    return err_code;
}


/**@brief Clear low_frequency timer */
static void low_freq_timer_clear ()
{
	
	
    nrf_drv_timer_clear(&m_iso7816_phy.low_freq_timer);
}


/**@brief Low frequency timer clean-up */
static void low_freq_timer_cleanup ()
{
    nrf_drv_timer_disable (&m_iso7816_phy.low_freq_timer);

    *(uint32_t *)0x4000AC0C = 1;
    nrf_drv_timer_uninit(&m_iso7816_phy.low_freq_timer);
}

/** @} */ // End tag for iso7816_phy_uart_func_low_freq_timer


/**@brief Initialize timers */
static uint32_t timers_init (void)
{
    uint32_t err_code = NRF_SUCCESS;

    err_code = high_freq_timer_init ();
    VERIFY_SUCCESS(err_code);

    err_code = low_freq_timer_init ();
    VERIFY_SUCCESS(err_code);

    return err_code;
}


/**@brief Uninit timers */
static void timers_cleanup ()
{
    low_freq_timer_cleanup ();
    high_freq_timer_cleanup ();
}


/**@brief Sleep for n cycles, with background task running */
static void sleep (uint32_t n, timer_handler_t cb)
{    
    m_iso7816_phy.last_wait   = n;
    m_iso7816_phy.low_freq_cb = cb;
    
    m_iso7816_phy.low_freq_timer.p_reg->INTENCLR = NRF_TIMER_INT_COMPARE0_MASK;

    nrf_drv_timer_extended_compare (&m_iso7816_phy.low_freq_timer,
                                    (nrf_timer_cc_channel_t)0,
                                    m_iso7816_phy.last_wait,
                                    (nrf_timer_short_mask_t) NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                    true);
    nrf_drv_timer_resume (&m_iso7816_phy.low_freq_timer);
}


/**@brief Sleep for n cycles, with background task running. Stop afterwards. */
static void sleep_and_stop (uint32_t n, timer_handler_t cb)
{
    m_iso7816_phy.last_wait   = n;
    m_iso7816_phy.low_freq_cb = cb;
    
    m_iso7816_phy.low_freq_timer.p_reg->INTENCLR = NRF_TIMER_INT_COMPARE0_MASK;

    nrf_drv_timer_extended_compare (&m_iso7816_phy.low_freq_timer,
                                    (nrf_timer_cc_channel_t)0,
                                    m_iso7816_phy.last_wait,
                                    (nrf_timer_short_mask_t) (NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK | NRF_TIMER_SHORT_COMPARE0_STOP_MASK),
                                    true);
    nrf_drv_timer_resume (&m_iso7816_phy.low_freq_timer);
}

/** @} */ // End tag for iso7816_phy_uart_func_timers


/**
 * @defgroup iso7816_phy_uart_uart Functions for UART handling.
 * @ingroup  iso7816_phy_uart_uart
 * @{
 */

/**@brief Function for handling events from @ref nrf_drv_uart */
static void on_uart_evt (nrf_drv_uart_event_t * p_evt, void * p_context)
{
    uint32_t          err_code = NRF_SUCCESS;
    iso7816_phy_evt_t l_evt;
    uint16_t          bytes_left;

    switch (p_evt->type)
    {
        case NRF_DRV_UART_EVT_TX_DONE:
            m_iso7816_phy.tx_byte_cnt += p_evt->data.rxtx.bytes;
            bytes_left                 = m_iso7816_phy.tx_req_cnt - m_iso7816_phy.tx_byte_cnt;

            /* Check number of bytes left. */
            if (bytes_left > 0)
            {
                /* Continue sending. */
                uint8_t bytes_to_send = (bytes_left > ISO7816_PHY_UART_MAX_TX)? ISO7816_PHY_UART_MAX_TX: bytes_left;

                err_code = nrf_drv_uart_tx ((const nrf_drv_uart_t *)&(m_iso7816_phy.iso7816_uart_instance), &m_iso7816_phy.tx_data[m_iso7816_phy.tx_byte_cnt], bytes_to_send);
                VERIFY_SUCCESS_VOID(err_code);
            }
            else
            {
                m_iso7816_phy.tx_byte_cnt = 0;

                /* Avoid shuting down WT timer. */
                err_code = nrf_drv_ppi_channel_disable (m_iso7816_phy.io_ppi_timer_resume);
                VERIFY_SUCCESS_VOID(err_code);

                /* Disable post-TX timer. */
                nrf_drv_timer_extended_compare (&m_iso7816_phy.low_freq_timer,
                                                (nrf_timer_cc_channel_t) ISO7816_PHY_TIMER_CH_TX_END,
                                                0, (nrf_timer_short_mask_t)0, false);

                /* Start WT timer. */
                err_code = iso7816_phy_timer_start (m_iso7816_phy.WT * m_iso7816_phy.Fi);
                VERIFY_SUCCESS_VOID(err_code);

                /* Notify upper layer. */
                l_evt.evt_id          = ISO7816_PHY_EVT_TX_DONE;
                l_evt.evt.tx_done.len = m_iso7816_phy.tx_req_cnt;
                m_iso7816_phy.evt_cb(&l_evt);

            }
            break;

        case NRF_DRV_UART_EVT_RX_DONE:
            DBG_SIGNAL();

            if (m_iso7816_phy.rx_on_going)
            {
                m_iso7816_phy.rx_on_going = false;
            }

            /* Events will be delayed to low_freq_timer_handler(). */
            m_iso7816_phy.rx_data_rdy  = true;
            m_iso7816_phy.rx_data_bak  = p_evt->data.rxtx.p_data;
            m_iso7816_phy.rx_bytes_bak = p_evt->data.rxtx.bytes;

            if (!m_iso7816_phy.rx_queued)
            {
                // receive 1 byte
                m_iso7816_phy.rx_buff     = &m_iso7816_phy.rx_data_internal;
                m_iso7816_phy.rx_buff_len = 1;
            }
            else
            {
                m_iso7816_phy.rx_queued = false;
                memcpy (m_iso7816_phy.rx_q_buff, p_evt->data.rxtx.p_data, p_evt->data.rxtx.bytes);

                m_iso7816_phy.rx_buff     = &m_iso7816_phy.rx_q_buff[p_evt->data.rxtx.bytes];
                m_iso7816_phy.rx_buff_len = m_iso7816_phy.rx_q_buff_len - p_evt->data.rxtx.bytes;
            }
            uart_rx();
            break;

        case NRF_DRV_UART_EVT_ERROR:
            DBG_SIGNAL();
            DBG_SIGNAL();
            DBG_SIGNAL();
            break;

        default:
            break;
    }
}


/**@brief Funciton for initializing @ref nrf_drv_uart. */
static uint32_t uart_init (void)
{
    uint32_t err_code = NRF_SUCCESS;
    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;

    uart_config.pseltxd      = ISO7816_PHY_UART_DUMMY_PIN;
    uart_config.pselrxd      = ISO7816_PHY_UART_DUMMY_PIN;
    uart_config.p_context    = &m_iso7816_phy;
    uart_config.parity       = NRF_UART_PARITY_INCLUDED;
    uart_config.baudrate     = (nrf_uart_baudrate_t)m_iso7816_phy.baudrate;
    uart_config.use_easy_dma = true;

    /* Backup config, initialization will happen right after cold reset is requested. */
    memcpy (&m_iso7816_phy.uart_config, &uart_config, sizeof(nrf_drv_uart_config_t));
    return err_code;
}


/**@brief Function for configuring io_pin as rx and then enable UART RX */
static uint32_t uart_enable_rx (void)
{
    uint32_t err_code = NRF_SUCCESS;

    if (m_iso7816_phy.rx_enabled)
    {
        /* do nothing if Rx is already enabled. */
        return err_code;
    }
    else if (m_iso7816_phy.tx_enabled)
    {
        /* Need to disable UART first if Rx is active. */
        err_code = uart_disable ();
        VERIFY_SUCCESS(err_code);
    }

    m_iso7816_phy.uart_config.pseltxd = ISO7816_PHY_UART_DUMMY_PIN;
    m_iso7816_phy.uart_config.pselrxd = m_iso7816_phy.io_pin;

    err_code = nrf_drv_uart_init((const nrf_drv_uart_t *)&(m_iso7816_phy.iso7816_uart_instance), &m_iso7816_phy.uart_config, on_uart_evt);
		Y_SPRINTF("[uart]: error code %d", err_code);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_drv_ppi_channel_enable (m_iso7816_phy.io_ppi_rx_end);
    VERIFY_SUCCESS(err_code);

    m_iso7816_phy.rx_enabled = true;
    return err_code;
}


/**@brief Function for configuring io_pin as tx and then enable UART TX */
static uint32_t uart_enable_tx (void)
{
    uint32_t err_code = NRF_SUCCESS;

    if (m_iso7816_phy.tx_enabled)
    {
        /* do nothing if Tx is already enabled. */
        return err_code;
    }
    else if (m_iso7816_phy.rx_enabled)
    {
        /* Need to disable UART first if Rx is active. */
        err_code = uart_disable ();
        VERIFY_SUCCESS(err_code);
    }

    m_iso7816_phy.uart_config.pseltxd = m_iso7816_phy.io_pin;
    m_iso7816_phy.uart_config.pselrxd = ISO7816_PHY_UART_DUMMY_PIN;

    err_code = nrf_drv_uart_init((const nrf_drv_uart_t *)&(m_iso7816_phy.iso7816_uart_instance), &m_iso7816_phy.uart_config, on_uart_evt);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_drv_ppi_channel_enable (m_iso7816_phy.io_ppi_tx_end_suspend);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_drv_ppi_channel_enable (m_iso7816_phy.io_ppi_tx_end_timer_start);
    VERIFY_SUCCESS(err_code);

    m_iso7816_phy.tx_enabled = true;
    m_iso7816_phy.rx_on_going = false;
    return err_code;
}


/**@brief Function for starting UART RX (1-byte) */
static uint32_t uart_rx (void)
{
    uint32_t err_code = NRF_SUCCESS;

    err_code =  nrf_drv_uart_rx((const nrf_drv_uart_t *)&(m_iso7816_phy.iso7816_uart_instance), m_iso7816_phy.rx_buff, m_iso7816_phy.rx_buff_len);
    VERIFY_SUCCESS(err_code);

    m_iso7816_phy.rx_on_going = true;

    /* Enable post-RX timer */
    // plus ~0.5 ETU delay from UART RX_END
    nrf_drv_timer_extended_compare (&m_iso7816_phy.low_freq_timer,
                                    (nrf_timer_cc_channel_t) ISO7816_PHY_TIMER_CH_RX_END,
                                    (1 + m_iso7816_phy.N) * m_iso7816_phy._1_etu + (m_iso7816_phy._1_etu >> 1),
                                    (nrf_timer_short_mask_t)0,
                                    true);
    return err_code;
}

/**@brief Function for disabling UART. */
static uint32_t uart_disable (void)
{
    uint32_t err_code = NRF_SUCCESS;

    /* Disable all post-Tx & post-Rx PPI channels. */
    err_code = nrf_drv_ppi_channel_disable (m_iso7816_phy.io_ppi_rx_end);
    //VERIFY_SUCCESS(err_code);

    err_code = nrf_drv_ppi_channel_disable (m_iso7816_phy.io_ppi_tx_end_suspend);
    //VERIFY_SUCCESS(err_code);

    err_code = nrf_drv_ppi_channel_disable (m_iso7816_phy.io_ppi_tx_end_timer_start);
    //VERIFY_SUCCESS(err_code);

    err_code = nrf_drv_ppi_channel_disable (m_iso7816_phy.io_ppi_timer_resume);
    //VERIFY_SUCCESS(err_code);

    /* Disable post RX timer interrupt. */
    nrf_drv_timer_extended_compare (&m_iso7816_phy.low_freq_timer, (nrf_timer_cc_channel_t) ISO7816_PHY_TIMER_CH_RX_END,
                                    0, (nrf_timer_short_mask_t)0, false);

		Y_SPRINTF("[phy_uart]: uninit");
    nrf_drv_uart_uninit((const nrf_drv_uart_t *)&(m_iso7816_phy.iso7816_uart_instance));

    m_iso7816_phy.tx_enabled  = false;
    m_iso7816_phy.rx_enabled  = false;
    m_iso7816_phy.rx_on_going = false;
    return err_code;
}

/** @} */ // End tag for iso7816_phy_uart_uart


/**@brief 2nd phase of cold reset */
static void cold_reset_cont (void)
{
    iso7816_phy_evt_t evt;

    if (m_iso7816_phy.sys_state != ISO7816_PHY_SYS_FSM_COLD_RESET)
    {
        return;
    }

    if (io_get_value() != 1)
    {
        m_iso7816_phy.sys_state = ISO7816_PHY_SYS_FSM_DISABLED;

        evt.evt_id = ISO7816_PHY_EVT_RST_TIMEOUT;
        m_iso7816_phy.evt_cb(&evt);
    }
    else
    {
        nrf_gpio_pin_set (m_iso7816_phy.rst_pin);   // RST=1
        DBG_SIGNAL();

        m_iso7816_phy.rx_buff     = &m_iso7816_phy.rx_data_internal;
        m_iso7816_phy.rx_buff_len = 1;

        uart_rx ();

        DBG_SIGNAL();
        m_iso7816_phy.total_wait = 0;

        m_iso7816_phy.sys_state = ISO7816_PHY_SYS_FSM_RUNNING;

        evt.evt_id = ISO7816_PHY_EVT_COLD_RESET;
        m_iso7816_phy.evt_cb(&evt);
    }
}


/**@brief 2nd phase of warm reset */
static void warm_reset_cont (void)
{
    uint32_t          err_code = NRF_SUCCESS;
    iso7816_phy_evt_t evt;

    if (m_iso7816_phy.sys_state != ISO7816_PHY_SYS_FSM_WARM_RESET)
    {
        return;
    }

    nrf_gpio_pin_set (m_iso7816_phy.rst_pin);   // RST=1

    err_code = uart_disable ();
    VERIFY_SUCCESS_VOID(err_code);

    err_code = uart_enable_rx ();
    VERIFY_SUCCESS_VOID(err_code);

    m_iso7816_phy.rx_buff     = &m_iso7816_phy.rx_data_internal;
    m_iso7816_phy.rx_buff_len = 1;
    m_iso7816_phy.rx_queued   = false;

    uart_rx ();

    DBG_SIGNAL();

    m_iso7816_phy.total_wait = 0;
    m_iso7816_phy.max_wait   = 0;

    m_iso7816_phy.sys_state = ISO7816_PHY_SYS_FSM_RUNNING;

    evt.evt_id = ISO7816_PHY_EVT_WARM_RESET;
    m_iso7816_phy.evt_cb(&evt);
}


/**@brief 2nd phase of clock stop */
static void clk_stop_cont (void)
{
    iso7816_phy_evt_t evt;

    clk_disable();  // Disable clock
    nrf_drv_timer_disable (&m_iso7816_phy.low_freq_timer);
    nrf_drv_gpiote_out_task_disable(m_iso7816_phy.clk_pin);
    nrf_gpio_pin_write (m_iso7816_phy.clk_pin, m_iso7816_phy.idle_lv);    // CLK level

    m_iso7816_phy.sys_state = ISO7816_PHY_SYS_FSM_IDLE;

    evt.evt_id = ISO7816_PHY_EVT_CLK_STOPPED;
    m_iso7816_phy.evt_cb(&evt);
}


/**@brief 2nd phase of clock resume */
static void clk_resume_cont (void)
{
    iso7816_phy_evt_t evt;

    m_iso7816_phy.sys_state = ISO7816_PHY_SYS_FSM_RUNNING;

    evt.evt_id = ISO7816_PHY_EVT_CLK_RESUMED;
    m_iso7816_phy.evt_cb(&evt);
}


uint32_t iso7816_phy_init (iso7816_phy_init_t *init_param)
{
    VERIFY_PARAM_NOT_NULL(init_param);
    VERIFY_PARAM_NOT_NULL(init_param->evt_cb);
    uint32_t err_code = NRF_SUCCESS;

    /* Initialize driver context. */
    memset (&m_iso7816_phy, 0, sizeof(iso7816_phy_t));
    m_iso7816_phy.sys_state     = ISO7816_PHY_SYS_FSM_DISABLED;
    
    m_iso7816_phy.high_freq_timer = m_high_freq_timer;
    m_iso7816_phy.low_freq_timer  = m_low_freq_timer;
		m_iso7816_phy.iso7816_uart_instance = m_7816_uart_instance;
	
    m_iso7816_phy.vcc_pin     = init_param->vcc_pin;
    m_iso7816_phy.clk_pin     = init_param->clk_pin;
    m_iso7816_phy.rst_pin     = init_param->rst_pin;
    m_iso7816_phy.io_pin      = init_param->io_pin;
    m_iso7816_phy.dbg_act_pin = init_param->dbg_act_pin;
    m_iso7816_phy.dbg_ppi_pin = init_param->dbg_ppi_pin;

    m_iso7816_phy.evt_cb     = init_param->evt_cb;
    m_iso7816_phy.clk_freq   = init_param->clock_freq;

    m_iso7816_phy.Fi     = ISO7816_PHY_Fd;
    m_iso7816_phy.Di     = ISO7816_PHY_Dd;
    m_iso7816_phy._1_etu = ISO7816_PHY_Fd / ISO7816_PHY_Dd;
    m_iso7816_phy.WT     = ISO7816_PHY_WT - 12;

    m_iso7816_phy.baudrate = ((1 << (int)m_iso7816_phy.clk_freq) * 1000000
                              * m_iso7816_phy.Di * ISO7816_PHY_UART_BAUDRATE_RATIO)
                             / m_iso7816_phy.Fi;//10735

    m_iso7816_phy.rx_buff     = &m_iso7816_phy.rx_data_internal;
    m_iso7816_phy.rx_buff_len = 1;

    /* Initialize SDK resources */
    gpio_init ();
    err_code = timers_init ();
    VERIFY_SUCCESS(err_code);

    /* Initialize CLK signal */
    err_code = clk_init (init_param->clock_freq);
    VERIFY_SUCCESS(err_code);

    /* Initialize UART */
    err_code = uart_init ();
    VERIFY_SUCCESS(err_code);

    /* Initialize I/O dbg signal */
    err_code = dbg_init ();
    VERIFY_SUCCESS(err_code);

    return err_code;
}


uint32_t iso7816_phy_send (uint8_t *data, uint16_t len)
{
    uint32_t err_code = NRF_SUCCESS;
    
    /* check if there is an active/pending Tx */
    if (m_iso7816_phy.tx_byte_cnt != 0)
    {
        err_code = NRF_ERROR_BUSY;
    }
    else
    {
        uint8_t bytes_to_send;

        m_iso7816_phy.tx_data     = data;
        m_iso7816_phy.tx_byte_cnt = 0; 
        m_iso7816_phy.tx_req_cnt  = len; 

        bytes_to_send = (len >= ISO7816_PHY_UART_MAX_TX)? ISO7816_PHY_UART_MAX_TX: len;

        /* Enable post-RX timer */
        // plus ~0.5 ETU delay from UART TX_END
        nrf_drv_timer_extended_compare (&m_iso7816_phy.low_freq_timer,
                                        (nrf_timer_cc_channel_t) ISO7816_PHY_TIMER_CH_TX_END,
                                        (1 + m_iso7816_phy.N) * m_iso7816_phy._1_etu + (m_iso7816_phy._1_etu >> 1),
                                        (nrf_timer_short_mask_t)0,
                                        false);

        err_code = nrf_drv_ppi_channel_enable (m_iso7816_phy.io_ppi_timer_resume);
        VERIFY_SUCCESS(err_code);

        /* Put to UART TX */
        err_code = nrf_drv_uart_tx ((const nrf_drv_uart_t *)&(m_iso7816_phy.iso7816_uart_instance), data, bytes_to_send);
        VERIFY_SUCCESS(err_code);

        DBG_SIGNAL();
    }
    
    return err_code;
}


uint32_t iso7816_phy_tx_enable (void)
{
    return uart_enable_tx ();
}


uint32_t iso7816_phy_rx_enable (uint8_t * buff, uint8_t len)
{
    uint32_t err_code = NRF_SUCCESS;

    if (!m_iso7816_phy.rx_on_going)
    {
        err_code = uart_enable_rx ();
        VERIFY_SUCCESS(err_code);

        m_iso7816_phy.rx_buff     = buff;
        m_iso7816_phy.rx_buff_len = len;

        err_code = uart_rx ();
    }
    else
    {
        m_iso7816_phy.rx_queued     = true;
        m_iso7816_phy.rx_q_buff     = buff;
        m_iso7816_phy.rx_q_buff_len = len;
    }
    return err_code;
}


uint32_t iso7816_phy_txrx_disable (void)
{
    return uart_disable ();
}


uint32_t iso7816_phy_timer_start (uint32_t t)
{
    uint32_t err_code = NRF_SUCCESS;

    m_iso7816_phy.total_wait = 0;
    m_iso7816_phy.max_wait   = t;

    if (t > ISO7816_PHY_MAX_WAIT)
    {
        sleep (ISO7816_PHY_MAX_WAIT, long_timer_handler);
    }
    else
    {
        sleep_and_stop (t, long_timer_handler);
    }
    return err_code;
}


uint32_t iso7816_phy_set_coding (bool direct)
{
    uint32_t err_code = NRF_SUCCESS;

    if (!direct)
    {
       err_code = NRF_ERROR_NOT_SUPPORTED;
    } 
    return err_code;
}


void iso7816_phy_set_Fi_Di (uint16_t Fi, uint8_t Di)
{
//    m_iso7816_phy.Fi     = Fi;
//    m_iso7816_phy.Di     = Di;
//    m_iso7816_phy._1_etu = Fi / Di;

//    uint64_t i = ((1 << (uint8_t)m_iso7816_phy.clk_freq)) * 1000000 * (uint64_t)m_iso7816_phy.Di * ISO7816_PHY_UART_BAUDRATE_RATIO;
//    m_iso7816_phy.baudrate = i / m_iso7816_phy.Fi;
////124797
//    m_iso7816_phy.uart_config.baudrate = (nrf_uart_baudrate_t)m_iso7816_phy.baudrate;
}


void iso7816_phy_set_WT (uint32_t WT)
{
    m_iso7816_phy.WT = WT;
}


void iso7816_phy_set_N (uint8_t N)
{
    m_iso7816_phy.N = N;
}

uint32_t iso7816_phy_clk_stop (uint8_t idle_lv)
{
    uint32_t err_code = NRF_SUCCESS;

    m_iso7816_phy.idle_lv   = idle_lv;
    m_iso7816_phy.sys_state = ISO7816_PHY_SYS_FSM_CLK_STOP;
    low_freq_timer_clear ();
    sleep_and_stop (ISO7816_PHY_tg, clk_stop_cont);

    return err_code;
}


uint32_t iso7816_phy_clk_resume (void)
{
    uint32_t err_code = NRF_SUCCESS;
    VERIFY_TRUE ((m_iso7816_phy.sys_state == ISO7816_PHY_SYS_FSM_IDLE), NRF_ERROR_INVALID_STATE);

    nrf_drv_timer_enable (&m_iso7816_phy.low_freq_timer);

    nrf_drv_gpiote_out_task_enable(m_iso7816_phy.clk_pin);
    clk_enable ();                      // CLK oscillate
    low_freq_timer_clear ();
    sleep_and_stop (ISO7816_PHY_th, clk_resume_cont);
    return err_code;
}


uint32_t iso7816_phy_cold_reset (void)
{
    uint32_t err_code = NRF_SUCCESS;

    m_iso7816_phy.sys_state = ISO7816_PHY_SYS_FSM_COLD_RESET;

    nrf_gpio_pin_clear (m_iso7816_phy.rst_pin);     // RST=0
    nrf_gpio_pin_set (m_iso7816_phy.vcc_pin);       // Vcc=1
    DBG_SIGNAL();
    clk_enable ();                      // CLK oscillate
    DBG_SIGNAL();
    io_set_input ();                    // I/O as input

    sleep_and_stop (400 + ISO7816_PHY_COLD_RESET_EXTRA_DELAY, cold_reset_cont);

    uart_enable_rx ();

    return err_code;
}

uint32_t iso7816_phy_warm_reset (void)
{
    uint32_t err_code = NRF_SUCCESS;

    m_iso7816_phy.sys_state = ISO7816_PHY_SYS_FSM_WARM_RESET;

    nrf_gpio_pin_clear (m_iso7816_phy.rst_pin);     // RST=0
    io_set_input ();
    DBG_SIGNAL();

    m_iso7816_phy.Fi     = ISO7816_PHY_Fd;
    m_iso7816_phy.Di     = ISO7816_PHY_Dd;
    m_iso7816_phy._1_etu = ISO7816_PHY_Fd / ISO7816_PHY_Dd;

    low_freq_timer_clear ();
    sleep_and_stop (400, warm_reset_cont);

    return err_code;
}

uint32_t iso7816_phy_deactivate (void)
{
    uint32_t err_code = NRF_SUCCESS;

    m_iso7816_phy.sys_state = ISO7816_PHY_SYS_FSM_DEACTIVATE;

    /* Signals */
    nrf_gpio_pin_clear (m_iso7816_phy.rst_pin);     // RST=0
    clk_disable();                                  // Disable clock
    nrf_gpio_pin_clear (m_iso7816_phy.clk_pin);     // CLK=0
    io_disable();                                   // I/O=0, and disable
    nrf_gpio_pin_clear (m_iso7816_phy.vcc_pin);     // Vcc=0
    DBG_SIGNAL();

			/* Need to disable UART first if Rx is active. */
		err_code = uart_disable ();
	
    /* Release SDK resources */
    clk_cleanup ();
    timers_cleanup ();

	
    m_iso7816_phy.sys_state = ISO7816_PHY_SYS_FSM_DISABLED;
    return err_code;
}


uint32_t iso7816_phy_uninit (void)
{
    uint32_t err_code = NRF_SUCCESS;

    err_code = iso7816_phy_deactivate ();

    err_code = dbg_cleanup ();

    return err_code;
}

void send_test(uint8_t *p, uint16_t len)
{
				iso7816_phy_tx_enable ();
				iso7816_phy_send (p, len);

}

/** @} */ // End tag for iso7816_phy_uart

