/* 
 * Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 * 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */


/**@file
 *
 * @defgroup nrf_iso7816_phy   ISO7816-3 PHY driver
 * @{
 * @brief    ISO7816-3 PHY driver.
 *
 * @details  This module contains the APIs and types exposed by the ISO7816-3 PHY driver module.
 *           These APIs and types can be used by the application to perform byte-wise and timer
 *           operations in PHY layer.
 *
 * @warning  Not fully tested.
 *
 * @note     Requirements: timer #1, timer #2 and 4 GPIOs.
 *
 */


#ifndef ISO7816_PHY_H_
#define ISO7816_PHY_H_

/* include files */
#include <stdint.h>
#include <stdbool.h>
#include "nrf_error.h"
#include "nrf_gpio.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"


/**
 * @defgroup iso7816_phy_const Constants
 * @{
 */

#define ISO7816_PHY_INVALID_PIN_NUMBER 0xFF     /**< Invalid pin number */

#define ISO7816_PHY_Fd                 372      /**< Default F. */
#define ISO7816_PHY_Dd                 1        /**< Default D. */

/** @} */


/**
 * @defgroup iso7816_phy_enums Enumerations
 * @{
 */

/**@brief Event IDs */
typedef enum
{
    ISO7816_PHY_EVT_COLD_RESET = 0,         /**< Cold reset has been successfully performed. */
    ISO7816_PHY_EVT_WARM_RESET,             /**< Warm reset has been successfully performed. */
    ISO7816_PHY_EVT_TX_DONE,                /**< The requested data has been sent. */
    ISO7816_PHY_EVT_RX_DATA,                /**< Data has been received. */
    ISO7816_PHY_EVT_TIMEOUT,                /**< ATR/CWT/BWT timed out. */
    ISO7816_PHY_EVT_RST_TIMEOUT,            /**< Reset timed out. */
    ISO7816_PHY_EVT_CLK_STOPPED,            /**< Clock stopped. */
    ISO7816_PHY_EVT_CLK_RESUMED,            /**< Clock resumed. */
    ISO7816_PHY_EVT_MAX                     /**< Invalid. */
} iso7816_phy_evt_id_t;


/**@brief Supported clock frequency */
typedef enum
{
    ISO7816_PHY_FREQ_1MHZ = 0,      /**< 1MHz CLK */
    ISO7816_PHY_FREQ_2MHZ,          /**< 2MHz CLK */
    ISO7816_PHY_FREQ_4MHZ,          /**< 4MHz CLK */
    ISO7816_PHY_FREQ_END
} iso7816_phy_freq_t;

/** @} */


/**
 * @defgroup iso7816_phy_structs Structures
 * @{
 */
 
/**@brief Structure for event @ref ISO7816_PHY_EVT_TX_DONE */
typedef struct
{
    uint16_t len;                           /**< Length of data sent. */
} iso7816_phy_evt_data_sent_t;


/**@brief Structure for event @ref ISO7816_PHY_EVT_RX_DATA */
typedef struct
{
    uint8_t   * data;                       /**< Pointer to RX data buffer. */
    uint16_t    len;                        /**< Length of received data. */
} iso7816_phy_evt_rx_data_t;


/**@brief Application notification structure */
typedef struct
{
    iso7816_phy_evt_id_t evt_id;            /**< Event ID, see ISO7816_EVT_ID */
    union
    {
        iso7816_phy_evt_data_sent_t tx_done;/**< Number of bytes sent, for event @ref ISO7816_PHY_EVT_TX_DONE */
        iso7816_phy_evt_rx_data_t rx_data;  /**< Received RAW data, for event @ref ISO7816_PHY_EVT_RX_DATA */
    } evt;
} iso7816_phy_evt_t;


/**@brief Event callback function */
typedef uint32_t (* iso7816_phy_evt_handler_t)(iso7816_phy_evt_t * p_event);


/**@brief Initialization configuration structure */
typedef struct
{
    iso7816_phy_freq_t        clock_freq;           /**< Clock frequency, see @ref iso7816_freq_t */
    uint8_t                   vcc_pin;              /**< Pin number of VCC */
    uint8_t                   clk_pin;              /**< Pin number of CLK */
    uint8_t                   rst_pin;              /**< Pin number of RST */
    uint8_t                   io_pin;               /**< Pin number of I/O */
    uint8_t                   dbg_act_pin;          /**< Pin number of "Debug: software activity" */
    uint8_t                   dbg_ppi_pin;          /**< Pin number of "Debug: PPI" */
    iso7816_phy_evt_handler_t evt_cb;               /**< Event callback function */
} iso7816_phy_init_t;


/** @} */


/**
 * @defgroup iso7816_phy_functions Functions
 * @{
 */

/**@brief Initialization, performs cold reset. */
uint32_t iso7816_phy_init (iso7816_phy_init_t *init_param);

/**@brief Send data through I/O. */
uint32_t iso7816_phy_send (uint8_t *data, uint16_t len);

/**@brief Enable or Disable Tx over I/O. */
uint32_t iso7816_phy_tx_enable (void);

/**@brief Enable or Disable Rx over I/O. */
uint32_t iso7816_phy_rx_enable (uint8_t * buff, uint8_t len);

/**@brief Disable Tx/Rx. */
uint32_t iso7816_phy_txrx_disable (void);

/**@brief Start timer. */
uint32_t iso7816_phy_timer_start (uint32_t t);

/**@brief Stop timer. */
uint32_t iso7816_phy_timer_stop (void);

/**@brief Function for setting coding type */
uint32_t iso7816_phy_set_coding (bool direct);

/**@brief Function for setting Fi and Di */
void iso7816_phy_set_Fi_Di (uint16_t Fi, uint8_t Di);

/**@brief Function for setting WT, in unit of Fi */
void iso7816_phy_set_WT (uint32_t WT);

/**@brief Function for setting N */
void iso7816_phy_set_N (uint8_t N);

/**@brief Performs clock stop. */
uint32_t iso7816_phy_clk_stop (uint8_t idle_lv);

/**@brief Performs clock resume. */
uint32_t iso7816_phy_clk_resume (void);

/**@brief Performs cold reset. */
uint32_t iso7816_phy_cold_reset (void);

/**@brief Performs warm reset. */
uint32_t iso7816_phy_warm_reset (void);

/**@brief Performs deactivation. */
uint32_t iso7816_phy_deactivate (void);

/**@brief Clean-up (destructor) */
uint32_t iso7816_phy_uninit (void);

/** @} */ // End tag for Function group.

#endif  // ISO7816_PHY_H_

/** @} */ // End tag for the file.


