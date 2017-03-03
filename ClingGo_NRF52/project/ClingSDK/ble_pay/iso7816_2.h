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
 * @defgroup nrf_iso7816   ISO7816-3 driver
 * @{
 * @brief    ISO7816-3 TPDU transfer driver.
 *
 * @details  This module contains the APIs and types exposed by the ISO7816-3 driver module. These
 *           APIs and types can be used by the application to perform GET/PUT operations in TPDU
 *           transfer.
 *
 * @warning  Not fully tested.
 *
 * @note     Requirements: timer #1, timer #2 and 4 GPIOs.
 *
 */


#ifndef ISO7816_H_
#define ISO7816_H_

/* include files */
#include <stdint.h>
#include <stdbool.h>
#include "nrf_error.h"


/**
 * @defgroup iso7816_const Constants
 * @{
 */

#define ISO7816_INVALID_PIN_NUMBER 0xFF     /**< Invalid pin number */

#define ISO7816_TPDU_REQ_GET       0x00     /**< Get request: In TPDU, receive data from card. */
#define ISO7816_TPDU_REQ_PUT       0x01     /**< Put request: In TPDU, send data to card. */
#define ISO7816_TPDU_REQ_NULL			 0xff
/** @} */


/**
 * @defgroup iso7816_enums Enumerations
 * @{
 */

/**@brief Supported clock frequency */
typedef enum {
    ISO7816_FREQ_1MHZ = 0,                  /**< 1MHz CLK */
    ISO7816_FREQ_2MHZ,                      /**< 2MHz CLK */
    ISO7816_FREQ_4MHZ,                      /**< 4MHz CLK */
    ISO7816_FREQ_END
} iso7816_freq_t;


/**@brief Event IDs */
typedef enum {
    ISO7816_EVT_ID_RDY,                     /**< ISO7816 module was successfully initialized. */
    ISO7816_EVT_ID_TPDU_RESPONSE,           /**< TPDU response received. */
    ISO7816_EVT_ID_CLOCK_RESUMED,           /**< ISO7816 CLK was resumed. */
    ISO7816_EVT_ID_DEACTIVATED              /**< Interface device deactivated. */
} iso7816_evt_id_t;

/** @} */


/**
 * @defgroup iso7816_structs Structures
 * @{
 */

/**@brief Structure for event @ref ISO7816_EVT_ID_TPDU_RESPONSE */
typedef struct {
    uint8_t sw1;                            /**< Status word #1 */
    uint8_t sw2;                            /**< Status word #2 */
} iso7816_evt_tpdu_rsp_t;


/**@brief Structure for event @ref ISO7816_EVT_ID_RX_DATA */
typedef struct {
    uint8_t   * data;                       /**< Pointer to RX data buffer. */
    uint16_t    len;                        /**< Length of received data. */
} iso7816_evt_rx_data_t;


/**@brief Application notification structure */
typedef struct {
    iso7816_evt_id_t evt_id;                /**< Event ID, see ISO7816_EVT_ID */
    union {
        iso7816_evt_tpdu_rsp_t tpdu_rsp;    /**< TPDU response, for event @ref ISO7816_EVT_ID_TPDU_RESPONSE */
        iso7816_evt_rx_data_t rx_data;      /**< Received RAW data, for event @ref ISO7816_EVT_ID_RX_DATA */
    } evt;
} iso7816_evt_t;


/**@brief Event callback function */
typedef uint32_t (* iso7816_evt_handler_t)(iso7816_evt_t * p_event);


/**@brief Initialization configuration structure */
typedef struct {
    iso7816_freq_t        clock_freq;           /**< Clock frequency, see @ref iso7816_freq_t */
    uint8_t               vcc_pin;              /**< Pin number of VCC */
    uint8_t               clk_pin;              /**< Pin number of CLK */
    uint8_t               rst_pin;              /**< Pin number of RST */
    uint8_t               io_pin;               /**< Pin number of I/O */
    uint8_t               dbg_activity_pin;     /**< Pin number of "Debug: software activity" */
    uint8_t               dbg_ppi_pin;          /**< Pin number of "Debug: PPI" */
    iso7816_evt_handler_t evt_cb;               /**< Event callback function */
} iso7816_init_t;


/**@brief Parameters for TPDU request (see @ref iso7816_send_TPDU) */
typedef struct {
    uint8_t   cla;                      /**< CLA byte, see ISO7816-4, ch. 5.1.1 */
    uint8_t   ins;                      /**< INS byte, see ISO7816-4, ch. 5.1.2 */
    uint8_t   p1;                       /**< P1 byte, depends on INS. */
    uint8_t   p2;                       /**< P2 byte, depends on INS. */
    uint8_t   reserved;                 /**< Reserved. */
    uint8_t   req;                      /**< Request, GET or PUT, see @ref ISO7816_TPDU_REQ */
    uint16_t  len;                      /**< Length of data to be sent / received. */
    uint8_t * data;                     /**< Pointer to application provided buffer, for @ref ISO7816_TPDU_REQ_GET, and;
                                         *   Pointer to application provided data, for @ref ISO7816_TPDU_REQ_PUT.
                                         */
} iso7816_tpdu_t;

/** @} */


/**
 * @defgroup iso7816_functions Functions
 * @{
 */

/**@brief Initialization, performs cold reset. */
uint32_t iso7816_init (iso7816_init_t *init_param);

/**@brief Send TPDU */
uint32_t iso7816_send_TPDU (iso7816_tpdu_t *tpdu);

/**@brief Performs clock stop.
 * @note  Not supported.
 */
uint32_t iso7816_stop_clk (void);

/**@brief Performs warm reset. */
uint32_t iso7816_warm_reset (void);

/**@brief Clean-up (destructor) */
uint32_t iso7816_cleanup (void);

/** @} */ // End tag for Function group.

#endif  // ISO7816_H_

/** @} */ // End tag for the file.


