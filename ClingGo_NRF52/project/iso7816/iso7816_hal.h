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
 * @defgroup nrf_iso7816_hal   ISO7816-3 HAL driver
 * @{
 * @brief    ISO7816-3 HAL driver.
 *
 * @details  This module contains the APIs and types exposed by the ISO7816-3 HAL driver module.
 *           These APIs and types can be used by the application to perform TPDU exchange.
 *
 * @warning  Not fully tested.
 *
 * @note     Requirements: timer #1, timer #2 and 4 GPIOs.
 *
 */


#ifndef ISO7816_HAL_H_
#define ISO7816_HAL_H_

/* include files */
#include <stdint.h>
#include <stdbool.h>
#include "iso7816_phy.h"
#include "nrf_delay.h"

/*since fm151 does not have the ability to supporty stanrd 7816 port*/
//#define  	__FM151__


#define NRF_LOG_PRINTF Y_SPRINTF
#define NRF_LOG_INIT
#define NRF_LOG  Y_SPRINTF
#define NRF_LOG_HEX_CHAR(x)
/**
 * @defgroup iso7816_hal_const Constants
 * @{
 */

#define ISO7816_HAL_TPDU_REQ_GET       0x00     /**< Get request: In TPDU, receive data from card. */
#define ISO7816_HAL_TPDU_REQ_PUT       0x01     /**< Put request: In TPDU, send data to card. */

/** @} */


/**
 * @defgroup iso7816_hal_enums Enumerations
 * @{
 */

/**@brief Supported clock frequency */
typedef enum
{
    ISO7816_HAL_FREQ_1MHZ = ISO7816_PHY_FREQ_1MHZ,  /**< 1MHz CLK */
    ISO7816_HAL_FREQ_2MHZ = ISO7816_PHY_FREQ_2MHZ,  /**< 2MHz CLK */
    ISO7816_HAL_FREQ_4MHZ = ISO7816_PHY_FREQ_4MHZ,  /**< 4MHz CLK */
    ISO7816_HAL_FREQ_END
} iso7816_hal_freq_t;


/**@brief Event IDs */
typedef enum
{
    ISO7816_HAL_EVT_RDY,                        /**< ISO7816 module was successfully initialized. */
    ISO7816_HAL_EVT_NOT_INITIALIZED,            /**< ISO7816 module failed to initialize. */
    ISO7816_HAL_EVT_TPDU_RESPONSE,              /**< TPDU response received. */
    ISO7816_HAL_EVT_TPDU_TIMEOUT,               /**< TPDU timeout. */
    ISO7816_HAL_EVT_FAULT,                      /**< Faults which are not recoverable by warm reset. */
    ISO7816_HAL_EVT_MAX                         /**< Invalid. */
} iso7816_hal_evt_id_t;

/** @} */


/**
 * @defgroup iso7816_hal_structs Structures
 * @{
 */
 
/**@brief Structure for event @ref ISO7816_HAL_EVT_ID_TPDU_RESPONSE */
typedef struct
{
    uint8_t sw1;                                /**< Status word #1 */
    uint8_t sw2;                                /**< Status word #2 */
} iso7816_hal_evt_tpdu_rsp_t;


/**@brief Application notification structure */
typedef struct
{
    iso7816_hal_evt_id_t evt_id;                /**< Event ID, see ISO7816_HAL_EVT_ID */
    union
    {
        iso7816_hal_evt_tpdu_rsp_t tpdu_rsp;    /**< TPDU response, for event @ref ISO7816_HAL_EVT_ID_TPDU_RESPONSE */
    } evt;
} iso7816_hal_evt_t;


/**@brief Event callback function */
typedef uint32_t (* iso7816_hal_evt_handler_t)(iso7816_hal_evt_t * p_event);



/**@brief Initialization configuration structure */
typedef struct
{
    iso7816_hal_freq_t        clock_freq;           /**< Clock frequency, see @ref iso7816_hal_freq_t */
    uint8_t                   vcc_pin;              /**< Pin number of VCC */
    uint8_t                   clk_pin;              /**< Pin number of CLK */
    uint8_t                   rst_pin;              /**< Pin number of RST */
    uint8_t                   io_pin;               /**< Pin number of I/O */
    uint8_t                   dbg_act_pin;          /**< Pin number of "Debug: software activity" */
    uint8_t                   dbg_ppi_pin;          /**< Pin number of "Debug: PPI" */
    iso7816_hal_evt_handler_t evt_cb;               /**< Event callback function */
		bool											enable_pps;						/**< Enable PPS negocitation */
} iso7816_hal_init_t;


/**@brief Parameters for TPDU request (see @ref iso7816_hal_send_tpdu) */
typedef struct
{
    uint8_t   cla;                      /**< CLA byte, see ISO7816-4, ch. 5.1.1 */
    uint8_t   ins;                      /**< INS byte, see ISO7816-4, ch. 5.1.2 */
    uint8_t   p1;                       /**< P1 byte, depends on INS. */
    uint8_t   p2;                       /**< P2 byte, depends on INS. */
    uint8_t   reserved;                 /**< Reserved. */
    uint8_t   req;                      /**< Request, GET or PUT, see @ref ISO7816_HAL_TPDU_REQ */
    uint16_t  len;                      /**< Length of data to be sent / received. */
    uint8_t * data;                     /**< Pointer to application provided buffer, for @ref ISO7816_HAL_TPDU_REQ_GET, and;
                                         *   Pointer to application provided data, for @ref ISO7816_HAL_TPDU_REQ_PUT.
                                         */
} iso7816_hal_tpdu_t;

/** @} */


/**
 * @defgroup iso7816_hal_functions Functions
 * @{
 */

/**@brief Initialization, performs cold reset. */
uint32_t iso7816_hal_init (iso7816_hal_init_t *init_param);

/**@brief Send TPDU */
uint32_t iso7816_hal_send_tpdu (iso7816_hal_tpdu_t *tpdu);

/**@brief Suspend PHY and HAL (clock stop, blocking). */
uint32_t iso7816_hal_suspend (void);

/**@brief Resume PHY and HAL (clock resume, blocking). */
uint32_t iso7816_hal_resume (void);

/**@brief Low level control. */
uint32_t iso7816_hal_ioctl (void *cmd);

/**@brief Clean-up (destructor) */
uint32_t iso7816_hal_uninit (void);

/**@brief Tool to log TPDU request */
void iso7816_hal_log_tpdu_req (iso7816_hal_tpdu_t *tpdu);

/**@brief Tool to log TPDU response */
void iso7816_hal_log_tpdu_rsp (iso7816_hal_tpdu_t *tpdu, uint8_t sw1, uint8_t sw2);

/**@brief Get ATR string. */
void iso7816_hal_get_atr (uint8_t **data, uint16_t *len);

/** @} */ // End tag for Function group.

#endif  // ISO7816_HAL_H_

/** @} */ // End tag for the file.


