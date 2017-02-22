/**
 * \addtogroup BSP
 * \{
 * \addtogroup DEVICES
 * \{
 * \addtogroup SPI
 * \{
 */

/*
 *****************************************************************************************
 *
 * @file hw_spi.c
 *
 * @brief Implementation of the SPI Low Level Driver.
 *
 * Copyright (C) 2015. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd. All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 *****************************************************************************************
 */
#define dg_configUSE_HW_SPI 1
#if dg_configUSE_HW_SPI


#include <stdint.h>
//#include <core_cm0.h>
#include "hw_spi.h"
#include "main.h"

typedef enum {
    HW_SPI_TRANSFER_READ      = 1,
    HW_SPI_TRANSFER_WRITE     = 2,
    HW_SPI_TRANSFER_READWRITE = 3,
} HW_SPI_TRANSFER;

typedef struct {
    SPI_Pad            cs_pad;
    hw_spi_tx_callback rx_cb;
    hw_spi_tx_callback tx_cb;
    void               *cb_data;

    const uint8_t      *tx_buffer;
    uint16_t           tx_len;
    uint16_t           tx_ix;

    uint8_t            *rx_buffer;
    uint16_t           rx_len;
    uint16_t           rx_ix;

    HW_SPI_TRANSFER    transfer_mode;
#ifdef HW_SPI_DMA_SUPPORT
    uint8_t            use_dma;
    DMA_setup          tx_dma;
    DMA_setup          rx_dma;
#endif
} SPI_Data;

static SPI_Data spi_data[2];

#define SPI_INT(id)  ((id) == HW_SPI1 ? (SPI_IRQn) : (SPI2_IRQn))
#define SPIIX(id)    ((id) == HW_SPI1 ? 0 : 1)
#define SPIDATA(id)  (&spi_data[SPIIX(id)])

//==================== Configuration functions =================================

uint8_t hw_spi_is_interrupt_enabled(HW_SPI_ID id)
{
    // Get MINT bit from SPI control register
    return 1;
}

uint8_t hw_spi_get_interrupt_status(HW_SPI_ID id)
{
    // Get SPI interrupt bit from SPI control register
    return 1;//HW_SPI_REG_GETF(id, SPI_CTRL_REG, SPI_INT_BIT);
}

void hw_spi_clear_interrupt(HW_SPI_ID id)
{
    // Write 1 to SPI Clear Interrupt Register
    //SBA(id)->SPI_CLEAR_INT_REG = 1;
}

void hw_spi_enable(HW_SPI_ID id, uint8_t on)
{
    // Set SPI ON bit in SPI control register
    //HW_SPI_REG_SETF(id, SPI_CTRL_REG, SPI_ON, on);
}

uint8_t hw_spi_is_enabled(HW_SPI_ID id)
{
    // Get SPI ON bit from SPI control register
    return 1;//HW_SPI_REG_GETF(id, SPI_CTRL_REG, SPI_ON);
}

void hw_spi_set_clock_phase(HW_SPI_ID id, HW_SPI_PHA phase)
{

}

uint8_t hw_spi_get_clock_phase(HW_SPI_ID id)
{
    // Get SPI clock phase bit from SPI control register
    return 1;//HW_SPI_REG_GETF(id, SPI_CTRL_REG, SPI_PHA);
}

void hw_spi_set_clock_polarity(HW_SPI_ID id, HW_SPI_POL pol)
{
//        uint32_t on = hw_spi_is_enabled(id);
//        hw_spi_enable(id, 0);                                   // close SPI block
//        HW_SPI_REG_SETF(id, SPI_CTRL_REG, SPI_POL, pol);        // set SPI POL bit
//        hw_spi_enable(id, on);                                  // open SPI block
}

uint8_t hw_spi_get_clock_polarity(HW_SPI_ID id)
{
    // Get SPI clock polarity bit from SPI control register
    return 1;//HW_SPI_REG_GETF(id, SPI_CTRL_REG, SPI_POL);
}

void hw_spi_set_clock_freq(HW_SPI_ID id, HW_SPI_FREQ freq)
{
//        uint32_t on = hw_spi_is_enabled(id);
//        hw_spi_enable(id, 0);                                   // close SPI block
//        HW_SPI_REG_SETF(id, SPI_CTRL_REG, SPI_CLK, freq);       // set frequency divider bit
//        hw_spi_enable(id, on);                                  // open SPI block
}

uint8_t hw_spi_get_clock_freq(HW_SPI_ID id)
{
    // Get SPI clock frequency field from SPI control register
    return 1;//HW_SPI_REG_GETF(id, SPI_CTRL_REG, SPI_CLK);
}

void hw_spi_set_do_level(HW_SPI_ID id, uint8_t spi_do)
{
//        uint32_t on = hw_spi_is_enabled(id);
//        hw_spi_enable(id, 0);                                   // close SPI block
//        HW_SPI_REG_SETF(id, SPI_CTRL_REG, SPI_DO, spi_do);      // set SPI DO output level
//        hw_spi_enable(id, on);                                  // open SPI block
}

uint8_t hw_spi_get_do_level(HW_SPI_ID id)
{
    // Get SPI DO output level from SPI control register
    return 1;//HW_SPI_REG_GETF(id, SPI_CTRL_REG, SPI_DO);
}

void hw_spi_set_force_do(HW_SPI_ID id, uint8_t spi_force_do)
{
//        uint32_t on = hw_spi_is_enabled(id);
//        hw_spi_enable(id, 0);                                           // close SPI block
//        HW_SPI_REG_SETF(id, SPI_CTRL_REG, SPI_FORCE_DO, spi_force_do);  // set SPI force DO bit
//        hw_spi_enable(id, on);                                          // open SPI block
}

uint8_t hw_spi_get_force_do(HW_SPI_ID id)
{
    // Get value of SPI force DO bit from SPI control register
    return 1;//HW_SPI_REG_GETF(id, SPI_CTRL_REG, SPI_FORCE_DO);
}

void hw_spi_set_mode(HW_SPI_ID id, HW_SPI_MODE smn)
{
//        uint32_t on = hw_spi_is_enabled(id);
//        hw_spi_enable(id, 0);                                   // close SPI block
//        HW_SPI_REG_SETF(id, SPI_CTRL_REG, SPI_SMN, smn);        // set SMN bit
//        hw_spi_enable(id, on);                                  // open SPI block
}

uint8_t hw_spi_is_slave(HW_SPI_ID id)
{
    // Get value of SPI master/slave mode from SPI control register
    return 0;//HW_SPI_REG_GETF(id, SPI_CTRL_REG, SPI_SMN);
}

void hw_spi_set_word_size(HW_SPI_ID id, HW_SPI_WORD word)
{
//        uint32_t on = hw_spi_is_enabled(id);
//        hw_spi_enable(id, 0);                                   // close SPI block
//        HW_SPI_REG_SETF(id, SPI_CTRL_REG, SPI_WORD, word);      // set SPI WORD bit
//        hw_spi_enable(id, on);                                  // open SPI block
}

void hw_spi_set_9th_bit(HW_SPI_ID id, uint8_t val)
{
    // Get value of the SPI_EN_CTRL bit from the SPI control register
    // HW_SPI_REG_SETF(id, SPI_CTRL_REG1, SPI_9BIT_VAL, val);
}

uint8_t hw_spi_get_9th_bit(HW_SPI_ID id)
{
    // Get value of the SPI_EN_CTRL bit from the SPI control register
    return 1;//HW_SPI_REG_GETF(id, SPI_CTRL_REG1, SPI_9BIT_VAL);
}

HW_SPI_WORD hw_spi_get_word_size(HW_SPI_ID id)
{
    // Get value of SPI master/slave mode from SPI control register
    return HW_SPI_WORD_8BIT;//HW_SPI_REG_GETF(id, SPI_CTRL_REG, SPI_WORD);
}

uint32_t hw_spi_get_memory_word_size(HW_SPI_ID id)
{
    switch (hw_spi_get_word_size(id)) {
    case HW_SPI_WORD_16BIT:
        return 2;
    case HW_SPI_WORD_32BIT:
        return 4;
    case HW_SPI_WORD_8BIT:
    case HW_SPI_WORD_9BIT:
    default:
        return 1;
    }

}

void hw_spi_reset(HW_SPI_ID id)
{
    // Set reset bit in SPI control register
    //HW_SPI_REG_SETF(id, SPI_CTRL_REG, SPI_RST, 1);
}

uint8_t hw_spi_is_tx_fifo_full(HW_SPI_ID id)
{
    // Get value of the SPI TX FIFO full bit from the SPI control register
    return 1;//HW_SPI_REG_GETF(id, SPI_CTRL_REG, SPI_TXH);
}

uint8_t hw_spi_get_di(HW_SPI_ID id)
{
    // Get value of the SPI DI pin
    return 1;//HW_SPI_REG_GETF(id, SPI_CTRL_REG, SPI_DI);
}

void hw_spi_set_cs_ctrl(HW_SPI_ID id, uint8_t spi_en_ctrl)
{
    uint32_t on = hw_spi_is_enabled(id);
    // open SPI block
}

uint8_t hw_spi_get_cs_ctrl(HW_SPI_ID id)
{
    // Get value of the SPI_EN_CTRL bit from the SPI control register
    return 1;//HW_SPI_REG_GETF(id, SPI_CTRL_REG, SPI_EN_CTRL);
}

void hw_spi_set_cs_pad(HW_SPI_ID id, const SPI_Pad *cs_pad)
{
    //SPI_Data *spid = SPIDATA(id);

    // spid->cs_pad.port = cs_pad->port;
    //  spid->cs_pad.pin = cs_pad->pin;
}
static const nrf_drv_spi_t m_spi_master_0 = NRF_DRV_SPI_INSTANCE(0);
/**@brief Handler for SPI0 master events.
 *
 * @param[in] event SPI master event.
 */
static BOOLEAN  m_transfer_completed = TRUE;
static void spi_master_0_event_handler(nrf_drv_spi_event_t event)
{
    switch (event) {
    case NRF_DRV_SPI_EVENT_DONE:
        m_transfer_completed = TRUE;
        break;

    default:
        // No implementation needed.
        break;
    }
}

void hw_spi_disable(HW_SPI_ID id)
{
    if (id == NRF_SPI0) {
        nrf_drv_spi_uninit(&m_spi_master_0);
    }
    //  SPI_Data *spid = SPIDATA(id);
}

void hw_spi_init(HW_SPI_ID id, const spi_config *cfg)
{

    I32U err_code = NRF_SUCCESS;

    nrf_drv_spi_config_t config ;
    SPI_Data *spid = SPIDATA(id);

    config.sck_pin      = SPIM0_SCK_PIN;
    config.mosi_pin     = SPIM0_MOSI_PIN;
    config.miso_pin     = SPIM0_MISO_PIN;
    config.ss_pin       = NRF_DRV_SPI_PIN_NOT_USED;
    config.irq_priority = APP_IRQ_PRIORITY_LOW;
    config.orc          = 0xCC;
    config.frequency    = (nrf_drv_spi_frequency_t)cfg->xtal_freq;	
    config.mode         = (nrf_drv_spi_mode_t)cfg->phase_mode;
    config.bit_order    = (0 ?  NRF_DRV_SPI_BIT_ORDER_LSB_FIRST : NRF_DRV_SPI_BIT_ORDER_MSB_FIRST);

    // set SPI CS pad
    spid->cs_pad.port = cfg->cs_pad.port;
    spid->cs_pad.pin = cfg->cs_pad.pin;

    if (id == NRF_SPI0) {
        nrf_drv_spi_uninit(&m_spi_master_0);
        err_code = nrf_drv_spi_init(&m_spi_master_0, &config, spi_master_0_event_handler);

        if (err_code == NRF_ERROR_INVALID_STATE) {
            Y_SPRINTF("[SPI] Uninitialized - figure out why!");
            nrf_drv_spi_uninit(&m_spi_master_0);
            err_code = nrf_drv_spi_init(&m_spi_master_0, &config, spi_master_0_event_handler);
        }
        APP_ERROR_CHECK(err_code);
    }

    //  SPI_Data *spid = SPIDATA(id);
}

//=========================== CS handling function =============================

void hw_spi_set_cs_low(HW_SPI_ID id)
{
    //SPI_Data *spid = SPIDATA(id);
    // hw_gpio_set_inactive(spid->cs_pad.port, spid->cs_pad.pin);    // pull CS low
    //	nrf_gpio_pin_set(spid->cs_pad.pin);
    //	nrf_gpio_cfg_output(spid->cs_pad.pin);
    //   nrf_gpio_pin_clear(spid->cs_pad.pin);
}

void hw_spi_set_cs_high(HW_SPI_ID id)
{
   // SPI_Data *spid = SPIDATA(id);
    // hw_gpio_set_inactive(spid->cs_pad.port, spid->cs_pad.pin);    // pull CS low
    //  nrf_gpio_pin_set(spid->cs_pad.pin);
    //hw_gpio_set_active(spid->cs_pad.port, spid->cs_pad.pin);    // push CS high
}

//=========================== FIFO control functions ===========================

void hw_spi_set_fifo_mode(HW_SPI_ID id, HW_SPI_FIFO mode)
{
//        uint32_t on = hw_spi_is_enabled(id);
//        hw_spi_enable(id, 0);                                           // close SPI block
//        HW_SPI_REG_SETF(id, SPI_CTRL_REG1, SPI_FIFO_MODE, mode);        // set SPI FIFO bit
//        hw_spi_enable(id, on);                                          // open SPI block
}

HW_SPI_FIFO hw_spi_get_fifo_mode(HW_SPI_ID id)
{
    // Get the SPI FIFO mode from the secondary SPI control register
    return HW_SPI_FIFO_NONE;//HW_SPI_REG_GETF(id, SPI_CTRL_REG1, SPI_FIFO_MODE);
}

HW_SPI_FIFO hw_spi_change_fifo_mode(HW_SPI_ID id, HW_SPI_FIFO mode)
{
    HW_SPI_FIFO old_mode = hw_spi_get_fifo_mode(id);

    if (old_mode != mode) {
        if (old_mode != HW_SPI_FIFO_RX_ONLY || !hw_spi_is_slave(id)) {
            // hw_spi_wait_while_busy(id);
        }
        // hw_spi_set_fifo_mode(id, mode);
    }

    return old_mode;
}

//=========================== DMA control functions ============================

#ifdef HW_SPI_DMA_SUPPORT

static void hw_spi_rx_dma_callback(void *user_data, uint16_t len)
{
    SPI_Data *spid = user_data;
    hw_spi_tx_callback cb = spid->rx_cb;

    spid->rx_cb = NULL;
    spid->rx_ix = len;
    spid->tx_ix = len;
    if (cb) {
        cb(spid->cb_data, len);
    }
}

static void hw_spi_tx_dma_callback(void *user_data, uint16_t len)
{
    SPI_Data *spid = user_data;
    hw_spi_tx_callback cb = spid->tx_cb;

    spid->tx_cb = NULL;
    spid->tx_ix = len;
    spid->tx_ix = len;
    if (cb) {
        cb(spid->cb_data, len);
    }
}

void hw_spi_set_dma_channels(HW_SPI_ID id, int8_t channel, HW_DMA_PRIO pri)
{
    SPI_Data *spid = SPIDATA(id);
    uint16_t wordsize = hw_spi_get_memory_word_size(id);

    /* Make sure the channel is valid or -1 (no DMA) */
    ASSERT_ERROR(channel < 0 ||
                 channel == HW_DMA_CHANNEL_0 ||
                 channel == HW_DMA_CHANNEL_2 ||
                 channel == HW_DMA_CHANNEL_4 ||
                 channel == HW_DMA_CHANNEL_6);

    if (channel < 0 || wordsize > 2) {
        spid->use_dma = 0;
        spid->rx_dma.channel_number = 0;
        spid->tx_dma.channel_number = 0;
    } else {
        spid->use_dma = 1;

        spid->rx_dma.channel_number = channel;
        spid->rx_dma.bus_width = wordsize == 1 ? HW_DMA_BW_BYTE : HW_DMA_BW_HALFWORD;
        spid->rx_dma.irq_enable = HW_DMA_IRQ_STATE_ENABLED;
        spid->rx_dma.dma_req_mux = SPIIX(id) == 0 ? HW_DMA_TRIG_SPI_RXTX :
                                   HW_DMA_TRIG_SPI2_RXTX;
        spid->rx_dma.irq_nr_of_trans = 0;
        spid->rx_dma.a_inc = HW_DMA_AINC_FALSE;
        spid->rx_dma.b_inc = HW_DMA_BINC_TRUE;
        spid->rx_dma.circular = HW_DMA_MODE_NORMAL;
        spid->rx_dma.dma_prio = pri;
        spid->rx_dma.dma_idle = HW_DMA_IDLE_INTERRUPTING_MODE; /* Not used by the HW in this case */
        spid->rx_dma.dma_init = HW_DMA_INIT_AX_BX_AY_BY;
        spid->rx_dma.dreq_mode = HW_DMA_DREQ_TRIGGERED;
        spid->rx_dma.src_address = (uint32_t)&SBA(id)->SPI_RX_TX_REG0;
        spid->rx_dma.dest_address = 0;  // Change during transmission
        spid->rx_dma.length = 0;       // Change during transmission
        spid->rx_dma.callback = hw_spi_rx_dma_callback;
        spid->rx_dma.user_data = spid;

        spid->tx_dma.channel_number = channel + 1;
        spid->tx_dma.bus_width = wordsize == 1 ? HW_DMA_BW_BYTE : HW_DMA_BW_HALFWORD;
        spid->tx_dma.irq_enable = HW_DMA_IRQ_STATE_ENABLED;
        spid->tx_dma.dma_req_mux = SPIIX(id) == 0 ? HW_DMA_TRIG_SPI_RXTX :
                                   HW_DMA_TRIG_SPI2_RXTX;
        spid->tx_dma.irq_nr_of_trans = 0;
        spid->tx_dma.a_inc = HW_DMA_AINC_TRUE;
        spid->tx_dma.b_inc = HW_DMA_BINC_FALSE;
        spid->tx_dma.circular = HW_DMA_MODE_NORMAL;
        spid->tx_dma.dma_prio = pri;
        spid->tx_dma.dma_idle = HW_DMA_IDLE_INTERRUPTING_MODE; /* Not used by the HW in this case */
        spid->tx_dma.dma_init = HW_DMA_INIT_AX_BX_AY_BY;
        spid->tx_dma.dreq_mode = HW_DMA_DREQ_TRIGGERED;
        spid->tx_dma.src_address = 0; // Change during transmission
        spid->tx_dma.dest_address = (uint32_t)&SBA(id)->SPI_RX_TX_REG0;
        spid->tx_dma.length = 0;     // Change during transmission
        spid->tx_dma.callback = hw_spi_tx_dma_callback;
        spid->tx_dma.user_data = spid;
    }
}
#endif

//===================== Read/Write functions ===================================

uint16_t hw_spi_writeread(HW_SPI_ID id, uint16_t val)
{
    uint16_t v;
//        hw_spi_fifo_write16(id, val);
//        while (!hw_spi_get_interrupt_status(id)) {
//        }
//        v = hw_spi_fifo_read16(id);
//        hw_spi_clear_interrupt(id);
    return v;
}

uint32_t hw_spi_writeread32(HW_SPI_ID id, uint32_t val)
{
    uint32_t v;
//        hw_spi_fifo_write32(id, val);
//        while (!hw_spi_get_interrupt_status(id)) {
//        }
//        v = hw_spi_fifo_read32(id);
//        hw_spi_clear_interrupt(id);
    return v;
}
#if 0
static void hw_spi_read_word(HW_SPI_ID id, uint8_t *buf, uint32_t wordsize)
{
    switch (wordsize) {
    case 1:
        // *buf = hw_spi_fifo_read8(id);
        break;
    case 2:
        // *(uint16_t *) buf = hw_spi_fifo_read16(id);
        break;
    case 4:
        //  *(uint32_t *) buf = hw_spi_fifo_read32(id);
        break;
    }
}

static void hw_spi_write_word(HW_SPI_ID id, const uint8_t *buf, uint32_t wordsize)
{
    switch (wordsize) {
    case 1:
        // hw_spi_fifo_write8(id, *buf);
        break;
    case 2:
        //hw_spi_fifo_write16(id, *(uint16_t *) buf);
        break;
    case 4:
        // hw_spi_fifo_write32(id, *(uint32_t *) buf);
        break;
    }
}

static void hw_spi_transfer_write(HW_SPI_ID id)
{

}

static void hw_spi_transfer_read(HW_SPI_ID id)
{
}

static void hw_spi_transfer(HW_SPI_ID id)
{

}
#endif
static void _spi_master_op_wait_done()
{
    I16U i;

    if (m_transfer_completed)
        return;

    // Wait for up to 200 ms
    for (i = 0; i < 200; i++) {
        BASE_delay_msec(1);

        if (m_transfer_completed)
            break;
    }

    if(!m_transfer_completed)
        Y_SPRINTF("[SPI] transfer incompleted, wait for 200ms");
}

int hw_spi_writeread_buf(HW_SPI_ID id, const uint8_t *out_buf, uint16_t out_len, uint8_t *in_buf, uint16_t in_len,
                         hw_spi_tx_callback cb, void *user_data)
{
    int ret = FALSE;
    SPI_Data *spid = SPIDATA(id);
    _spi_master_op_wait_done();
    if(m_transfer_completed == TRUE) {
        ret = TRUE;
        m_transfer_completed = FALSE;
        if (id == NRF_SPI0) {
            nrf_drv_spi_transfer(&m_spi_master_0, out_buf, out_len, in_buf, in_len, spid->cs_pad.pin);
        }
    }
    cb(user_data, 0);
    return ret;

}

int hw_spi_write_buf(HW_SPI_ID id, const uint8_t *out_buf, uint16_t len,
                     hw_spi_tx_callback cb, void *user_data)
{
    int ret = FALSE;
    SPI_Data *spid = SPIDATA(id);
    _spi_master_op_wait_done();
    static uint8_t inbuffer[2];
    if(m_transfer_completed == TRUE) {
        ret = TRUE;
        m_transfer_completed = FALSE;
        if (id == NRF_SPI0) {
            nrf_drv_spi_transfer(&m_spi_master_0, out_buf, len, inbuffer, 0, spid->cs_pad.pin);
        }
    }
    cb(user_data, 0);
    return ret;
}


int hw_spi_read_buf(HW_SPI_ID id, uint8_t *in_buf, uint16_t len,
                    hw_spi_tx_callback cb, void *user_data)
{
    int ret = FALSE;
    SPI_Data *spid = SPIDATA(id);
    _spi_master_op_wait_done();
    if(m_transfer_completed == TRUE) {
        ret = TRUE;
        m_transfer_completed = FALSE;
        if (id == NRF_SPI0) {
            nrf_drv_spi_transfer(&m_spi_master_0,  in_buf, 0,in_buf, len, spid->cs_pad.pin);
        }
    }
    cb(user_data, 0);
    return ret;
}
#if 0
//=========================== Interrupt handling ===============================

static void SPI_Interrupt_Handler(HW_SPI_ID id)
{

}

/**
 * \brief SPI1 Interrupt Handler
 *
 */
void SPI_Handler(void)
{

}

/**
 * \brief SPI2 Interrupt Handler
 *
 */
void SPI2_Handler(void)
{

}
#endif
#endif /* dg_configUSE_HW_SPI */
/**
 * \}
 * \}
 * \}
 */

