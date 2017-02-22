/******************************************************************************
 * Copyright 2013-2016 HIcling Systems (Mike)
 *
 * FileName: clingpay_cmd.c
 *
 * Description: entry file of user application
 *
 * Modification history:
 *     2016/1/1, v1.0 create this file.
 *******************************************************************************/
#include "clingpay_cmd.h"
#include "clingpay_mac.h"
#include "stdio.h"
#include <unistd.h>
#include "main.h"
#include "platform_devices.h"
#include "ad_uart.h"
#include "sys_power_mgr.h"
/*********************************************************************
 * MACROS
 */
#define Z32_PCKAGE_REV_IDENTIFICATION  0X6F
#define Z32_PCKAGE_SEND_IDENTIFICATION  0X80
#define COMPOSE_16BITS_DATA(_X, _Y) (((uint16_t)_X)<<8 | ((uint16_t)_Y))
#define SWAP_UINT16_T(__X)((((__X) & 0x00FF) << 8) | (((__X) & 0xFF00) >> 8))

#define PCKAGE_SIZE 5
#define BUFFER_SIZE 32

/*********************************************************************
 * TYPEDEFS
 */
/*used to implement state machine*/
enum {
        Z32_PACKAGE_BEGIN = 0,
        Z32_PACKAGE_TYPE,
        Z32_PACKAGE_BODY_SIZE_MSB,
        Z32_PACKAGE_BODY_SIZE_LSB,
        Z32_PACKAGE_UNSED_BYTE,
        Z32_PACKAGE_HEADER_END,
        Z32_PACKAGE_TRANSFER,
        Z32_PACKAGE_PAYLOAD,
        Z32_PACKAGE_END,
        Z32_PACKAGE_NULL
};

/*********************************************************************
 * GLOBAL VARIABLES
 */
static struct pay_rev_package shared_rev_pkg_mem;
#define SSC_MASK  0X0FFF
/*********************************************************************
 * static VARIABLES
 */
static int (*z32_rev_cb)(uint8_t*, uint16_t) = NULL;
static int (*pkg_rev_cb)(struct pay_rev_package*) = NULL;
static uint8_t _uart2_init = 0;
static uart_device uart_dbg_hdl_serial2 = NULL;
static TASK_HANDLE_SETS *hdls = NULL;
static uint16_t ssc_record = 0;
static OS_TIMER timer_cling_pay = NULL;
//static uint8_t tx_buffer[512];
//static uint8_t rx_buffer[512];

static uint8_t z32_rev_buf[300];
static void uart_read_cb(void *user_data, uint16_t transferred);
static int clingpay_send();
static uint16_t crc16_modbus(uint8_t *updata, uint16_t len);
static int ble_z32_send_package_back(uint8_t *p, uint16_t size);
/*********************************************************************
 * EXTERNAL VARIABLES
 */

static int start_z32_comm_activity()
{
        if (OS_TIMER_IS_ACTIVE(timer_cling_pay) == true) {
                OS_TIMER_STOP(timer_cling_pay, 0);
        }
        if (OS_TIMER_START(timer_cling_pay, 0) != OS_TIMER_SUCCESS) {
                return -1;
        }
        pm_stay_alive();
        return 0;
}
static int stop_z32_comm_activity()
{
        if (OS_TIMER_IS_ACTIVE(timer_cling_pay) == true) {
                OS_TIMER_STOP(timer_cling_pay, 0);
                pm_resume_sleep();
        }

        return 0;
}

static void cling_pay_cmd_timeout_cb(OS_TIMER t)
{
        OS_TIMER_STOP(timer_cling_pay, 0);
        pm_resume_sleep();
        return;
}

int reg_cling_pay_cmd_callback(int (*cb)(struct pay_rev_package*))
{
        if (cb != NULL) {
                pkg_rev_cb = cb;
        }
        return 0;
}

int set_z32_rev_cb(int (*cb)(uint8_t*, uint16_t))
{
        if (cb != NULL) {
                z32_rev_cb = cb;
        }
        return 0;

}

int restore_z32_rev_cb()
{
        z32_rev_cb = ble_z32_send_package_back;
        return 0;
}

static int ble_z32_send_package_back(uint8_t *p, uint16_t size)
{
        shared_rev_pkg_mem.header.data_lenth = (size);
        shared_rev_pkg_mem.header.cmd_type = (BTC_IO_OK);
        if (shared_rev_pkg_mem.data.body != NULL) {
                memcpy(shared_rev_pkg_mem.data.body, p, size);
        }
        OS_TASK_NOTIFY_FROM_ISR(hdls->ble_handle, BLE_APP_NOTIFY_CLING_PAY_CMD_MASK, eSetBits);
        return 0;

}
static int package_rev_fsm(uint8_t *p, uint16_t size)
{
        static uint8_t current_state = Z32_PACKAGE_BEGIN;
        static uint16_t body_size = 0;
        if (current_state != Z32_PACKAGE_HEADER_END) {
                for (int i = 0; i < size; i++) {
//                        printf("header----====+++++0x%02x current state = %d \r\n", p[i],
//                                current_state);
                        switch (current_state) {
                        case Z32_PACKAGE_NULL:
                                case Z32_PACKAGE_BEGIN:
                                case Z32_PACKAGE_HEADER_END:
                                if (p[i] == SYNC_HEAD) {
                                        body_size = 0;
                                        current_state = Z32_PACKAGE_TYPE;
                                }
                                break;
                        case Z32_PACKAGE_TYPE:
                                if (p[i] == SYNC_HEAD) {
                                        body_size = 0;
                                        current_state = Z32_PACKAGE_TYPE;
                                }
                                current_state = Z32_PACKAGE_BODY_SIZE_MSB;
                                break;
                        case Z32_PACKAGE_BODY_SIZE_MSB:
                                if (p[i] == SYNC_HEAD) {
                                        body_size = 0;
                                        current_state = Z32_PACKAGE_TYPE;
                                }
                                body_size = (p[i] << 8);
                                current_state = Z32_PACKAGE_BODY_SIZE_LSB;
                                break;

                        case Z32_PACKAGE_BODY_SIZE_LSB:
                                if (p[i] == SYNC_HEAD) {
                                        body_size = 0;
                                        current_state = Z32_PACKAGE_TYPE;
                                }
                                body_size |= p[i];
                                current_state = Z32_PACKAGE_UNSED_BYTE;
                                break;

                        case Z32_PACKAGE_UNSED_BYTE:
                                if (p[i] == SYNC_HEAD) {
                                        body_size = 0;
                                        current_state = Z32_PACKAGE_TYPE;
                                }
                                current_state = Z32_PACKAGE_HEADER_END;
                                if ((body_size - 1) > 0) {
//                                uart_dbg_hdl_serial2 = ad_uart_open(SERIAL2);
                                        ad_uart_read_async(uart_dbg_hdl_serial2, (char*)z32_rev_buf,
                                                body_size - 1, uart_read_cb, (void*)z32_rev_buf);
                                }
                                else {
                                        /*since data has been received corectly let us put device into sleep*/
                                        stop_z32_comm_activity();
                                        if (z32_rev_cb != NULL) {
                                                z32_rev_cb(NULL, 0);
                                        }
                                        current_state = Z32_PACKAGE_BEGIN;
                                }
                                break;
                        }

                }
        }
        else {
                stop_z32_comm_activity();
                if (z32_rev_cb != NULL) {
                        z32_rev_cb(z32_rev_buf, size);
                }
//                ble_z32_send_package_back(z32_rev_buf, size);
                current_state = Z32_PACKAGE_BEGIN;
                body_size = 0;
        }
        uint8_t header_remained = Z32_PACKAGE_HEADER_END - current_state;
        if (header_remained > 0) {
                ad_uart_read_async(uart_dbg_hdl_serial2, (char*)z32_rev_buf, header_remained,
                        uart_read_cb,
                        (void*)z32_rev_buf);

        }
        return 0;
}

static void uart_read_cb(void *user_data, uint16_t transferred)
{
        package_rev_fsm((uint8_t*)user_data, transferred);
}

void Clingpay_Init_UART()
{
        if (_uart2_init == 0) {
                ad_uart_init();
                uart_dbg_hdl_serial2 = ad_uart_open(SERIAL2);
                ad_uart_read_async(uart_dbg_hdl_serial2, (char*)z32_rev_buf, PCKAGE_SIZE,
                        uart_read_cb, (void*)z32_rev_buf);
                _uart2_init = 1;
                timer_cling_pay = OS_TIMER_CREATE("cling_pay_timer", OS_MS_2_TICKS(1000), pdTRUE, (void* )0,
                        cling_pay_cmd_timeout_cb);
                if (timer_cling_pay == NULL) {
                        return;
                }
        }
}

void Clingpay_Activate_ISO7816()
{
        char buf[PCKAGE_SIZE];

        if (_uart2_init == 0) {
                Clingpay_Init_UART();
                _uart2_init = 1;
        }

        buf[0] = SYNC_HEAD;
        buf[1] = ACTIVATE_ISO7816;
        buf[2] = 0x00;
        buf[3] = 0x01;
        buf[4] = 0x01;
        /*for the purpose to obbtian serail resorce*/
        ad_uart_abort_read_async(uart_dbg_hdl_serial2);
        ad_uart_bus_release(uart_dbg_hdl_serial2);
        ad_uart_write(uart_dbg_hdl_serial2, buf, sizeof(buf));
        ad_uart_read_async(uart_dbg_hdl_serial2, (char*)z32_rev_buf, PCKAGE_SIZE, uart_read_cb,
                (void*)z32_rev_buf);
        start_z32_comm_activity();

}

void Clingpay_Deactivate_ISO7816()
{
        char buf[PCKAGE_SIZE];
        int i = 0;
        if (_uart2_init == 0) {
                Clingpay_Init_UART();
                _uart2_init = 1;
        }

        buf[i++] = SYNC_HEAD;
        buf[i++] = DEACTIVATE_ISO7816;
        buf[i++] = 0x00;
        buf[i++] = 0x01;
        buf[i++] = 0x01;

        ad_uart_abort_read_async(uart_dbg_hdl_serial2);
        ad_uart_bus_release(uart_dbg_hdl_serial2);
        ad_uart_write(uart_dbg_hdl_serial2, buf, sizeof(buf));
        ad_uart_read_async(uart_dbg_hdl_serial2, (char*)z32_rev_buf, PCKAGE_SIZE, uart_read_cb,
                (void*)z32_rev_buf);
        start_z32_comm_activity();
        return;
}

void Clingpay_RETRIEVE_ATR(uint8_t *pATR, uint16_t *pATR_Len)
{
        char buf[PCKAGE_SIZE];
        if (_uart2_init == 0) {
                Clingpay_Init_UART();
                _uart2_init = 1;
        }

        buf[0] = SYNC_HEAD;
        buf[1] = RETRIEVE_ATR;
        buf[2] = 0x00;
        buf[3] = 0x01;
        buf[4] = 0x01;

        ad_uart_abort_read_async(uart_dbg_hdl_serial2);
        ad_uart_bus_release(uart_dbg_hdl_serial2);
        ad_uart_write(uart_dbg_hdl_serial2, buf, sizeof(buf));
        ad_uart_read_async(uart_dbg_hdl_serial2, (char*)z32_rev_buf, PCKAGE_SIZE, uart_read_cb,
                (void*)z32_rev_buf);
        start_z32_comm_activity();
}

void Clingpay_Send_APDU(uint8_t *pAPDU, uint16_t *pAPDU_Len)
{
        uint8_t buf[PCKAGE_SIZE - 1];

        volatile uint16_t apdu_length = *pAPDU_Len;

        if (apdu_length > 300) {        // APDU too large.
                return;
        }

        if (_uart2_init == 0) {
                Clingpay_Init_UART();
                _uart2_init = 1;
        }

        buf[0] = SYNC_HEAD;
        buf[1] = SEND_APDU;
        buf[2] = (apdu_length >> 8) & 0xFF;               //    / 256
        buf[3] = (apdu_length & 0xFF);                 // (apdu_length % 256);

        ad_uart_abort_read_async(uart_dbg_hdl_serial2);
        ad_uart_bus_release(uart_dbg_hdl_serial2);
        ad_uart_write(uart_dbg_hdl_serial2, (char*)buf, sizeof(buf));
        ad_uart_write(uart_dbg_hdl_serial2, (char*)pAPDU, *pAPDU_Len);
        ad_uart_read_async(uart_dbg_hdl_serial2, (char*)z32_rev_buf, PCKAGE_SIZE, uart_read_cb,
                (void*)z32_rev_buf);
        start_z32_comm_activity();
}

/*********************************************************************
 * FUNCTIONS
 */
static void clingpay_rev_data_call_back(uint8_t *p_buf, int num)
{
        uint8_t * buffer = p_buf;
        struct pay_rev_package rev;
//        memcpy(buffer, p_buf, num);
        memcpy(&rev, buffer, sizeof(rev.header));
        rev.header.data_lenth = SWAP_UINT16_T(rev.header.data_lenth);
        rev.header.cmd_type = SWAP_UINT16_T(rev.header.cmd_type);
        rev.header.ssc = SWAP_UINT16_T(rev.header.ssc);
        rev.data.body = buffer + sizeof(rev.header);
        ssc_record = rev.header.ssc;
        /*find pointer to crc segment*/
        uint8_t *p = rev.data.body + rev.header.data_lenth;
        rev.crc = (p[0] << 8 | p[1]);
        uint16_t crc = crc16_modbus(buffer, num - sizeof(uint16_t));
        crc = SWAP_UINT16_T(crc);
        Y_SPRINTF("[clinpay cmd] num = %d, cmd type = 0x%x, data lenth = %d indentification = 0x%x ssc = 0x%x crc = 0x%04x crc2 =0x%04x",num, rev.header.cmd_type, rev.header.data_lenth, rev.header.ind, rev.header.ssc, rev.crc, crc);
        if (crc != rev.crc) {
                Y_SPRINTF("[clinpay cmd] crc error");
                p = buffer;
                rev.header.ind = Z32_PCKAGE_SEND_IDENTIFICATION;
                rev.header.data_lenth = 0;
                rev.header.cmd_type = BTC_CRC_ERROR;
                memcpy(p, &rev, sizeof(rev.header));
                p += sizeof(rev.header);
                uint16_t crc = crc16_modbus(buffer, num - sizeof(uint16_t));
                memcpy(p, &crc, sizeof(crc));
                mac_send_payload((char*)buffer, (p - buffer));
                return;
        }
        memcpy(&shared_rev_pkg_mem, &rev, sizeof(rev));
        if (pkg_rev_cb != NULL) {
                /*each time when dialog get order from app, default z32 rev callback should be set*/
                restore_z32_rev_cb();
                /*pass to app layer*/
                pkg_rev_cb(&shared_rev_pkg_mem);
        }
        return;
}

static int clingpay_send()
{
//        uint8_t * buffer = tx_buffer;
//        struct pay_ack_package send_pkg;
        struct pay_rev_package rev;
        memcpy(&rev, &shared_rev_pkg_mem, sizeof(shared_rev_pkg_mem));
        uint8_t * buffer = rev.data.body - sizeof(shared_rev_pkg_mem.header);
        rev.header.ind = Z32_PCKAGE_SEND_IDENTIFICATION;
        if (ssc_record == 0) {
                ssc_record = rev.header.ssc;
        }
        else
        {
                rev.header.ssc = (ssc_record & SSC_MASK);
        }
        rev.header.data_lenth = SWAP_UINT16_T(rev.header.data_lenth);
        rev.header.cmd_type = SWAP_UINT16_T(rev.header.cmd_type);
        rev.header.ssc = SWAP_UINT16_T(rev.header.ssc);
        uint8_t *p = buffer;
        memcpy(p, &rev.header, sizeof(rev.header));
        p += sizeof(rev.header);
        memcpy(p, rev.data.body, SWAP_UINT16_T(rev.header.data_lenth));
        p += SWAP_UINT16_T(rev.header.data_lenth);
        rev.crc = crc16_modbus(buffer, p - buffer);
        memcpy(p, &rev.crc, sizeof(rev.crc));
//        rev.crc = SWAP_UINT16_T(rev.crc);
        p += sizeof(rev.crc);
        mac_send_payload((char*)buffer, (p - buffer));
        Y_SPRINTF("[clinpay cmd] cmd sended type = 0x%x, data lenth = %d indentification = 0x%x ssc = 0x%x", shared_rev_pkg_mem.header.cmd_type, shared_rev_pkg_mem.header.data_lenth, shared_rev_pkg_mem.header.ind, shared_rev_pkg_mem.header.ssc);
        return 0;
}

int clingpay_cmd_fsm()
{
        clingpay_send();
        return 0;
}

/******************************************************************************
 * FunctionName : clingpay_cmd_init
 * Description  : cling pay cmd layer init
 * Parameters   : none
 * Returns              : none
 *******************************************************************************/
int cling_pay_app_init(TASK_HANDLE_SETS *sets);
int clingpay_cmd_init(TASK_HANDLE_SETS *sets)
{
        hdls = sets;
        cling_pay_app_init(sets);
        Clingpay_Init_UART();
        set_recieved_data_call_back(clingpay_rev_data_call_back);
        /*make sure z32 is able to get into sleep sucessfully*/
        Clingpay_Deactivate_ISO7816();
        return 1;
}

static const uint8_t auchCRCHi[] =
        {
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
                0x81,
                0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
                0xC0,
                0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
                0x01,
                0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80,
                0x41,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
                0x81,
                0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
                0xC0,
                0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                0x01,
                0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
                0x40,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
                0x81,
                0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01,
                0xC0,
                0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
                0x01,
                0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
                0x41,
                0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
                0x81,
                0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01,
                0xC0,
                0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                0x01,
                0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
                0x41,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
                0x81,
                0x40
        };

static const uint8_t auchCRCLo[] =
        {
                0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5,
                0xC4,
                0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9,
                0x09,
                0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F,
                0xDD,
                0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13,
                0xD3,
                0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6,
                0xF7,
                0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA,
                0x3A,
                0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA,
                0xEE,
                0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6,
                0x26,
                0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3,
                0xA2,
                0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF,
                0x6F,
                0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79,
                0xBB,
                0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75,
                0xB5,
                0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90,
                0x91,
                0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C,
                0x5C,
                0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
                0x88,
                0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C,
                0x8C,
                0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81,
                0x80,
                0x40
        };

static uint16_t crc16_modbus(uint8_t *updata, uint16_t len)
{
        uint8_t uchCRCHi = 0xff;
        uint8_t uchCRCLo = 0xff;
        uint uindex;
        while (len--)
        {
                uindex = uchCRCHi ^ *updata++;
                uchCRCHi = uchCRCLo ^ auchCRCHi[uindex];
                uchCRCLo = auchCRCLo[uindex];
        }
        return (uchCRCHi << 8 | uchCRCLo);
}

void Clingpay_Retrieve_Balance(uint8_t *pAPDU, uint16_t *pAPDU_Len)
{
        char buf[PCKAGE_SIZE];
        *pAPDU_Len = 0;

        if (_uart2_init == 0) {
                Clingpay_Init_UART();
                _uart2_init = 1;
        }

        buf[0] = SYNC_HEAD;
        buf[1] = RETRIEVE_BALANCE;
        buf[2] = 0x00;
        buf[3] = 0x01;
        buf[4] = 0x00;
        ad_uart_abort_read_async(uart_dbg_hdl_serial2);
        ad_uart_bus_release(uart_dbg_hdl_serial2);
        ad_uart_write(uart_dbg_hdl_serial2, buf, sizeof(buf));
        ad_uart_read_async(uart_dbg_hdl_serial2, (char*)z32_rev_buf, PCKAGE_SIZE, uart_read_cb,
                (void*)z32_rev_buf);
        start_z32_comm_activity();
}

////    #if 0
////    static void _power_on_iso7816()
////    {
////      int i;
////      char buf[8];
////      char buf_ret[8];
////
////      for (i=0; i<8; i++) {
////        buf[i]     = 0;
////        buf_ret[i] = 0;
////      }
////
////      buf[0] = SYNC_HEAD;
////      buf[1] = (_command_id & 0x0f);
////      buf[2] = 0x00;
////      buf[3] = 0x01;
////      buf[4] = 0x01;
////
////      ad_uart_write(uart_dbg_hdl_serial2, buf, 5);
////      ad_uart_read(uart_dbg_hdl_serial2, buf_ret, 5, 100);
////
////      Y_SPRINTF("[CLING PAY] 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", buf_ret[0], buf_ret[1], buf_ret[2], buf_ret[3], buf_ret[4]);
////      _command_id++;
////    }
////
////    void clingpay_cmd_test()
////    {
////      char buf[16];
////
////      N_SPRINTF("[CLING PAY] test message 02.");
////
////      if (_uart2_init_==0) {
////        ad_uart_init();
////        uart_dbg_hdl_serial2 = ad_uart_open(SERIAL2);
////        _uart2_init_ = 1;
////      }
////
////      if (_uart2_init_!=0 && _activate_iso7816_!=0) {
////        N_SPRINTF("[CLING PAY] UART Init and ISO7816 Activated.");
////      }
////
////      if (_uart2_init_!=0) {
////    #if 0
////        buf[0]  = 0x11;
////        buf[1]  = 0x22;
////        buf[2]  = 0x33;
////        buf[3]  = 0x44;
////        buf[4]  = 0x55;
////        buf[5]  = 0x66;
////        buf[6]  = 0x77;
////        buf[7]  = 0x88;
////        buf[8]  = 0x99;
////        buf[9]  = 0xaa;
////        buf[10] = 0xbb;
////        buf[11] = 0xcc;
////        buf[12] = 0xdd;
////        buf[13] = 0xee;
////        buf[14] = 0xfe;
////        buf[15] = 0xff;
////
////        ad_uart_write(uart_dbg_hdl_serial2, buf, 16);
////    #endif
////      }
////
////    //if (_activate_iso7816_==0 && _uart2_init_!=0) {
////      if ( 1 ) {
////        _power_on_iso7816();
////        _activate_iso7816_ = 1;
////      }
////
////    }
////    #endif

#if 0
void Clingpay_Command_DES(uint8_t *pDES, uint16_t *pDES_Len)
{
        char buf[5];
        char buf_ret[5];

        uint16_t des_length = *pDES_Len;

        if (_uart2_init == 0) {
                Clingpay_Init_UART();
                _uart2_init = 1;
        }

        buf[0] = SYNC_HEAD;
        buf[1] = COMMAND_DES;
        buf[2] = ((des_length + 1) >> 8) & 0xFF;
        buf[3] = ((des_length + 1) & 0xFF);
        buf[4] = 0x00;

        ad_uart_write(uart_dbg_hdl_serial2, buf, 5);
        ad_uart_write(uart_dbg_hdl_serial2, pDES, des_length);

        buf_ret[0] = 0x00;
        buf_ret[1] = 0x00;
        buf_ret[2] = 0x00;
        buf_ret[3] = 0x00;
        buf_ret[4] = 0x00;

        ad_uart_read(uart_dbg_hdl_serial2, buf_ret, 5, 10);
        ad_uart_read(uart_dbg_hdl_serial2, pDES, 8, 10);
        *pDES_Len = 8;
}

void Clingpay_Command_TDES(uint8_t *pTDES, uint16_t *pTDES_Len)
{
        char buf[5];
        char buf_ret[5];

        uint16_t tdes_length = *pTDES_Len;

        if (_uart2_init == 0) {
                Clingpay_Init_UART();
                _uart2_init = 1;
        }

        buf[0] = SYNC_HEAD;
        buf[1] = COMMAND_TDES;
        buf[2] = ((tdes_length + 1) >> 8) & 0xFF;
        buf[3] = ((tdes_length + 1) & 0xFF);
        buf[4] = 0x00;

        ad_uart_write(uart_dbg_hdl_serial2, buf, 5);
        ad_uart_write(uart_dbg_hdl_serial2, pTDES, tdes_length);

        buf_ret[0] = 0x00;
        buf_ret[1] = 0x00;
        buf_ret[2] = 0x00;
        buf_ret[3] = 0x00;
        buf_ret[4] = 0x00;

        ad_uart_read(uart_dbg_hdl_serial2, buf_ret, 5, 10);
        ad_uart_read(uart_dbg_hdl_serial2, pTDES, 8, 10);
        *pTDES_Len = 8;
}

void Clingpay_Command_SHA1(uint8_t *pSHA, uint16_t *pSHA_Len)
{
        char buf[5];
        char buf_ret[5];

        uint16_t sha_length = *pSHA_Len;

        if (_uart2_init == 0) {
                Clingpay_Init_UART();
                _uart2_init = 1;
        }

        buf[0] = SYNC_HEAD;
        buf[1] = COMMAND_SHA1;
        buf[2] = ((sha_length + 1) >> 8) & 0xFF;
        buf[3] = ((sha_length + 1) & 0xFF);
        buf[4] = 0x00;

        ad_uart_write(uart_dbg_hdl_serial2, buf, 5);
        ad_uart_write(uart_dbg_hdl_serial2, pSHA, sha_length);

        buf_ret[0] = 0x00;
        buf_ret[1] = 0x00;
        buf_ret[2] = 0x00;
        buf_ret[3] = 0x00;
        buf_ret[4] = 0x00;

        ad_uart_read(uart_dbg_hdl_serial2, buf_ret, 5, 10);
        ad_uart_read(uart_dbg_hdl_serial2, pSHA, 20, 10);
        *pSHA_Len = 20;
}

void Clingpay_Command_Rand(uint8_t *pRND, uint16_t *pRND_Len)
{
        char buf[5];
        char buf_ret[5];

        uint16_t rnd_length = *pRND_Len;

        if (_uart2_init == 0) {
                Clingpay_Init_UART();
                _uart2_init = 1;
        }

        buf[0] = SYNC_HEAD;
        buf[1] = COMMAND_RAND;
        buf[2] = ((rnd_length + 1) >> 8) & 0xFF;
        buf[3] = ((rnd_length + 1) & 0xFF);
        buf[4] = 0x00;

        ad_uart_write(uart_dbg_hdl_serial2, buf, 5);
        ad_uart_write(uart_dbg_hdl_serial2, pRND, rnd_length);

        buf_ret[0] = 0x00;
        buf_ret[1] = 0x00;
        buf_ret[2] = 0x00;
        buf_ret[3] = 0x00;
        buf_ret[4] = 0x00;

        ad_uart_read(uart_dbg_hdl_serial2, buf_ret, 5, 10);

        rnd_length = buf_ret[2] * 256 + buf_ret[3];
        ad_uart_read(uart_dbg_hdl_serial2, pRND, rnd_length, 10);
        *pRND_Len = rnd_length;
}

void Clingpay_Command_RSA_Generate(uint8_t *pRSA, uint16_t *pRSA_Len)
{
        char buf[5];
        char buf_ret[5];

        uint16_t rsa_length = *pRSA_Len;

        if (_uart2_init == 0) {
                Clingpay_Init_UART();
                _uart2_init = 1;
        }

        buf[0] = SYNC_HEAD;
        buf[1] = COMMAND_RSA_GEN;
        buf[2] = ((rsa_length + 1) >> 8) & 0xFF;
        buf[3] = ((rsa_length + 1) & 0xFF);
        buf[4] = 0x00;

        ad_uart_write(uart_dbg_hdl_serial2, buf, 5);
        ad_uart_write(uart_dbg_hdl_serial2, pRSA, rsa_length);

        buf_ret[0] = 0x00;
        buf_ret[1] = 0x00;
        buf_ret[2] = 0x00;
        buf_ret[3] = 0x00;
        buf_ret[4] = 0x00;

        ad_uart_read(uart_dbg_hdl_serial2, buf_ret, 5, 10);

        rsa_length = buf_ret[2] * 256 + buf_ret[3];
        ad_uart_read(uart_dbg_hdl_serial2, pRSA, rsa_length, 10);
        *pRSA_Len = rsa_length;
}

void Clingpay_Command_RSA_Calculate(uint8_t *pRSA, uint16_t *pRSA_Len)
{
        char buf[5];
        char buf_ret[5];

        uint16_t rsa_length = *pRSA_Len;

        if (_uart2_init == 0) {
                Clingpay_Init_UART();
                _uart2_init = 1;
        }

        buf[0] = SYNC_HEAD;
        buf[1] = COMMAND_RSA_CAL;
        buf[2] = ((rsa_length + 1) >> 8) & 0xFF;
        buf[3] = ((rsa_length + 1) & 0xFF);
        buf[4] = 0x00;

        ad_uart_write(uart_dbg_hdl_serial2, buf, 5);
        ad_uart_write(uart_dbg_hdl_serial2, pRSA, rsa_length);

        buf_ret[0] = 0x00;
        buf_ret[1] = 0x00;
        buf_ret[2] = 0x00;
        buf_ret[3] = 0x00;
        buf_ret[4] = 0x00;

        ad_uart_read(uart_dbg_hdl_serial2, buf_ret, 5, 10);

        rsa_length = buf_ret[2] * 256 + buf_ret[3];
        ad_uart_read(uart_dbg_hdl_serial2, (char*)pRSA, rsa_length, 10);
        *pRSA_Len = rsa_length;
}

void Clingpay_Command_MAC(uint8_t *pMAC, uint16_t *pMAC_Len)
{
        char buf[5];
        char buf_ret[5];

        uint16_t mac_length = *pMAC_Len;

        if (_uart2_init == 0) {
                Clingpay_Init_UART();
                _uart2_init = 1;
        }

        buf[0] = SYNC_HEAD;
        buf[1] = COMMAND_MAC;
        buf[2] = ((mac_length + 1) >> 8) & 0xFF;
        buf[3] = ((mac_length + 1) & 0xFF);
        buf[4] = 0x00;

        ad_uart_write(uart_dbg_hdl_serial2, buf, 5);
        ad_uart_write(uart_dbg_hdl_serial2, pMAC, mac_length);

        buf_ret[0] = 0x00;
        buf_ret[1] = 0x00;
        buf_ret[2] = 0x00;
        buf_ret[3] = 0x00;
        buf_ret[4] = 0x00;

        ad_uart_read(uart_dbg_hdl_serial2, buf_ret, 5, 10);
        ad_uart_read(uart_dbg_hdl_serial2, pMAC, 8, 10);

        *pMAC_Len = 8;
}

void Clingpay_Command_HMAC(uint8_t *pHMAC, uint16_t *pHMAC_Len)
{
        char buf[5];
        char buf_ret[5];

        uint16_t hmac_length = *pHMAC_Len;

        if (_uart2_init == 0) {
                Clingpay_Init_UART();
                _uart2_init = 1;
        }

        buf[0] = SYNC_HEAD;
        buf[1] = COMMAND_HMAC;
        buf[2] = ((hmac_length + 1) >> 8) & 0xFF;
        buf[3] = ((hmac_length + 1) & 0xFF);
        buf[4] = 0x00;

        ad_uart_write(uart_dbg_hdl_serial2, buf, 5);
        ad_uart_write(uart_dbg_hdl_serial2, pHMAC, hmac_length);

        buf_ret[0] = 0x00;
        buf_ret[1] = 0x00;
        buf_ret[2] = 0x00;
        buf_ret[3] = 0x00;
        buf_ret[4] = 0x00;

        ad_uart_read(uart_dbg_hdl_serial2, buf_ret, 5, 10);
        ad_uart_read(uart_dbg_hdl_serial2, pHMAC, 20, 10);

        *pHMAC_Len = 20;
}
#endif
