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
#include "ble_pay_mac.h"
#include "ble_pay_cmd.h"
#include "ble_pay_app.h"
#include "ble_pay_service.h"
#include "stdio.h"
#include "assert.h"
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "OSAL.h"
/*********************************************************************
 * MACROS
 */
#define Z32_PCKAGE_REV_IDENTIFICATION  0X6F
#define Z32_PCKAGE_SEND_IDENTIFICATION  0X80
#define COMPOSE_16BITS_DATA(_X, _Y) (((uint16_t)_X)<<8 | ((uint16_t)_Y))
#define SWAP_UINT16_T(__X)((((__X) & 0x00FF) << 8) | (((__X) & 0xFF00) >> 8))

#define PCKAGE_SIZE 5
#define BUFFER_SIZE 32
enum{
		BLE_PAY_PACKAGE_INIT = 0,
		BLE_PAY_PACKAGE_ORG,
		BLE_PAY_PACKAGE_REV,
		BLE_PAY_CRC_ERROR_PROCESS,
		BLE_PAY_CMD_PROCESS
};
static int curret_state = BLE_PAY_PACKAGE_INIT;
#define FSM_STATE_SWITCH_TO(X) do{curret_state = X;}while(0)
#define FSM_STATE_SWITCH_TO_WITH_NOTIFY(X) do{curret_state = X; OS_TASK_NOTIFY_FROM_ISR(OS_GET_CURRENT_TASK(), BLE_APP_NOTIFY_CLING_PAY_CMD_MASK, OS_NOTIFY_SET_BITS);}while(0)
#define GET_CURRENT_FSM_STATE() (curret_state)


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
static struct pay_rev_package shared_rev_pkg_mem __attribute__((unused));
#define SSC_MASK  0X0FFF
/*********************************************************************
 * static VARIABLES
 */
//static int (*z32_rev_cb)(uint8_t*, uint16_t) = NULL;
static int (*pkg_rev_cb)(struct pay_rev_package*) = NULL;
//static uint8_t _uart2_init = 0;
static uint16_t ssc_record = 0;
//static uint8_t tx_buffer[512];
//static uint8_t rx_buffer[512];

//static uint8_t z32_rev_buf[300];
//static void uart_read_cb(void *user_data, uint16_t transferred);
static int clingpay_send(uint8_t *data, size_t length);
static uint16_t crc16_modbus(uint8_t *updata, uint16_t len);
//static int ble_z32_send_package_back(uint8_t *p, uint16_t size);
static void clingpay_rev_data_call_back(uint8_t *p_buf, int num);
static void raw_data_process(uint8_t *p_buf, uint32_t num);
struct bytes{
	uint8_t *p;
	uint16_t size;
};

static struct bytes pbuffer_g;
/*********************************************************************
 * EXTERNAL VARIABLES
 */


int cling_pay_cmd_init()
{
			memset(&pbuffer_g, 0, sizeof(pbuffer_g));
			set_recieved_data_call_back(clingpay_rev_data_call_back);
      cling_pay_mac_init();
			return 1;
}

int reg_cling_pay_cmd_callback(int (*cb)(struct pay_rev_package*))
{
        if (cb != NULL) {
                pkg_rev_cb = cb;
								pkg_rev_cb = pkg_rev_cb;
        }
        return 0;
}


int cling_pay_fsm(void *data)
{
	
		int one_more_loop = false;
	  N_SPRINTF("[cling_pay_fsm] entered");
		switch(GET_CURRENT_FSM_STATE()){
			case BLE_PAY_PACKAGE_INIT:break;
			case BLE_PAY_PACKAGE_ORG:
				raw_data_process(pbuffer_g.p, pbuffer_g.size);
				one_more_loop = true;
				break;
			case BLE_PAY_CRC_ERROR_PROCESS:
					if(pbuffer_g.size > 0){
							mac_send_payload((char*)pbuffer_g.p, pbuffer_g.size);
							memset(&pbuffer_g, 0, sizeof(pbuffer_g));
					}
					FSM_STATE_SWITCH_TO(BLE_PAY_PACKAGE_INIT);
			break;
			case BLE_PAY_CMD_PROCESS:
					Y_SPRINTF("[cling_pay_fsm] data recieved");
#if 0
				  if (pkg_rev_cb != NULL) {
                /*each time when dialog get order from app, default z32 rev callback should be set*/
                //restore_z32_rev_cb();
                /*pass to app layer*/
								struct pay_rev_package rev;
								memcpy(&rev, pbuffer_g.p, pbuffer_g.size);
                pkg_rev_cb(&rev);
        }
#endif
					CLASS(cling_pay_app)*p =cling_pay_app_get_instance();
						/*send event to 1280 mgr*/
					p->send_event_from_app(p, pbuffer_g.p, pbuffer_g.size);
					memset(&pbuffer_g, 0, sizeof(pbuffer_g));
					FSM_STATE_SWITCH_TO(BLE_PAY_PACKAGE_INIT);
				  break;
		} 
		 N_SPRINTF("[cling_pay_fsm] leaved");
	  return one_more_loop;
}


static void raw_data_process(uint8_t *p_buf, uint32_t num)
{
        uint8_t * buffer = p_buf;
				static struct pay_rev_package rev;
					Y_SPRINTF("[clinpay cmd]raw_datalenth = %d", num);
	
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
                N_SPRINTF("[clinpay cmd] crc error");
                p = buffer;
                rev.header.ind = Z32_PCKAGE_SEND_IDENTIFICATION;
                rev.header.data_lenth = 0;
                rev.header.cmd_type = BTC_CRC_ERROR;
                memcpy(p, &rev, sizeof(rev.header));
                p += sizeof(rev.header);
                uint16_t crc = crc16_modbus(buffer, num - sizeof(uint16_t));
                memcpy(p, &crc, sizeof(crc));
								pbuffer_g.p = buffer;
								pbuffer_g.size = p - buffer;
								FSM_STATE_SWITCH_TO(BLE_PAY_CRC_ERROR_PROCESS);
                //mac_send_payload((char*)buffer, (p - buffer));
                return;
       }
				pbuffer_g.p = (void*)&rev;
				pbuffer_g.size = sizeof(rev);
#if 0
				for(int i=0; i < ((struct pay_rev_package*)pbuffer_g.p)->header.data_lenth; i++){
					Y_SPRINTF("0x%02x", ((struct pay_rev_package*)pbuffer_g.p)->data.body[i]);
				}
#endif
				FSM_STATE_SWITCH_TO(BLE_PAY_CMD_PROCESS);
        return;

}
/*********************************************************************
 * FUNCTIONS
 */
static void clingpay_rev_data_call_back(uint8_t *p_buf, int num)
{
				pbuffer_g.p = p_buf;
				pbuffer_g.size = num;
				FSM_STATE_SWITCH_TO_WITH_NOTIFY(BLE_PAY_PACKAGE_ORG);
}

static int clingpay_send(uint8_t *data, size_t length)
{
//        uint8_t * buffer = tx_buffer;
//        struct pay_ack_package send_pkg;
        struct pay_rev_package rev;
        memcpy(&rev, data, sizeof(rev));
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
				Y_SPRINTF("[clinpay cmd]crc16");
        rev.crc = crc16_modbus(buffer, p - buffer);
        memcpy(p, &rev.crc, sizeof(rev.crc));
//        rev.crc = SWAP_UINT16_T(rev.crc);
        p += sizeof(rev.crc);
        mac_send_payload((char*)buffer, (p - buffer));
        Y_SPRINTF("[clinpay cmd] cmd sended type = 0x%x, data lenth = %d indentification = 0x%x ssc = 0x%x", shared_rev_pkg_mem.header.cmd_type, shared_rev_pkg_mem.header.data_lenth, shared_rev_pkg_mem.header.ind, shared_rev_pkg_mem.header.ssc);
        return 0;
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
        uint32_t uindex;
        while (len--)
        {
                uindex = uchCRCHi ^ *updata++;
                uchCRCHi = uchCRCLo ^ auchCRCHi[uindex];
                uchCRCLo = auchCRCLo[uindex];
        }
        return (uchCRCHi << 8 | uchCRCLo);
}

int clingpay_cmd_fsm()
{
       // clingpay_send();
        return 0;
}


static const struct ble_pay_adapter_call_backs local_cb = {
			 .init = cling_pay_cmd_init,
			 .core_fsm = cling_pay_fsm,
			 .rx_cb = NULL
};
/******************************************************************************
 * FunctionName : clingpay_cmd_init
 * Description  : cling pay cmd layer init
 * Parameters   : none
 * Returns              : none
 *******************************************************************************/
int clingpay_cmd_init()
{ 
				
				CLASS(cling_pay_app)*p =cling_pay_app_get_instance();
				p->reg_adaptor(p, (struct ble_pay_adapter_call_backs *)&local_cb);
				p->reg_ble_tx(p, clingpay_send);
				memset(&pbuffer_g, 0, sizeof(pbuffer_g));
				set_recieved_data_call_back(clingpay_rev_data_call_back);
				cling_pay_mac_init();
				return 1;
}

