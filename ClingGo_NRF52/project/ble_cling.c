/**
 ****************************************************************************************
 *
 * @file ble_cling_service.c
 *
 * @brief ble_cling_service protocol file
 *
 * Copyright (C) 2015. hicling elec Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <black.orca.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "osal.h"
#include "ble_common.h"
#include "ble_gap.h"
#include "ble_gatts.h"
#include "stdio.h"
#include "main.h"
#include <btle_api.h>
#include <ble_cus.h>
#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * MACROS
 */
#define DEF_HICLING_NOTIFY1_POINT_USER_DESC     "NOTIFY1_POINT"
#define DEF_HICLING_NOTIFY2_POINT_USER_DESC     "NOTIFY2_POINT"
#define DEF_HICLING_NOTIFY3_POINT_USER_DESC     "NOTIFY3_POINT"
#define DEF_HICLING_NOTIFY4_POINT_USER_DESC     "NOTIFY4_POINT"

#define DEF_HICLING_WRITE1_POINT_USER_DESC     "WRITE1_POINT"
#define DEF_HICLING_WRITE2_POINT_USER_DESC     "WRITE2_POINT"
#define DEF_HICLING_WRITE3_POINT_USER_DESC     "WRITE3_POINT"
#define DEF_HICLING_WRITE4_POINT_USER_DESC     "WRITE4_POINT"

#define CONFIG_BLE_DLGDEBUG_MAX_CP_LEN 64

#define HANDLE_STEPS_EVEY_VALUE_WITH_NTFPRO ble_gatts_get_num_attr(0,1,1)
#define HANDLE_STEPS_EVEY_VALUE             ble_gatts_get_num_attr(0,1,0)

/*********************************************************************
 * TYPEDEFS
 */

enum {
        HICLING_IDX_SVC = 0,

        HICLING_IDX_NOTIFY1_POINT_CHAR,
        HICLING_IDX_NOTIFY1_POINT_VAL,
        HICLING_IDX_NOTIFY1_POINT_CFG,
        HICLING_IDX_NOTIFY1_POINT_USER_DESC,

        HICLING_IDX_NOTIFY2_POINT_CHAR,
        HICLING_IDX_NOTIFY2_POINT_VAL,
        HICLING_IDX_NOTIFY2_POINT_CFG,
        HICLING_IDX_NOTIFY2_POINT_USER_DESC,

        HICLING_IDX_NOTIFY3_POINT_CHAR,
        HICLING_IDX_NOTIFY3_POINT_VAL,
        HICLING_IDX_NOTIFY3_POINT_CFG,
        HICLING_IDX_NOTIFY3_POINT_USER_DESC,

        HICLING_IDX_NOTIFY4_POINT_CHAR,
        HICLING_IDX_NOTIFY4_POINT_VAL,
        HICLING_IDX_NOTIFY4_POINT_CFG,
        HICLING_IDX_NOTIFY4_POINT_USER_DESC,

        HICLING_IDX_WRITE1_POINT_CHAR,
        HICLING_IDX_WRITE1_POINT_VAL,
        HICLING_IDX_WRITE1_POINT_USER_DESC,

        HICLING_IDX_WRITE2_POINT_CHAR,
        HICLING_IDX_WRITE2_POINT_VAL,
        HICLING_IDX_WRITE2_POINT_USER_DESC,

        HICLING_IDX_WRITE3_POINT_CHAR,
        HICLING_IDX_WRITE3_POINT_VAL,
        HICLING_IDX_WRITE3_POINT_USER_DESC,

        HICLING_IDX_WRITE4_POINT_CHAR,
        HICLING_IDX_WRITE4_POINT_VAL,
        HICLING_IDX_WRITE4_POINT_USER_DESC,

        HICLING_IDX_NB
};

enum {
        HICLING_SERVICE_UUID_16 = 0XFFE0,
        HICLING_NOTIFY1_POINT_UUID_16 = 0xFFE1,
        HICLING_NOTIFY2_POINT_UUID_16,
        HICLING_NOTIFY3_POINT_UUID_16,
        HICLING_NOTIFY4_POINT_UUID_16,
        HICLING_NOTIFY_END_POINT_UUID_16,

        HICLING_WRITE1_POINT_UUID_16 = 0xFFE5,
        HICLING_WRITE2_POINT_UUID_16,
        HICLING_WRITE3_POINT_UUID_16,
        HICLING_WRITE4_POINT_UUID_16,
        HICLING_WRITE_END_POINT_UUID_16,

};

typedef struct {
        ble_service_t svc;
				uint16_t service_handle;
				uint16_t conn_handle;
        // handles
        uint16_t cp_val_h[HICLING_WRITE_END_POINT_UUID_16 - HICLING_NOTIFY1_POINT_UUID_16];
        uint16_t cp_ccc_h[HICLING_NOTIFY_END_POINT_UUID_16 - HICLING_NOTIFY1_POINT_UUID_16];
				ble_gatts_char_handles_t     cling_handles[HICLING_WRITE_END_POINT_UUID_16 - HICLING_NOTIFY1_POINT_UUID_16];          /**< Handles related to the Battery Level characteristic. */
        void *handlers;
} cling_service_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
/**@brief Structure for holding the data that will be transmitted to the connected central.
 */

typedef enum {
    READ_REQ,      /**< Type identifying that this tx_message is a read request. */
    WRITE_REQ      /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

typedef struct {
    uint8_t                  gattc_value[20];  /**< The message to write. */
    ble_gattc_write_params_t gattc_params;                       /**< The GATTC parameters for this message. */
} write_params_t;

typedef struct {
    uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
    tx_request_t type;         /**< Type of message. (read or write). */
    union {
        uint16_t       read_handle;  /**< Read request handle. */
        write_params_t write_req;    /**< Write request message. */
    } req;
} tx_message_t;
#define TX_BUFFER_MASK 0x0a


#define IS_BUIFFER_LOCKED() (seam_buffer == true)
#define LOCK_BUFFER() (seam_buffer = true)
#define UNLOCK_BUFFER() (seam_buffer = false)
	

//static ble_pay_c_t * mp_ble_bas_c;                 /**< Pointer to the current instance of the BAS Client module. The memory for this is provided by the application.*/
static tx_message_t  m_tx_buffer[TX_BUFFER_MASK + 1];  /**< Transmit buffer for the messages that will be transmitted to the central. */
static uint32_t      m_tx_insert_index = 0;        /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t      m_tx_index        = 0;        /**< Current index in the transmit buffer containing the next message to be transmitted. */

static  uint16_t seam_buffer = false;
static  cling_service_t svc;
static  TASK_HANDLE_SETS *task_handles;
static int (*ble_write_callback_p)(ble_service_t *svc, uint16_t conn_idx, uint16_t uuid_num,
        uint8_t *s, size_t size) = NULL;
static int (*ble_on_conn_callback_p)(ble_service_t *svc, uint16_t conn_idx) = NULL;
static int (*ble_on_disconn_callback_p)() = NULL;
static void _on_ble_evt(ble_evt_t * p_ble_evt);
static const uint16_t uuid_check_table[] = {
        /*0*3 OFFSET HANDLE*/
        HICLING_NOTIFY1_POINT_UUID_16,
        0,
        0,
        /*183 OFFSET HANDLE*/
        HICLING_NOTIFY2_POINT_UUID_16,
        0,
        0,

        /*2*3 OFFSET HANDLE*/
        HICLING_NOTIFY3_POINT_UUID_16,
        0,
        0,

        /*3*3 OFFSET HANDLE*/
        HICLING_NOTIFY4_POINT_UUID_16,
        0,
        0,

        /*4*2 OFFSET HANDLE*/
        HICLING_WRITE1_POINT_UUID_16,
        0,
        /*5*2 OFFSET HANDLE*/
        HICLING_WRITE2_POINT_UUID_16,
        0,
        /*6*2 OFFSET HANDLE*/
        HICLING_WRITE3_POINT_UUID_16,
        0,
        /*7*2 OFFSET HANDLE*/
        HICLING_WRITE4_POINT_UUID_16,
        0,

};
/*********************************************************************
 * LOCAL DECLARATION
 *
 */
static void handle_write_req(uint16_t handle, uint8_t *p_data, uint16_t size);
static void handle_conection_req(uint16_t conn_idx);
static void hanlde_disconnected_evt_t(void);
static void tx_buffer_process(cling_service_t * p_bas);
static void tx_send(uint16_t handle_num, uint8_t *str , uint16_t len);

int cling_service_reg_write_callback(int (*p)(ble_service_t *svc, uint16_t conn_idx, uint16_t uuid_num, uint8_t *s, size_t size))
{
		    if (p != NULL) {
                ble_write_callback_p = p;
        }
        return BLE_STATUS_OK;

}


int cling_service_reg_conn_callback(int (*p)(ble_service_t *svc, uint16_t conn_idx))
{
	
			
        if (p != NULL) {
                ble_on_conn_callback_p = p;
        }
        return BLE_STATUS_OK;

}

int cling_service_reg_disconn_callback(int (*p)(void))
{
        if (p != NULL) {
                ble_on_disconn_callback_p = p;
        }
        return BLE_STATUS_OK;

}

ble_service_t *cling_service_init(const ble_service_config_t *cfg, TASK_HANDLE_SETS *sets)
{

    task_handles = sets;
	  uint32_t            err_code = NRF_SUCCESS;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_battery_level[20] __attribute__((unused));
		cling_service_t *dbg = &svc;
//    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
//    uint8_t             init_len;
    int i = 0;
	
	
	    // Here the sec level for the Battery Service can be changed/increased.
//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.write_perm);
	
	
	
	    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, HICLING_SERVICE_UUID_16);
    /*16bytes */
    

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &(dbg->service_handle));
	
    memset(&cccd_md, 0, sizeof(cccd_md));

    // According to BAS_SPEC_V10, the read operation on cccd should be possible without
   // authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write =  1;
    char_md.char_props.read   = 0;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; //NULL
    char_md.p_sccd_md         = NULL;

    //BLE_UUID_BLE_ASSIGN(ble_uuid, UICO_LEN_NOTIFY_UUID);
    //ble_uuid.uuid = 0;
    //ble_uuid.type = BLE_UUID_TYPE_BLE;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20;
    attr_char_value.p_value   = NULL;//initial_battery_level;
		
		 for (uint32_t i = HICLING_NOTIFY1_POINT_UUID_16; i < HICLING_WRITE_END_POINT_UUID_16; i++) {
			 BLE_UUID_BLE_ASSIGN(ble_uuid, i);
			 if(i >= HICLING_NOTIFY_END_POINT_UUID_16 ){
								char_md.char_props.notify = 0;
								char_md.p_cccd_md         = NULL; //NULL
			 }
			 err_code = sd_ble_gatts_characteristic_add(dbg->service_handle, &char_md,
               &attr_char_value,
               &dbg->cling_handles[i - HICLING_NOTIFY1_POINT_UUID_16]);			 
        }
      ble_on_event_process_reg(_on_ble_evt);
   
}

static void hanlde_disconnected_evt_t()
{
				svc.conn_handle = 0xff;
        if (ble_on_disconn_callback_p != NULL) {
                ble_on_disconn_callback_p();
        }
}

static void handle_conection_req(uint16_t conn_idx)
{
				ble_service_t *dbgs = &svc.svc;
				svc.conn_handle = conn_idx;
        if (ble_on_conn_callback_p != NULL) {
                ble_on_conn_callback_p(dbgs, conn_idx);
        }
        return;

}

static void handle_write_req(uint16_t handle, uint8_t *p_data, uint16_t size)
{
        cling_service_t *dbgs = (cling_service_t *)&svc;
				ble_service_t *ble_dbgs = &svc.svc;
        Y_SPRINTF("[ble cust]: handle write handle = %d number = %d\r\n", handle, sizeof(dbgs->cp_ccc_h) / sizeof(dbgs->cp_ccc_h[0]));

        /*check for notification enable event*/
        for (int i = 0; i < (HICLING_NOTIFY_END_POINT_UUID_16 - HICLING_NOTIFY1_POINT_UUID_16); i++) {
                Y_SPRINTF("[ble cust]: current ccc hanle value = %d\r\n", dbgs->cling_handles[i].cccd_handle);
                if (handle == dbgs->cling_handles[i].cccd_handle) {

                        return;
                }

        }
        /*check for value write event*/
        for (int i = 0; i < (HICLING_WRITE_END_POINT_UUID_16 - HICLING_NOTIFY1_POINT_UUID_16); i++) {
//                N_SPRINTF("[ble cust]: current  cp_val_h value = %d\r\n", dbgs->cp_val_h[i]);
                if (handle == dbgs->cling_handles[i].value_handle) {
                        Y_SPRINTF("[ble cust]: current  cp_val_h value = %d\r\n", dbgs->cling_handles[i].value_handle);
                        /*pass data to user space*/
                        if (ble_write_callback_p != NULL) {
                                ble_write_callback_p(ble_dbgs, dbgs->conn_handle,
                                        uuid_check_table[handle - dbgs->cling_handles[0].value_handle],
                                        (uint8_t*)p_data, size);
                        }
                        Y_SPRINTF("[ble cust]: handle write data = %d lenth %d uuid = 0x%04x\r\n",
                                p_data[0], size, uuid_check_table[handle - dbgs->cling_handles[0].value_handle]);
                       // status = ATT_ERROR_OK;
                }
        }
}



ble_error_t cus_notify_send(uint16_t conn_idx, uint16_t uuid_num, uint8_t *t,
        size_t size)
{
        cling_service_t *dbgs = (cling_service_t *)&svc;
//        ble_storage_get_u16(conn_idx, dbgs->cp_ccc_h[uuid_num - HICLING_NOTIFY1_POINT_UUID_16],
//                &ccc_val);
//
//        if (!(ccc_val & GATT_CCC_NOTIFICATIONS)) {
//                N_SPRINTF("ccd not enabled\r\n");
//                return BLE_ERROR_FAILED;
//        }
        //ble_gatts_set_value(dbgs->cp_val_h[uuid_num - HICLING_NOTIFY1_POINT_UUID_16], size, t);
				tx_send(dbgs->cling_handles[uuid_num - HICLING_NOTIFY1_POINT_UUID_16].value_handle, t , size);
        return 1;
}

static void on_write(ble_evt_t * p_ble_evt)
{
			 ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
			 handle_write_req(p_evt_write->handle, p_evt_write->data,  p_evt_write->len);
			
}		


/*********************************************************************
 * @fn      tx_send
 *
 * @brief   send message
 *
 * @param   task_id,message_type.
 *
 * @return  LX_OK ,LX_OK
 */
static void tx_send(uint16_t handle_num, uint8_t *str , uint16_t len)
{
    tx_message_t * p_msg;
		cling_service_t *dbgs = (cling_service_t *)&svc;
    if(len > 20) {
        return ;
    }
#if 0
    /*if buffer is locked no operation should take place*/
    if(IS_BUIFFER_LOCKED()) {
        return;
    }
#endif
    /*lock buffer to occupy it*/
    LOCK_BUFFER();
    //????
    p_msg = &m_tx_buffer[m_tx_insert_index++];
		   if(m_tx_insert_index >= TX_BUFFER_MASK){
				m_tx_insert_index = 0;
		}
    //m_tx_insert_index &= TX_BUFFER_MASK;
    //????
    p_msg->req.write_req.gattc_params.handle = handle_num;

    
    p_msg->req.write_req.gattc_params.len = len;
    //??
    p_msg->req.write_req.gattc_params.p_value = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset = 0;
    //???
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    //????
    p_msg->conn_handle = dbgs->conn_handle;
    //???????
    p_msg->type = WRITE_REQ;

    memcpy(p_msg->req.write_req.gattc_value, str, len);
    UNLOCK_BUFFER();
    //connected_pc_tool
    //????
		Y_SPRINTF("[ble uico]: m_tx_insert_index = %d", m_tx_insert_index);
    tx_buffer_process(dbgs);
}


/*********************************************************************
 * @fn      send_message
 *
 * @brief   This function is used for sending messge to specific task.
 *
 * @param   task_id,message_type.
 *
 * @return  LX_OK ,LX_OK
 */

static void tx_buffer_process(cling_service_t * p_bas)
{
    if(m_tx_index != m_tx_insert_index) {
        uint32_t err_code;

        if(m_tx_buffer[m_tx_index].type == READ_REQ) {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        } else {
            /*
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
            */
            /*if buffer is locked*/
            if(IS_BUIFFER_LOCKED()) {
								N_SPRINTF("[ble_pay]: buffer is locked");
                return;
            }
            LOCK_BUFFER();
            {
                ble_gatts_value_t gatts_value;
                // Initialize value struct.
                memset(&gatts_value, 0, sizeof(gatts_value));

                gatts_value.len     = m_tx_buffer[m_tx_index].req.write_req.gattc_params.len;
                gatts_value.offset  = m_tx_buffer[m_tx_index].req.write_req.gattc_params.offset;
                gatts_value.p_value = m_tx_buffer[m_tx_index].req.write_req.gattc_value;

#if 1
                // Update database.
                err_code = sd_ble_gatts_value_set(m_tx_buffer[m_tx_index].conn_handle,
                                                  m_tx_buffer[m_tx_index].req.write_req.gattc_params.handle,
                                                  &gatts_value);


                if(err_code != NRF_SUCCESS) {
										N_SPRINTF("[ble_pay]:value updated failed ");
                    return ;
                }
#endif
                ble_gatts_hvx_params_t hvx_params;

                memset(&hvx_params, 0, sizeof(hvx_params));

                hvx_params.handle = m_tx_buffer[m_tx_index].req.write_req.gattc_params.handle;
                hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
                hvx_params.offset = 0;//20-lenth-1;//gatts_value.offset;
                hvx_params.p_len  = &(gatts_value.len);
                hvx_params.p_data = gatts_value.p_value;
                Y_SPRINTF("[ble_pay]: data lenth = %d..\r\n", *(hvx_params.p_len));
                err_code = sd_ble_gatts_hvx(m_tx_buffer[m_tx_index].conn_handle, &hvx_params);
                //err_code = sd_ble_gatts_hvx(m_tx_buffer[m_tx_index].conn_handle, &hvx_params);
            }
        }

        if(err_code == NRF_SUCCESS) {
            N_SPRINTF("[ble_pay]: SD Read/Write API returns Success..\r\n");
            m_tx_index++;
				  	Y_SPRINTF("[ble_pay]: tx sucess m_tx_index = %d", m_tx_index);
            if(m_tx_index >= TX_BUFFER_MASK){
								m_tx_index = 0;
						}
            UNLOCK_BUFFER();
            //tx_buffer_process(p_bas);
        } else {
            UNLOCK_BUFFER();
					  //app_timer_start(ble_pay_timer_id, SYSCLK_INTERVAL_10MS, p_bas);
            N_SPRINTF("[ble_pay]: SD Read/Write API returns error. This message sending will be "
                      "attempted again..");
        }
    } else {
				N_SPRINTF("[ble_pay]: tx buffer empty");
        /*notify up layer the message that data has been sended successfully*/
      //  ble_pay_evt_t evt;
     //   evt.evt_type = ble_pay_EVT_BUFFER_SENDED_SUCESSFULLY;
     //   if(p_bas->evt_handler != NULL) {
     //      p_bas->evt_handler(p_bas, &evt);
     //   }
    }
}


static void _on_ble_evt(ble_evt_t * p_ble_evt)
{
	BLE_CTX *r = &cling.ble;

	switch (p_ble_evt->header.evt_id)
	{
		case BLE_EVT_TX_COMPLETE:
			//r->tx_buf_available += (I8U) p_ble_evt->evt.common_evt.params.tx_complete.count;
			tx_buffer_process(NULL);
			break;

		case BLE_GAP_EVT_CONNECTED:
			//_on_connect(p_ble_evt);
			handle_conection_req( p_ble_evt->evt.gap_evt.conn_handle);
			break;
					
		case BLE_GAP_EVT_DISCONNECTED:	
			hanlde_disconnected_evt_t();
			break;
					
		case BLE_GATTS_EVT_WRITE:
			on_write(p_ble_evt);
			//on_write(uint16_t handle, uint8_t *p_data, uint16_t size)
			break;

		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
			break;

		case BLE_GAP_EVT_AUTH_STATUS:
			//_BTLE_LOG("[BLE] AUTH GAP EVT - write CCCD");
			break;

		case BLE_GAP_EVT_TIMEOUT:

			if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
			{ 
				if (r->btle_State == BTLE_STATE_ADVERTISING) {
					if ((r->adv_mode == BLE_ADV_SLEEP) || (!cling.system.b_powered_up)) {
						r->btle_State = BTLE_STATE_IDLE;
						//_BTLE_LOG("[BTLE] Advertising timeout, go idle");
					} else {
						//BTLE_execute_adv(FALSE);
						//_BTLE_LOG("[BTLE] Advertising timeout, repeat advertising");
					}
				} 
			}
			break;
							
		case BLE_GATTC_EVT_TIMEOUT:
		case BLE_GATTS_EVT_TIMEOUT:
			// Disconnect on GATT Server and Client timeout events.
			
			///_BTLE_LOG("[BTLE] GATTS - BLE disconnect");
		//	if(BTLE_is_connected())
			//	BTLE_disconnect(BTLE_DISCONN_REASON_GATT_TIMEOUT);
			break;

		default:
			break;
	}
}

#ifdef __cplusplus
}
#endif
