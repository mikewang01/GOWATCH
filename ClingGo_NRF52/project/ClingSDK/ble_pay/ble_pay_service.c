/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

/* Attention!
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile
*  qualification listings, this section of source code must not be modified.
*/

#include "ble_pay_service.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "main.h"
//#include <absacc.h>
#define INVALID_BATTERY_LEVEL 255

typedef enum {
    READ_REQ,      /**< Type identifying that this tx_message is a read request. */
    WRITE_REQ      /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

/**@brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct {
    uint8_t                  gattc_value[20];  /**< The message to write. */
    ble_gattc_write_params_t gattc_params;                       /**< The GATTC parameters for this message. */
} write_params_t;

/**@brief Structure for holding the data that will be transmitted to the connected central.
 */
typedef struct {
    uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
    tx_request_t type;         /**< Type of message. (read or write). */
    union {
        uint16_t       read_handle;  /**< Read request handle. */
        write_params_t write_req;    /**< Write request message. */
    } req;
} tx_message_t;

#define TX_BUFFER_MASK 0x0a
//static ble_pay_c_t * mp_ble_bas_c;                 /**< Pointer to the current instance of the BAS Client module. The memory for this is provided by the application.*/
static tx_message_t  m_tx_buffer[TX_BUFFER_MASK + 1];  /**< Transmit buffer for the messages that will be transmitted to the central. */
static uint32_t      m_tx_insert_index = 0;        /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t      m_tx_index        = 0;        /**< Current index in the transmit buffer containing the next message to be transmitted. */


/**@brief Function for passing any pending request from the buffer to the stack.
 */
#define UICO_UUID_BASE {0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x9A, 0x94,\
                        0xE3, 0x11, 0x7B, 0x66, 0x00, 0x55, 0x4E, 0x6A };

#define PAY_UUID_SERVICE       0xFFE1
												
#define CLING_DEBUG_RX_UUID        0xFFE2
#define CLING_DEBUG_TX_UUID        0xFFE1
												
#define FIRST_HANDLE    0x0001
#define LAST_HANDLE     0xFFFF

static void tx_buffer_process(ble_pay_t * p_bas);
static ble_pay_t * p_bas_local = NULL;
static void tx_send(ble_pay_t * p_ble_nus_c, uint16_t handle, uint8_t *str , uint16_t len);
int transfer_data_to_uico_touch(uint8_t *buffer_p, uint16_t tx_lenth, uint16_t *rx_lenth_p);
//static void (*rx_stream_func)(uint8_t) = NULL;
static bool seam_buffer = false;

static uint8_t tx_handle_num = 0xff;
//static uint8_t rx_handle_num = 0xff;
												
#define IS_BUIFFER_LOCKED() (seam_buffer == true)
#define LOCK_BUFFER() (seam_buffer = true)
#define UNLOCK_BUFFER() (seam_buffer = false)


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_pay_t * p_bas, ble_evt_t * p_ble_evt)
{
    p_bas->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static uint8_t tx_notification_flag = false;
static void on_disconnect(ble_pay_t * p_bas, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_bas->conn_handle = BLE_CONN_HANDLE_INVALID;
    tx_notification_flag = false;
    /*notify up layer the mail of tets tool disconnected*/
    ble_pay_evt_t evt;
    evt.evt_type = ble_pay_EVT_TEST_TOOL_DISCONNECTED ;
    if(p_bas->evt_handler != NULL) {
        p_bas->evt_handler(p_bas, &evt);
    }
}


/**@brief Function for notidfying pc the data over ble
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */

int ble_pay_notify_data_to_pc(ble_pay_t * p_bas, ble_gatts_char_handles_t handle, uint8_t *buffer, uint16_t lenth)
{
    tx_send(p_bas, handle.value_handle, buffer , lenth);
		return 1;
}
/**@brief Function for notidfying pc the data over ble
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static int (*ble_recievd_callback)(uint8_t *, uint16_t) __attribute__((unused)) = NULL;
int ble_pay_recieve_data_from_pc_register(int (*p)(uint8_t *, uint16_t))
{
    if(p != NULL) {
        ble_recievd_callback = p;
    }

    return true;
}
/**@brief Function for handling the Write event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */

static void on_write(ble_pay_t * p_bas, ble_evt_t * p_ble_evt)
{
    if(p_bas->is_notification_supported) {
        ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
        int i = 0;

        for(i = 0; i < 2 ; i++) {
            int k = 0;
            if((p_evt_write->handle == p_bas->battery_level_handles[i].cccd_handle)) {
                N_SPRINTF("[BLE PAY] cccd_handle %d DATA RECIEVED lenth = %d handle =%d", i, p_evt_write->len, p_evt_write->handle);

                for(k = 0; k < p_evt_write->len ; k++) {
                    N_SPRINTF("0x%02x", p_evt_write->data[k]);
                }
								if( p_evt_write->data[0] > 0){
										tx_notification_flag = true;
								}else{
										tx_notification_flag = false;	
								}
                /*notice uuper layer this device has been connected with pc tool*/
                ble_pay_evt_t evt;
                if(i == tx_handle_num) {
                    tx_notification_flag = true;
                }
                if(ble_srv_is_notification_enabled(p_evt_write->data)) {
                    evt.evt_type = ble_pay_EVT_NOTIFICATION_ENABLED;
                } else {
                    evt.evt_type = ble_pay_EVT_NOTIFICATION_DISABLED;
                }
                if(p_bas->evt_handler != NULL) {
                    p_bas->evt_handler(p_bas, &evt);
                }

            }
            if((p_evt_write->handle == p_bas->battery_level_handles[i].value_handle)) {
                N_SPRINTF("[BLE PAY] value_handle %d DATA RECIEVED lenth = %d", i, p_evt_write->len);
								int receive_one_char_callback(unsigned char rev_char);
								for(int i=0; i < p_evt_write->len; i++){
									/*if while package has been recieved then there is no need to prrocess more*/
										if(receive_one_char_callback(p_evt_write->data[i]) == true){
												break;
										}
								}
            }
        }

        if(

            (p_evt_write->handle == p_bas->battery_level_handles[0].cccd_handle)
            &&
            (p_evt_write->len == 2)
        ) {
            // CCCD written, call application event handler
            if(p_bas->evt_handler != NULL) {
                ble_pay_evt_t evt;

                if(ble_srv_is_notification_enabled(p_evt_write->data)) {
                    evt.evt_type = ble_pay_EVT_NOTIFICATION_ENABLED;
                } else {
                    evt.evt_type = ble_pay_EVT_NOTIFICATION_DISABLED;
                }

                p_bas->evt_handler(p_bas, &evt);
            }
        }
    }
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */

static void _on_ble_evt(ble_evt_t * p_ble_evt)
{
    ble_pay_t * p_bas = p_bas_local;

    switch(p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_bas, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_bas, p_ble_evt);
            break;

        case  BLE_EVT_TX_COMPLETE:
            //N_SPRINTF("[%s]:BLE_EVT_TX_COMPLETEP\r\n", __FUNCTION__);
						if(tx_notification_flag == true){
								tx_buffer_process(p_bas);
						}
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_bas, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            N_SPRINTF("[%s]:BLE_GATTC_EVT_WRITE_RSP\r\n", __FUNCTION__);
            tx_buffer_process(p_bas);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            N_SPRINTF("[%s]:BLE_GATTC_EVT_READ_RSP\r\n", __FUNCTION__);
            break;

        default:
            // No implementation needed.
            break;
    }

}


/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_bas        Battery Service structure.
 * @param[in]   p_bas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t cling_pay_char_add(ble_pay_t * p_bas, const ble_pay_init_t * p_bas_init)
{
    uint32_t            err_code = NRF_SUCCESS;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_battery_level[20] __attribute__((unused));
//    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
//    uint8_t             init_len;
    int i = 0;
#if 1
    p_bas->is_notification_supported =   p_bas_init->support_notification;

    // Add Battery Level characteristic
    if(p_bas->is_notification_supported) {
        memset(&cccd_md, 0, sizeof(cccd_md));

        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
        // authentication.
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        cccd_md.write_perm = p_bas_init->battery_level_char_attr_md.cccd_write_perm;
        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    }

#endif
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write =  1;
    char_md.char_props.read   = 0;
    char_md.char_props.notify = (p_bas->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = (p_bas->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md         = NULL;

    //BLE_UUID_BLE_ASSIGN(ble_uuid, UICO_LEN_NOTIFY_UUID);
    ble_uuid.uuid = 0;
    ble_uuid.type = p_bas->uuid_type;//p_bas->uuid_type;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_bas_init->battery_level_char_attr_md.read_perm;
    attr_md.write_perm = p_bas_init->battery_level_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    initial_battery_level[0] = p_bas_init->initial_batt_level[0];

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20;
    attr_char_value.p_value   = NULL;//initial_battery_level;



    //BLE_UUID_BLE_ASSIGN(ble_uuid, UICO_LEN_NOTIFY_UUID);
    ble_uuid.uuid = CLING_DEBUG_TX_UUID;
    ble_uuid.type = p_bas->uuid_type;//p_bas->uuid_type;
		tx_handle_num = i; 
    err_code = sd_ble_gatts_characteristic_add(p_bas->service_handle, &char_md,
               &attr_char_value,
               &p_bas->battery_level_handles[i++]);

  
    //BLE_UUID_BLE_ASSIGN(ble_uuid, UICO_LEN_NOTIFY_UUID);
    ble_uuid.uuid = CLING_DEBUG_RX_UUID;
    ble_uuid.type = p_bas->uuid_type;//p_bas->uuid_type; 
    err_code = sd_ble_gatts_characteristic_add(p_bas->service_handle, &char_md,
               &attr_char_value,
               &p_bas->battery_level_handles[i++]);
							 
    p_bas->report_ref_handle = BLE_GATT_HANDLE_INVALID;


    return err_code;
}


void ble_pay_transmit_complete_handler(void * p_context)
{

    ble_pay_t * p_bas = (ble_pay_t *)p_context;
#if 0
    /*notify up layer the message that data has been sended successfully*/
    ble_pay_evt_t evt;
    evt.evt_type = ble_pay_EVT_BUFFER_SENDED_SUCESSFULLY;
    if(p_bas->evt_handler != NULL) {
        p_bas->evt_handler(p_bas, &evt);
    }
#else
		 tx_buffer_process(p_bas);
#endif

}
/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_bas        Battery Service structure.
 * @param[in]   p_bas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_pay_init(ble_pay_t * p_bas, const ble_pay_init_t * p_bas_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    p_bas->uuid_type = BLE_UUID_TYPE_BLE;

    p_bas_local = p_bas;
    // Initialize service structure
    p_bas->evt_handler               = p_bas_init->evt_handler;
    p_bas->conn_handle               = BLE_CONN_HANDLE_INVALID;
    p_bas->is_notification_supported = p_bas_init->support_notification;
    //p_bas->battery_level_las+++++++++++++t[0]        = NULL;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, PAY_UUID_SERVICE);
    /*16bytes */
    ble_uuid.type = p_bas->uuid_type;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_bas->service_handle);

    if(err_code != NRF_SUCCESS) {
        return err_code;
    }
		/*add ble event dispacher to ble event manager*/
		ble_on_event_process_reg(_on_ble_evt);
    // Add battery level characteristic
    return cling_pay_char_add(p_bas, p_bas_init);
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

static void tx_buffer_process(ble_pay_t * p_bas)
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
                N_SPRINTF("[ble_pay]: data lenth = %d..\r\n", *(hvx_params.p_len));
                err_code = sd_ble_gatts_hvx(m_tx_buffer[m_tx_index].conn_handle, &hvx_params);
                //err_code = sd_ble_gatts_hvx(m_tx_buffer[m_tx_index].conn_handle, &hvx_params);
            }
        }

        if(err_code == NRF_SUCCESS) {
            N_SPRINTF("[ble_pay]: SD Read/Write API returns Success..\r\n");
            m_tx_index++;
				  	N_SPRINTF("[ble_pay]: tx sucess m_tx_index = %d", m_tx_index);
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
#if 1
				N_SPRINTF("[ble_pay]: tx buffer empty");
        /*notify up layer the message that data has been sended successfully*/
        ble_pay_evt_t evt;
        evt.evt_type = ble_pay_EVT_BUFFER_SENDED_SUCESSFULLY;
        if(p_bas->evt_handler != NULL) {
             p_bas->evt_handler(p_bas, &evt);
        }
#else
        app_timer_start(ble_pay_timer_id, SYSCLK_INTERVAL_10MS, p_bas);
#endif
    }
}


void ble_debug_tool_send_test()
{
    if(tx_notification_flag == true) {
        ble_pay_t * p_bas = p_bas_local;
        uint8_t buffer[] = {0x7d, 0x00, 0x7e};
        // tx_send(p_bas, p_bas->battery_level_handles[0].value_handle, &lenth , 1);
        tx_send(p_bas, p_bas->battery_level_handles[1].value_handle , buffer , sizeof(buffer));
        N_SPRINTF("[ble_pay]:ble_debug_tool_send_test");
    }

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
static void tx_send(ble_pay_t * p_ble_nus_c, uint16_t handle_num, uint8_t *str , uint16_t len)
{
    tx_message_t * p_msg;

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
    p_msg->conn_handle = p_ble_nus_c->conn_handle;
    //???????
    p_msg->type = WRITE_REQ;

    memcpy(p_msg->req.write_req.gattc_value, str, len);
    UNLOCK_BUFFER();
    //connected_pc_tool
    //????
		N_SPRINTF("[ble uico]: m_tx_insert_index = %d", m_tx_insert_index);
    tx_buffer_process(p_ble_nus_c);
}

#if 1
/*********************************************************************
 * @fn      tx_buffer_frame
 *
 * @brief   add meessage to coresponed buffer according to char handle
 *
 * @param   task_id,message_type.
 *
 * @return  LX_OK ,LX_OK
 */
int tx_buffer_frame(ble_pay_t * p_bas, ble_gatts_char_handles_t handle, uint8_t *buffer, uint16_t lenth)
{

    uint8_t *s;
		uint8_t a1[20];
		uint8_t a2[20];
    ble_gatts_value_t gatts_value;
    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));
    gatts_value.len     = lenth;
    gatts_value.offset  = 0;
    gatts_value.p_value = buffer;
    // Update database.
    uint16_t err_code = sd_ble_gatts_value_set(p_bas->conn_handle,
                        handle.value_handle,
                        &gatts_value);

    if(err_code != NRF_SUCCESS) {
        return false;
    }

    if(lenth == 20) {
        memcpy(a1, buffer, 20);
        s = a1;
    } else {
        memcpy(a2, buffer, lenth);
        s = a2;
    }

    ble_gatts_hvx_params_t hvx_params;
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = m_tx_buffer[m_tx_index].req.write_req.gattc_params.handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;//20-lenth-1;//gatts_value.offset;
    hvx_params.p_len  = &(gatts_value.len);
    hvx_params.p_data = s;//buffer;
    N_SPRINTF("[ble_pay]: data lenth = %d..\r\n", *(hvx_params.p_len));
    err_code = sd_ble_gatts_hvx(m_tx_buffer[m_tx_index].conn_handle, &hvx_params);

    if(err_code == NRF_SUCCESS) {
        return true;
    } else {
        return false;
    }



}

void ble_pay_add_to_buffer(ble_pay_t * p_bas, uint16_t handle, uint8_t *str , uint16_t len)
{
    tx_message_t * p_msg;

    if(len > 20) {
        return ;
    }

    /*if buffer is locked*/
    if(IS_BUIFFER_LOCKED()) {
        return;
    }

    LOCK_BUFFER();
    //????
    p_msg = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= 0x07;
    //????
    p_msg->req.write_req.gattc_params.handle = handle;

    N_SPRINTF("[ble uico]: sended handle= %d lenth = %d", p_msg->req.write_req.gattc_params.handle, len);
    p_msg->req.write_req.gattc_params.len = len;
    //??
    p_msg->req.write_req.gattc_params.p_value = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset = 0;
    //???
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    //????
    p_msg->conn_handle = p_bas->conn_handle;
    //???????
    p_msg->type = WRITE_REQ;

    memcpy(p_msg->req.write_req.gattc_value, str, len);
    UNLOCK_BUFFER();
}
void ble_pay_send_buffer(ble_pay_t * p_bas)
{
    tx_buffer_process(p_bas);
}

int ble_pay_write_interface(uint8_t *p, size_t lenth)
{
		ble_pay_t * p_bas = p_bas_local;
		if((tx_handle_num != 0xff) && (p_bas != NULL) && (tx_notification_flag == true)){
			uint16_t i = lenth/20;
			if(lenth%20){
				i++;
			}
			N_SPRINTF("[ble uico]: ble_pay_write_interfacelenth = %d tx_handle_num = %d", i,  p_bas->battery_level_handles[tx_handle_num].value_handle);
			/*make sure each data buffer size is less than 20bytes*/
			for(; i>0; i--){
				N_SPRINTF("[ble uico]: hello");
				ble_pay_notify_data_to_pc(p_bas, p_bas->battery_level_handles[tx_handle_num] ,p , 20);
				p+=20;
			}
		}
		return 0;
}
#endif

