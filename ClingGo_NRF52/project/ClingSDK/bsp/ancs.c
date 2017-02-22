///* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
// *
// * The information contained herein is property of Nordic Semiconductor ASA.
// * Terms and conditions of usage are described in detail in NORDIC
// * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
// *
// * Licensees are granted free, non-transferable use of the information. NO
// * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
// * the file.
// */
//
///* Disclaimer: This client implementation of the Apple Notification Center Service can and will be changed at any time by Nordic Semiconductor ASA.
// * Server implementations such as the ones found in iOS can be changed at any time by Apple and may cause this client implementation to stop working.
// */
//
//#include "main.h"
//#include "ble_ancs_c.h"
//#include "ble_db_discovery.h"
//#include "pstorage.h"
//#include <string.h>
//#include <stdbool.h>
//#include "ble_err.h"
//#include "ble_srv_common.h"
//#include "nordic_common.h"
//#include "nrf_assert.h"
//#include "device_manager.h"
//#include "app_error.h"
//
//
//#ifdef _ENABLE_ANCS_
//
//static ancs_tx_message_t      m_tx_buffer[3];       /**< Transmit buffer for messages to be transmitted to the Notification Provider. */
//static I8U                    m_tx_insert_index ;   /**< Current index in the transmit buffer where the next message should be inserted. */
//static I8U                    m_tx_index;           /**< Current index in the transmit buffer from where the next message to be transmitted resides. */
//
//static ble_ancs_c_service_t   m_service;            /**< Current service data. */
//static ble_ancs_notif_t       ancs_notif;
//static ble_ancs_notif_attr_t  ancs_notif_attr;
//static I32U  ancs_timer, ancs_t_curr, ancs_t_diff;
//
///**@brief 128-bit service UUID for the Apple Notification Center Service.*/
//const ble_uuid128_t ble_ancs_base_uuid128 =
//{
//  {
//	// 7905F431-B5CE-4E99-A40F-4B1E122D00D0
//	0xd0, 0x00, 0x2d, 0x12, 0x1e, 0x4b, 0x0f, 0xa4,
//	0x99, 0x4e, 0xce, 0xb5, 0x31, 0xf4, 0x05, 0x79
//  }
//};
//
///**@brief 128-bit control point UUID.
// */
//const ble_uuid128_t ble_ancs_cp_base_uuid128 =
//{
//  {
//	// 69d1d8f3-45e1-49a8-9821-9BBDFDAAD9D9
//	0xd9, 0xd9, 0xaa, 0xfd, 0xbd, 0x9b, 0x21, 0x98,
//	0xa8, 0x49, 0xe1, 0x45, 0xf3, 0xd8, 0xd1, 0x69
//  }
//};
//
///**@brief 128-bit notification source UUID.
//*/
//const ble_uuid128_t ble_ancs_ns_base_uuid128 =
//{
//  {
//	// 9FBF120D-6301-42D9-8C58-25E699A21DBD
//	0xbd, 0x1d, 0xa2, 0x99, 0xe6, 0x25, 0x58, 0x8c,
//	0xd9, 0x42, 0x01, 0x63, 0x0d, 0x12, 0xbf, 0x9f
//  }
//};
//
///**@brief 128-bit data source UUID.
//*/
//const ble_uuid128_t ble_ancs_ds_base_uuid128 =
//{
//  {
//	// 22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB
//	0xfb, 0x7b, 0x7c, 0xce, 0x6a, 0xb3, 0x44, 0xbe,
//	0xb5, 0x4b, 0xd6, 0x24, 0xe9, 0xc6, 0xea, 0x22
//  }
//};
//
///**@brief Function for handling events from the database discovery module.*/
//static void db_discover_evt_handler(ble_db_discovery_evt_t * p_evt)
//{
//  ble_ancs_evt_type evt_type;
//  ble_gatt_db_char_t * p_chars;
//  p_chars = p_evt->params.discovered_db.charateristics;
//
//  N_SPRINTF("[ANCS]: Database Discovery handler called with event 0x%x\r\n", p_evt->evt_type);
//
//  // Check if the ANCS Service was discovered.
//  if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
//	  p_evt->params.discovered_db.srv_uuid.uuid == ANCS_UUID_SERVICE &&
//	  p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_VENDOR_BEGIN)
//  {
//	  // Find the handles of the ANCS characteristic.
//	  for (I32U i = 0; i < p_evt->params.discovered_db.char_count; i++)
//	  {
//	    switch (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid)
//      {
//		   case ANCS_UUID_CHAR_CONTROL_POINT:
//			    N_SPRINTF("[ANCS]: Control Point Characteristic found...");
//			    m_service.control_point.properties   = p_chars[i].characteristic.char_props;
//			    m_service.control_point.handle_decl  = p_chars[i].characteristic.handle_decl;
//			    m_service.control_point.handle_value = p_chars[i].characteristic.handle_value;
//			    m_service.control_point.handle_cccd  = p_chars[i].cccd_handle;
//			    break;
//
//		   case ANCS_UUID_CHAR_DATA_SOURCE:
//			    N_SPRINTF("[ANCS]: Data Source Characteristic found...");
//			    m_service.data_source.properties   = p_chars[i].characteristic.char_props;
//			    m_service.data_source.handle_decl  = p_chars[i].characteristic.handle_decl;
//			    m_service.data_source.handle_value = p_chars[i].characteristic.handle_value;
//			    m_service.data_source.handle_cccd  = p_chars[i].cccd_handle;
//			    break;
//
//		   case ANCS_UUID_CHAR_NOTIFICATION_SOURCE:
//			    N_SPRINTF("[ANCS]: Notification point Characteristic found...");
//			    m_service.notif_source.properties   = p_chars[i].characteristic.char_props;
//		      m_service.notif_source.handle_decl  = p_chars[i].characteristic.handle_decl;
//		      m_service.notif_source.handle_value = p_chars[i].characteristic.handle_value;
//			    m_service.notif_source.handle_cccd  = p_chars[i].cccd_handle;
//			    break;
//
//			 default:
//			    break;
//	    }
//	  }
//		evt_type = BLE_ANCS_EVT_DISCOVER_COMPLETE;
//		ANCS_on_event_handling(evt_type);
//  }
//  else
//  {
//	  // Record the current time ,disconnecting BLE after 60 seconds.
//	  ancs_timer = CLK_get_system_time();
//		evt_type = BLE_ANCS_EVT_DISCOVER_FAILED;
//		ANCS_on_event_handling(evt_type);
//  }
//}
//
///**@brief Function for passing any pending request from the buffer to the stack.
// */
//static void _tx_buffer_process(void)
//{
//  I32U err_code;
//
//  if (m_tx_index != m_tx_insert_index) {
//    // Prevent buf overflow.
//	  if(m_tx_index >= 2)
//	    m_tx_index=0;
//
//	  err_code = sd_ble_gattc_write(cling.ble.conn_handle,&m_tx_buffer[m_tx_index].req.write_req.gattc_params);
//	  if (err_code == NRF_SUCCESS)
//	    m_tx_index++;
//  }
//}
//
///**@brief Function for parsing received notification attribute response data.*/
//static void _parse_get_notif_attrs_response( const I8U *data, I16U len)
//{
//  ANCS_CONTEXT *a = &cling.ancs;
//	static I16U  current_len, buff_idx;
//
//  for (I16U i = 0; i < len; i++)
//  {
//    switch (a->parse_state)
//    {
//      case PARSE_STAT_COMMAND_ID:
//			{
//			  if(data[i] != 0){
//				 // Parse command error ,then give up parsing.
//				 	a->parse_state = PARSE_STAT_COMMAND_ID;
//				 return;
//			  }
//			  a->parse_state = PARSE_STAT_NOTIFICATION_UID_1;
//			  break;
//			}
//		  case PARSE_STAT_NOTIFICATION_UID_1:
//			{
//			  ancs_notif_attr.uid = data[i];
//			  a->parse_state = PARSE_STAT_NOTIFICATION_UID_2;
//			  break;
//			}
//      case PARSE_STAT_NOTIFICATION_UID_2:
//			{
//			  ancs_notif_attr.uid |= (data[i] << 8);
//			  a->parse_state = PARSE_STAT_NOTIFICATION_UID_3;
//			  break;
//			}
//      case PARSE_STAT_NOTIFICATION_UID_3:
//      {
//			  ancs_notif_attr.uid |= (data[i] << 8);
//			  a->parse_state = PARSE_STAT_NOTIFICATION_UID_4;
//			  break;
//			}
//      case PARSE_STAT_NOTIFICATION_UID_4:
//			{
//				ancs_notif_attr.uid |= (data[i] << 8);
//				a->parse_state = PARSE_STAT_ATTRIBUTE_TITLE_ID;
//				break;
//			}
//		  case PARSE_STAT_ATTRIBUTE_TITLE_ID:
//			{
//        ancs_notif_attr.id = (ble_ancs_notif_attr_id_t)data[i];
//				if(ancs_notif_attr.id != BLE_ANCS_NOTIF_ATTR_ID_TITLE){
//					// Parse attr title id error,then give up parsing.
//				  	a->parse_state = PARSE_STAT_COMMAND_ID;
//				  N_SPRINTF("[ANCS] parse attr title id error :%d",ancs_notif_attr.id);
//					return;
//				}
//				a->parse_state = PARSE_STAT_ATTRIBUTE_TITLE_LEN1;
//				break;
//			}
//			case PARSE_STAT_ATTRIBUTE_TITLE_LEN1:
//			{
//				ancs_notif_attr.len=data[i];
//				a->parse_state = PARSE_STAT_ATTRIBUTE_TITLE_LEN2;
//				break;
//			}
//      case PARSE_STAT_ATTRIBUTE_TITLE_LEN2:
//			{
//        ancs_notif_attr.len |= (data[i] << 8);
//			  if(ancs_notif_attr.len > ANCS_SUPPORT_MAX_TITLE_LEN){
//				  // The length of the error ,then give up parsing.
//				  a->parse_state = PARSE_STAT_COMMAND_ID;
//				  N_SPRINTF("[ANCS] get title len overstep the boundary --- parse error.");
//				  return;
//			  }
//				// Notification title length.
//				a->buf[0] = (I8U)ancs_notif_attr.len;
//        N_SPRINTF("[ANCS] get attr title len :%d",ancs_notif_attr.len );
//        buff_idx = 2;
//        current_len = 0;
//				a->parse_state = PARSE_STAT_ATTRIBUTE_TITLE_READY;
//	      break;
//			}
//			case PARSE_STAT_ATTRIBUTE_TITLE_READY:
//			{
//				a->buf[buff_idx++] = data[i];
//        current_len++;
//        if(current_len == a->buf[0])
//        {
//          a->parse_state = PARSE_STAT_ATTRIBUTE_MESSAGE_ID;
//        }
//        break;
//			}
//		  case PARSE_STAT_ATTRIBUTE_MESSAGE_ID:
//			{
//        ancs_notif_attr.id = (ble_ancs_notif_attr_id_t)data[i];
//				if(ancs_notif_attr.id != BLE_ANCS_NOTIF_ATTR_ID_MESSAGE){
//					// parse attr message id error,then give up parsing.
//				  a->parse_state = PARSE_STAT_COMMAND_ID;
//				  N_SPRINTF("[ANCS] parse attr message id error :%d",ancs_notif_attr.id);
//					return;
//				}
//				a->parse_state = PARSE_STAT_ATTRIBUTE_MESSAGE_LEN1;
//				break;
//			}
//			case PARSE_STAT_ATTRIBUTE_MESSAGE_LEN1:
//			{
//				ancs_notif_attr.len=data[i];
//				a->parse_state = PARSE_STAT_ATTRIBUTE_MESSAGE_LEN2;
//				break;
//			}
//      case PARSE_STAT_ATTRIBUTE_MESSAGE_LEN2:
//			{
//        ancs_notif_attr.len |= (data[i] << 8);
//			  if(ancs_notif_attr.len > ANCS_SUPPORT_MAX_MESSAGE_LEN){
//				  // The length of the error ,then give up parsing.
//				  a->parse_state = PARSE_STAT_COMMAND_ID;
//				  N_SPRINTF("[ANCS] get attr message len overstep the boundary and parse err");
//				  return;
//			  }
//				if(ancs_notif_attr.len >= 190)
//			    ancs_notif_attr.len=190;
//				// Notification message length.
//        a->buf[1] = (I8U)ancs_notif_attr.len;
//        N_SPRINTF("[ANCS] get attr title len :%d",ancs_notif_attr.len );
//        current_len = 0;
//				a->parse_state = PARSE_STAT_ATTRIBUTE_MESSAGE_READY;
//	      break;
//			}
//			case PARSE_STAT_ATTRIBUTE_MESSAGE_READY:
//			{
//				a->buf[buff_idx++] = data[i];
//        current_len++;
//        if(current_len == a->buf[1])
//        {
//          a->parse_state = PARSE_STAT_COMMAND_ID;
//					// ANCS state switching to "STATE STORE NOTIF ATTRIBUTE".
//					a->state = BLE_ANCS_STATE_STORE_NOTIF_ATTRIBUTE;
//        }
//        break;
//			}
//    }
//  }
//}
//
///**@brief Function for checking if data in an iOS notification is out of bounds.*/
//static I32U _ancs_verify_notification_format(const ble_ancs_notif_t  notif)
//{
//	if((notif.evt_id >= BLE_ANCS_NB_OF_EVT_ID)|| (notif.category_id >= BLE_ANCS_NB_OF_CATEGORY_ID))
//		return NRF_ERROR_INVALID_PARAM;
//	else
//	  return NRF_SUCCESS;
//}
//
///**@brief Function for receiving and validating notifications received from the Notification Provider.*/
//static void _ancs_parse_notif(const I8U *p_data, const I16U notif_len)
//{
//	I32U err_code;
//	ANCS_CONTEXT *a = &cling.ancs;
//	ble_ancs_evt_type evt_type;
//
//  if(a->start_record_time){
//		//Record the current time for Filtering old notifiction.
//		ancs_timer =CLK_get_system_time();
//	  ancs_t_curr = 0;
//	  ancs_t_diff = 0;
//
//	  a->filtering_flag = FALSE;
//    a->start_record_time	= FALSE;
//    return;
//	}
//
//	if(!a->filtering_flag){
//		// After 10 seconds, we start enable notification.
//		ancs_t_curr =  CLK_get_system_time();
//	  ancs_t_diff = (ancs_t_curr - ancs_timer);
//
//	  if (ancs_t_diff >= ANCS_FILTERING_OLD_MSG_DELAY_TIME)
//		  cling.ancs.filtering_flag = TRUE;
//
//    // Wait 10 seconds.
//    if(!a->filtering_flag)
//			return;
//  }
//
//	if (notif_len != BLE_ANCS_NOTIFICATION_DATA_LENGTH)
//	  // Invalid notific length
//	  return;
//
//	ancs_notif.evt_id         =  (ble_ancs_evt_id_t) p_data[BLE_ANCS_NOTIF_EVT_ID_INDEX];
//
//	ancs_notif.evt_flags      =   p_data[BLE_ANCS_NOTIF_FLAGS_INDEX];
//
//	ancs_notif.category_id    =   p_data[BLE_ANCS_NOTIF_CATEGORY_ID_INDEX];
//
//	ancs_notif.category_count =   p_data[BLE_ANCS_NOTIF_CATEGORY_CNT_INDEX];
//
//	ancs_notif.notif_uid      =   uint32_decode(&p_data[BLE_ANCS_NOTIF_NOTIF_UID]);
//
//	err_code = _ancs_verify_notification_format(ancs_notif);
//	if (err_code != NRF_SUCCESS)
//	// Invalid notific type
//	  return;
//
//	// ANCS event busy,waiting until the free.
//	if(a->state != BLE_ANCS_STATE_IDLE)
//		return;
//
//	evt_type = BLE_ANCS_EVT_NOTIF_REQ;
//	ANCS_on_event_handling(evt_type);
//}
//
///**@brief Function for receiving and validating notifications received from the Notification Provider.
// * @param[in] p_ble_evt Bluetooth stack event.
// */
//static void _on_evt_gattc_notif(const ble_evt_t * p_ble_evt)
//{
//	const ble_gattc_evt_hvx_t * p_notif = &p_ble_evt->evt.gattc_evt.params.hvx;
//
//	if (p_notif->handle == m_service.notif_source.handle_value)
//	{
//	  N_SPRINTF("[ANCS] get notif len :%d", p_notif->len);
//	  _ancs_parse_notif(p_notif->data, p_notif->len);
//	}
//	else if (p_notif->handle == m_service.data_source.handle_value)
//	{
//		N_SPRINTF("[ANCS] get data source len :%d", p_notif->len);
//		_parse_get_notif_attrs_response(p_notif->data, p_notif->len);
//	}
//	else
//	{
//		// No applicable action.
//	}
//}
//
///**@brief Function for handling write response events.*/
//static void _on_evt_write_rsp(void)
//{
//  _tx_buffer_process();
//}
//
//void ANCS_on_ble_evt(const ble_evt_t * p_ble_evt)
//{
//	switch (p_ble_evt->header.evt_id)
//	{
//		case BLE_GATTC_EVT_WRITE_RSP:
//				 _on_evt_write_rsp();
//				 break;
//
//		case BLE_GATTC_EVT_HVX:
//				 _on_evt_gattc_notif(p_ble_evt);
//				 break;
//
//		default:
//				 break;
//	}
//}
//
//static I32U _ble_ancs_init(void)
//{
//	ANCS_CONTEXT *a = &cling.ancs;
//
//	m_tx_insert_index = 0;
//	m_tx_index        = 0;
//
//	a->state = BLE_ANCS_STATE_IDLE;
//	a->parse_state = PARSE_STAT_COMMAND_ID;
//
//	memset(&m_service, 0, sizeof(ble_ancs_c_service_t));
//	memset(m_tx_buffer, 0, sizeof(m_tx_buffer));
//
//	m_service.handle = BLE_GATT_HANDLE_INVALID;
//
//	ble_uuid_t ancs_uuid;
//	ancs_uuid.uuid = ANCS_UUID_SERVICE;
//	ancs_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;
//
//	return ble_db_discovery_evt_register(&ancs_uuid, db_discover_evt_handler);
//}
//
///**@brief Function for creating a TX message for writing a CCCD.*/
//
//static I32U cccd_configure(const uint16_t conn_handle, const uint16_t handle_cccd, bool enable)
//{
//	ancs_tx_message_t * p_msg;
//	I16U  cccd_val = enable ? BLE_CCCD_NOTIFY_BIT_MASK : 0;
//
//	// Prevent buf overflow.
//	if(m_tx_insert_index >= 2)
//		m_tx_insert_index	= 0;
//
//	p_msg = &m_tx_buffer[m_tx_insert_index++];
//
//	p_msg->req.write_req.gattc_params.handle   = handle_cccd;
//	p_msg->req.write_req.gattc_params.len      = 2;
//	p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
//	p_msg->req.write_req.gattc_params.offset   = 0;
//	p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
//	p_msg->req.write_req.gattc_value[0]        = LSB(cccd_val);
//	p_msg->req.write_req.gattc_value[1]        = MSB(cccd_val);
//	p_msg->conn_handle                         = conn_handle;
//	p_msg->type                                = ANCS_WRITE_REQ;
//
//	_tx_buffer_process();
//	return NRF_SUCCESS;
//}
//
//static I32U _ancs_c_notif_source_notif_enable(void)
//{
//	N_SPRINTF("[ANCS]: Enable Notification Source notifications. writing to handle: %i",m_service.notif_source.handle_cccd);
//	return cccd_configure(cling.ble.conn_handle, m_service.notif_source.handle_cccd, true);
//}
//
//#if 0
//static I32U _ancs_c_notif_source_notif_disable(void)
//{
//	return cccd_configure(cling.ble.conn_handle, m_service.notif_source.handle_cccd, false);
//}
//#endif
//
//static I32U _ancs_c_data_source_notif_enable(void)
//{
//	N_SPRINTF("[ANCS]: Enable Data Source notifications. Writing to handle: %i",m_service.data_source.handle_cccd);
//	return cccd_configure(cling.ble.conn_handle, m_service.data_source.handle_cccd, true);
//}
//
//#if 0
//static I32U _ancs_c_data_source_notif_disable(void)
//{
//  return cccd_configure(cling.ble.conn_handle, m_service.data_source.handle_cccd, false);
//}
//#endif
//
//static I32U _ble_ancs_get_notif_attrs( const I32U p_uid)
//{
//	ancs_tx_message_t * p_msg;
//	I16U index    = 0;
//	I16U title_max_len=0;
//	I16U message_max_len=0;
//
//	// Prevent buf overflow.
//	if(m_tx_insert_index >= 2)
//		m_tx_insert_index	= 0;
//
//	p_msg = &m_tx_buffer[m_tx_insert_index++];
//
//	p_msg->req.write_req.gattc_params.handle   = m_service.control_point.handle_value;
//	p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
//	p_msg->req.write_req.gattc_params.offset   = 0;
//	p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
//
//	// Command ID.
//	p_msg->req.write_req.gattc_value[index++] = BLE_ANCS_COMMAND_ID_GET_NOTIF_ATTRIBUTES;
//
//	// Encode Notification UID.
//	index += uint32_encode(p_uid, &p_msg->req.write_req.gattc_value[index]);
//
//	// Attribute ID.
//	p_msg->req.write_req.gattc_value[index++] = BLE_ANCS_NOTIF_ATTR_ID_TITLE;
//
//	// Attribute set supported max title length.
//	title_max_len = ANCS_SUPPORT_MAX_TITLE_LEN;
//	p_msg->req.write_req.gattc_value[index++] = (I8U) (title_max_len);
//	p_msg->req.write_req.gattc_value[index++] = (I8U) (title_max_len >> 8);
//
//	//Encode Attribute ID.
//	p_msg->req.write_req.gattc_value[index++] = BLE_ANCS_NOTIF_ATTR_ID_MESSAGE;
//
//	// Attribute set supported max message length.
//	message_max_len = ANCS_SUPPORT_MAX_MESSAGE_LEN;
//	p_msg->req.write_req.gattc_value[index++] = (I8U) (message_max_len);
//	p_msg->req.write_req.gattc_value[index++] = (I8U) (message_max_len >> 8);
//
//	p_msg->req.write_req.gattc_params.len = index;
//	p_msg->conn_handle                    = cling.ble.conn_handle;
//	p_msg->type                           = ANCS_WRITE_REQ;
//
//	_tx_buffer_process();
//
//	return NRF_SUCCESS;
//}
//
//static BOOLEAN _ancs_request_attrs_pro(const ble_ancs_notif_t notif)
//{
//	ANCS_CONTEXT *a = &cling.ancs;
//
//	if((ancs_notif.evt_id == BLE_ANCS_EVENT_ID_NOTIFICATION_REMOVED)&&(ancs_notif.category_id == BLE_ANCS_CATEGORY_ID_INCOMING_CALL)) {
//	  // When incoming call removed,stop notifying.
//		NOTIFIC_stop_notifying();
//
//	  a->state = BLE_ANCS_STATE_IDLE;
//
//		return FALSE;
//	}
//
//	// We only need receive new added notifications,ignore modified and removed notifications.
//  if(ancs_notif.evt_id == BLE_ANCS_EVENT_ID_NOTIFICATION_ADDED){
//
//	  _ble_ancs_get_notif_attrs(notif.notif_uid);
//		return TRUE;
//	} else {
//
//		a->state = BLE_ANCS_STATE_IDLE;
//		return FALSE;
//	}
//}
//
///**@brief Function for setting up GATTC notifications from the Notification Provider.
// *
// * @details This function is called when a successful connection has been established.
// */
//static void _apple_notification_setup(void)
//{
//	I32U err_code;
//
//  // Delay because we cannot add a CCCD to close to starting encryption. iOS specific.
//	nrf_delay_ms(100);
//
//	err_code = _ancs_c_notif_source_notif_enable();
//	APP_ERROR_CHECK(err_code);
//
//	err_code = _ancs_c_data_source_notif_enable();
//	APP_ERROR_CHECK(err_code);
//}
//
//void ANCS_service_add(void)
//{
//	ble_uuid_t    service_uuid;
//	I8U           m_ancs_uuid_type;
//	I32U          err_code;
//
//	err_code = sd_ble_uuid_vs_add(&ble_ancs_base_uuid128, &m_ancs_uuid_type);
//	APP_ERROR_CHECK(err_code);
//
//	err_code = sd_ble_uuid_vs_add(&ble_ancs_cp_base_uuid128, &service_uuid.type);
//	APP_ERROR_CHECK(err_code);
//
//	err_code = sd_ble_uuid_vs_add(&ble_ancs_ns_base_uuid128, &service_uuid.type);
//	APP_ERROR_CHECK(err_code);
//
//	err_code = sd_ble_uuid_vs_add(&ble_ancs_ds_base_uuid128, &service_uuid.type);
//	APP_ERROR_CHECK(err_code);
//
//	err_code = _ble_ancs_init();
//	APP_ERROR_CHECK(err_code);
//}
//
//static BOOLEAN _ancs_supported_is_enable()
//{
//	I16U id_mask;
//
//  // If none of categories is supported, we do nothing
//	if (cling.ancs.supported_categories == 0)
//		return FALSE;
//
//	// If this is a "OTHER" category, we bond with 'NEWS' for the present.
//	if(ancs_notif.category_id == BLE_ANCS_CATEGORY_ID_OTHER)
//		ancs_notif.category_id  = BLE_ANCS_CATEGORY_ID_NEWS;
//
//	// Check whether corresponding category is enabled
//	id_mask = 1 << (ancs_notif.category_id-1);
//
//	if (id_mask & cling.ancs.supported_categories)
//		return TRUE;
//
//	return FALSE;
//}
//
//void ANCS_nflash_store_one_message(I8U *data)
//{
//  // Use message 4k space from nflash
//  I32U addr = SYSTEM_NOTIFICATION_SPACE_START;
//
//  N_SPRINTF("[ANCS] nflash store message: %d, %d, %d", cling.ancs.pkt.title_len, cling.ancs.pkt.message_len, cling.ancs.message_total);
//
//  addr += ((cling.ancs.message_total-1) << 8);
//
//  FLASH_Write_App(addr, data, 128);
//
//  addr += 128;
//
//  FLASH_Write_App(addr, data+128, 128);
//}
//
//static void _ancs_store_attrs_pro(void)
//{
//	I8U *p;
//	I8U len;
//
//	N_SPRINTF("[ANCS] start store attr message to nflash ... ");
//
//  // when a new notification message arrives, start notification state machine
//  // Do not notify user if not set supported
//  if(_ancs_supported_is_enable())
//    NOTIFIC_start_notifying(ancs_notif.category_id);
//
//  if(cling.ancs.message_total >= 16) {
//
//	  FLASH_erase_App(SYSTEM_NOTIFICATION_SPACE_START);
//
//		BASE_delay_msec(50); // Add latency before storing messages (Erasure latency: 50 ms)
//
//	  N_SPRINTF("[ANCS] message is full, go erase the message space");
//
//	  cling.ancs.message_total = 0;
//  }
//
//  cling.ancs.message_total++;
//
//  N_SPRINTF("[ANCS] message total is :%d ",cling.ancs.message_total);
//
//	len = cling.ancs.buf[0] +cling.ancs.buf[1];
//	p = &cling.ancs.buf[len+2];
//	len = (254 - len);
//
//	// Clear the unused data space.
//	memset(p,0,len);
//
//	ANCS_nflash_store_one_message(cling.ancs.buf);
//}
//
//void ANCS_on_event_handling(ble_ancs_evt_type evt_type)
//{
//	if (OTA_if_enabled())
//    return;
//
//	// If it's not IOS phone,do nothing.
//	if(cling.gcp.host_type != HOST_TYPE_IOS)
//		return;
//
//	switch (evt_type)
//	{
//		case BLE_ANCS_EVT_NULL:
//			// No implementation needed.
//			break;
//		case BLE_ANCS_EVT_DISCOVER_COMPLETE:
//		{
//			Y_SPRINTF("[ANCS] Apple Notification Service discovered on the server.");
//			// Enable ancs notifiction.
//			_apple_notification_setup();
//
//			// Attrs parse state init.
//			cling.ancs.parse_state = PARSE_STAT_COMMAND_ID;
//
//			 // Record the current time for Filtering old notifiction.
//      cling.ancs.start_record_time = TRUE;
//			break;
//		}
//		case BLE_ANCS_EVT_NOTIF_REQ:
//		{
//			// Start sent request to access notify.
//      if (_ancs_request_attrs_pro(ancs_notif)) {
//
//			  cling.ancs.state = BLE_ANCS_STATE_WAITING_PARSE_COMPLETE;
//
//			  cling.ancs.parse_time = CLK_get_system_time();
//
//        Y_SPRINTF("[ANCS] Start sent request to access notify.");
//			}
//
//      break;
//		}
//		case BLE_ANCS_EVT_DISCOVER_FAILED:
//    {
//			Y_SPRINTF("[ANCS] Apple Notification Service not discovered on the server...");
//
//			ancs_t_curr =  CLK_get_system_time();
//	    ancs_t_diff = (ancs_t_curr - ancs_timer);
//
//			if(ancs_t_diff >= ANCS_DISCOVERY_FAIL_DISCONNECT_DELAY_TIME){
//
//		    if (cling.gcp.host_type == HOST_TYPE_IOS) {
//
//				  if(BTLE_is_connected())
//					  BTLE_disconnect(BTLE_DISCONN_REASON_ANCS_DISC_FAIL);
//			  }
//		  }
//      break;
//		}
//		default:
//		  // No implementation needed.
//		  break;
//	}
//}
//
///**@brief handling the Apple Notification Service client.*/
//void ANCS_state_machine(void)
//{
//	ANCS_CONTEXT *a = &cling.ancs;
//
//	if (OTA_if_enabled())
//    return;
//
//	// If it's not IOS phone,do nothing.
//	if(cling.gcp.host_type != HOST_TYPE_IOS)
//		return;
//
//  switch (a->state)
//  {
//		case BLE_ANCS_STATE_IDLE:
//			break;
//		case BLE_ANCS_STATE_START_DISCOVER:
//		{
//		  if (CLK_get_system_time() > (a->dis_time + ANCS_START_SERVICE_DISCOVERY_DELAY_TIME)) {
//		    HAL_start_ancs_service_discovery();
//			  a->state = BLE_ANCS_STATE_IDLE;
//				Y_SPRINTF("[ANCS] start ancs service discovery");
//		  }
//		  break;
//		}
//    case BLE_ANCS_STATE_WAITING_PARSE_COMPLETE:
//		{
//			if (CLK_get_system_time() > (a->parse_time + ANCS_PARSE_NOTIF_ATTRIBUTE_TIMEOUT)) {
//				a->state = BLE_ANCS_STATE_IDLE;
//			}
//      break;
//		}
//		case BLE_ANCS_STATE_STORE_NOTIF_ATTRIBUTE:
//		{
//			Y_SPRINTF("[ANCS] Start store the specific content of notify.");
//			// Store notifiction data to nflash.
//		  _ancs_store_attrs_pro();
//			a->state = BLE_ANCS_STATE_IDLE;
//		  break;
//		}
//    default:
//      // No implementation needed.
//      break;
//    }
//}
//#endif
