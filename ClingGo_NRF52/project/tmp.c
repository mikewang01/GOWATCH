#include <time.h>
#include "osal.h"
#include "stdio.h"
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "nfc_t4t_lib.h"
/*********************************************************************
 * LOCAL DECLARATION
 */



#include "ble_cus.h"
static void nfc_t4t_callback(void          * p_context,
                                   nfc_t4t_event_t event,
                                   const uint8_t * p_data, 
                                   size_t          data_length,
                                   uint32_t        flags)
{
	
	    Y_SPRINTF("[NFC_EVENT]: event id = %d", event);
			  switch (event)
    {
			case NFC_T4T_EVENT_NONE: break;

    case NFC_T4T_EVENT_FIELD_ON: break;
 
   case  NFC_T4T_EVENT_FIELD_OFF: break;
 
    case NFC_T4T_EVENT_NDEF_READ: break;


    case NFC_T4T_EVENT_NDEF_UPDATED: break;

    case NFC_T4T_EVENT_DATA_TRANSMITTED: break;

    case NFC_T4T_EVENT_DATA_IND: 
			  for(int i = 0; i < data_length; i++){
						printf(" 0x%02x", p_data[i]);
				}
				nfc_t4t_response_pdu_send(p_data, data_length);
				break;
    }	
}
	
void nfc_test()
{
		Y_SPRINTF("[NFC_EVENT]: nfc init");
		nfc_t4t_setup(nfc_t4t_callback, NULL);
		nfc_t4t_emulation_start();
}