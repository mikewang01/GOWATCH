/***************************************************************************
 * File: interrupt.c
 * 
 * Description: interrupt handler routines
 *
 * Created on Feb 20, 2014
 *
 ******************************************************************************/

#include "main.h"
#include "nrf_drv_uart.h"
void tx_sucess_int (void);
void rx_sucess_int (uint8_t data);
void _config_iso7816_output(void);
//void UART0_IRQHandler(void)
//{
//#if 1
//	 if (NRF_UART0->EVENTS_ERROR != 0)
//    {
//			  NRF_UART0->ERRORSRC = 0xff;
//        //nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_TXDRDY);
//       // nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_RXTO);
//        nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_ERROR);  /* thanks for correction Lalit*/
//				if((NRF_UART0->INTENSET&UART_INTENCLR_TXDRDY_Msk) == UART_INTENCLR_TXDRDY_Msk){
//						/*check under which circumstance this error happened*/
//						//nrf_uart_disable(NRF_UART0);
//						_config_iso7816_output();
//					//	nrf_uart_enable(NRF_UART0);
//					  NRF_UART0->EVENTS_TXDRDY = 0;
//						tx_sucess_int ();
//				
//				}else if((NRF_UART0->INTENSET&UART_INTENCLR_RXDRDY_Msk) == UART_INTENCLR_RXDRDY_Msk){
//				}
//    }
//		if ( NRF_UART0->EVENTS_RXDRDY != 0 && ((NRF_UART0->INTENSET&UART_INTENCLR_RXDRDY_Msk) == UART_INTENCLR_RXDRDY_Msk)) {
//        NRF_UART0->EVENTS_RXDRDY = 0;
//				uint8_t t =  NRF_UART0->RXD;
//        rx_sucess_int (t);

//    }
//		if(NRF_UART0->EVENTS_TXDRDY != 0 && ((NRF_UART0->INTENSET&UART_INTENCLR_TXDRDY_Msk) == UART_INTENCLR_TXDRDY_Msk)) {
//        NRF_UART0->EVENTS_TXDRDY = 0;
//        tx_sucess_int ();

//    }
//		

//		
//#endif
//}
