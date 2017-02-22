///***************************************************************************/
///**
// * File: uart.c
// *
// * Description: UART driver mainly for debugging purpose
// *
// * Created on Jan 26, 2014
// *
// ******************************************************************************/
//
//#include "main.h"
//
//void UART2_api_init(void)
//{
//	// UART initialization
//#ifndef _CLING_PC_SIMULATION_
//	//Init pads
//	GPIO_ConfigurePin(UART2_GPIO_PORT, UART2_TX_PIN, OUTPUT, PID_UART2_TX, false);
//	GPIO_ConfigurePin(UART2_GPIO_PORT, UART2_RX_PIN, INPUT, PID_UART2_RX, false);
//
//	// Initialize UART component
//	SetBits16(CLK_PER_REG, UART2_ENABLE, 1);      // enable  clock for UART 2
//	uart2_init(UART2_BAUDRATE, UART2_DATALENGTH);
//#endif
//}
//
//void UART2_print(char *str)
//{
//#ifndef _CLING_PC_SIMULATION_
//	uart2_write((I8U *)str, strlen(str), NULL); // send next string character
//	uart2_finish_transfers();
//#endif
//}
//
//void UART2_disabled()
//{
//#ifndef _CLING_PC_SIMULATION_
//  uart2_flow_off();
//#endif
//}
