/***************************************************************************//**
 * File uart.h
 * 
 * Description: UART header
 *
 *
 ******************************************************************************/

#ifndef __UART_HEADER__
#define __UART_HEADER__

#include "standards.h"

// Select UART settings
#define UART2_BAUDRATE     UART_BAUDRATE_115K2       // Baudrate in bits/s: {9K6, 14K4, 19K2, 28K8, 38K4, 57K6, 115K2}
#define UART2_DATALENGTH   UART_CHARFORMAT_8         // Datalength in bits: {5, 6, 7, 8}
#define UART2_PARITY       UART_PARITY_NONE          // Parity: {UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD}
#define UART2_STOPBITS     UART_STOPBITS_1           // Stop bits: {UART_STOPBITS_1, UART_STOPBITS_2}
#define UART2_FLOWCONTROL  UART_FLOWCONTROL_DISABLED // Flow control: {UART_FLOWCONTROL_DISABLED, UART_FLOWCONTROL_ENABLED}

#define UART2_GPIO_PORT    GPIO_PORT_0
#define UART2_TX_PIN       GPIO_UART2_TX
#define UART2_RX_PIN       GPIO_UART2_RX

void UART2_api_init(void);
void UART2_print(char *str);
void UART2_disabled(void);

#endif // __BTLE_HEADER__
/** @} */
