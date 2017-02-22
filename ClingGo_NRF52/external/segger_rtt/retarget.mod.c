/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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
 
#include <stdio.h>
#include <stdint.h>
#include "nordic_common.h"
#include "nrf_error.h"
#include "SEGGER_RTT.h"

#if !defined(__ICCARM__)
struct __FILE 
{
    int handle;
};
#endif

FILE __stdout;
FILE __stdin;

#define RTRGT_JLINK_READ_CH_ID    0
#define RTRGT_JLINK_WRITE_CH_ID   0
#define RTRGT_NON_BLOCK 1
/**@brief Function for initializing RTT terminal.
 */
static void rtt_init(void)
{
	static uint8_t is_rtt_init = 0;
	if(is_rtt_init == 0){
			SEGGER_RTT_Init();
			SEGGER_RTT_SetTerminal(0);
		  is_rtt_init = 1;
	}
}



#if defined(__CC_ARM) ||  defined(__ICCARM__)
int fgetc(FILE * p_file)
{
    uint8_t input;
		rtt_init();
    while (SEGGER_RTT_Read (RTRGT_JLINK_READ_CH_ID, (char *)&input, 1) == 0)
    {
        // No implementation needed.
    }
    return input;
}


int fputc(int ch, FILE * p_file)
{
    UNUSED_PARAMETER(p_file);
		rtt_init();
    // UNUSED_VARIABLE(app_uart_put((uint8_t)ch));
#if RTRGT_NON_BLOCK
    SEGGER_RTT_Write(RTRGT_JLINK_WRITE_CH_ID, (char *)&ch, 1);
#else
    while (SEGGER_RTT_Write(RTRGT_JLINK_WRITE_CH_ID, (char *)&ch, 1) == 0) ;
#endif
    return ch;
}
#elif defined(__GNUC__)


int _write(int file, const char * p_char, int len)
{
    int i;
    int bytesSent = 0;
		rtt_init();
    UNUSED_PARAMETER(file);

#if RTRGT_NON_BLOCK
    bytesSent = SEGGER_RTT_Write(RTRGT_JLINK_WRITE_CH_ID, p_char, len);
#else
    do
    {
        bytesSent += SEGGER_RTT_Write(RTRGT_JLINK_WRITE_CH_ID, &(p_char [bytesSent]), len - bytesSent);
    } while (bytesSent < len);
#endif
    return bytesSent;
}


int _read(int file, char * p_char, int len)
{		rtt_init();
    UNUSED_PARAMETER(file);
    return SEGGER_RTT_Read(RTRGT_JLINK_READ_CH_ID, p_char, len);
}
#endif

