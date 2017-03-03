#ifndef __fm151_H__
#define __fm151_H__

#include "standards.h"
#include "stdint.h"
#define ISO7816_ENABLE

#define _EIGHT_MHZ_                        1    // 8MHz
#define _FOUR_MHZ_                         2    // 4MHz
#define _TWO_PLUS_MHZ_                     3    // 2.6667MHz

#define _TIMER_START                       1
#define _TIMER_STOP                        0

#define _BAUDRATE_1200            0x0004F000
#define _BAUDRATE_2400            0x0009D000
#define _BAUDRATE_4800            0x0013B000
#define _BAUDRATE_9600            0x00275000
#define _BAUDRATE_14400           0x003B0000
#define _BAUDRATE_19200           0x004EA000
#define _BAUDRATE_28800           0x0075F000
#define _BAUDRATE_38400           0x009D5000
#define _BAUDRATE_57600           0x00EBF000
#define _BAUDRATE_76800           0x013A9000
#define _BAUDRATE_115200          0x01D7E000
#define _BAUDRATE_230400          0x03AFB000
#define _BAUDRATE_250000          0x04000000
#define _BAUDRATE_460800          0x075F7000
#define _BAUDRATE_921600          0X0EBEDFA4
#define _BAUDRATE_1000000         0x10000000

/*
 * our smart card clock frequency is 4 MHz, so the
 * baud rate should be 4,000,000/372 = 10753.
 * To set baud rate as 10753, the register should be
 * set as 0x002C3E2A
 */
#define _BAUDRATE_10753          0x002C3E2A

#define ISO7816_REGION0     0x00
#define ISO7816_REGION1     0x01
#define ISO7816_EEPROM      0x02
#define ISO7816_COMMAND     0x03

// Region 0 Register
#define IO_CFG1                             0x00     // IO Configuration 1 
#define IO_CFG2                             0x01     // IO Configuration 2 
#define OPERATE_CTRL                        0x02     // Operation Control Register 
#define WAKEUP_CTRL                         0x03     // Wake-up Control      
#define AGC_CTRL                            0x04     // Gain and AGC Control
#define RCV_DEM_AND_PHASE_CTRL              0x05     // Demodulator and Phase Adjustment Control
#define AUX_CTRL1                           0x06     // Auxiliary Control 1
#define ASEB_DRV_CTRL                       0x07     // ACLB Non-modulated Level Control 
#define ASEB_RT_DPD_CTRL                    0x08     // ACLB R2T/T2R and Digital Propagation Delay
#define VP_RF_TX_CTRL                       0x09     // VP_RF and Tx Type
#define AUX_CTRL2                           0x0A     // Auxiliary Control 2
#define QFA_ATT_CTRL                        0x0B     // Q Factor and AAT Control
#define VDD_SE_VP_RF                        0x0C     // VDD_SE and VP_RF during QFA and AAT Control
#define APC_CTRL                            0x0D     // APC Control 
#define MASK_INT                            0x0E     // Mask Interrupt Register

// Region 1 Register
#define MISC_CTRL                           0x19
#define MODE1                               0x1A

typedef enum {
    ISO7816_OP_IDLE,
    ISO7816_OP_ACTIVATE,
    ISO7816_OP_DEACTIVATE,
    ISO7816_OP_RETRIEVE_ATR,
    ISO7816_OP_SEND_APDU,
    ISO7816_OP_RETRIEVE_BALANCE,
} ISO7816_OPERATION;

typedef struct tagISO7816_CTX {
//ISO7816_OPERATION stat;
    BOOLEAN bISO7816_spi_mode;
    BOOLEAN bISO7816_activated;
} ISO7816_CTX;

extern void ISO7816_Init(void);
extern void ISO7816_command_process(ISO7816_OPERATION op_code, I8U *pBuf, I16U *len);
extern int fm151_reg_response_callback(int (*p)(void*data, uint16_t length) );
extern int active_fm151(void);
extern int deactive_fm151(void);

struct fm151_adapter_call_backs {
    int (*init)(void);
    int (*core_fsm)(void *p_dr);
    int (*rx_cb)(void *p_dr, uint16_t length);
};


#endif
