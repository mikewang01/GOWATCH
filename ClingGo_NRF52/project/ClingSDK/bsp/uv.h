

#ifndef __uv_H__
#define __uv_H__

#include "standards.h"

#define uv_I2C_ADDR                 		(0xC0)    // 0x60

#define uv_REGS_PART_ID                 (0x00)
#define uv_REGS_REV_ID                  (0x01)
#define uv_REGS_SEQ_ID                  (0x02)
#define uv_REGS_INT_CFG                 (0x03)    // 0: INT pin is never driven/1: INT pin driven low whenever an IRQ_STATUS and its corresponding IRQ_ENABLE bits match
#define uv_REGS_IRQ_ENABLE              (0x04)    // ALS Interrupt Enable
#define uv_REGS_HW_KEY                  (0x07)    // System must write value 0x17 to this register for proper Si1132 operation.
#define uv_REGS_MEAS_RATE0              (0x08)    // MEAS_RATE1 and MEAS_RATE0 together form a 16-bit value: MEAS_RATE[15:0]
#define uv_REGS_MEAS_RATE1              (0x09)
#define uv_REGS_MEAS_UCOEF0             (0x13)
#define uv_REGS_MEAS_UCOEF1             (0x14)
#define uv_REGS_MEAS_UCOEF2             (0x15)
#define uv_REGS_MEAS_UCOEF3             (0x16)
#define uv_REGS_PARAM_WR                (0x17)    // Mailbox register for passing parameters from the host to the sequencer.
#define uv_REGS_COMMAND                 (0x18)    // Mailbox register for command.
#define uv_REGS_RESPONSE                (0x20)
#define uv_REGS_IRQ_STATUS              (0x21)    // Command Interrupt Status/ALS Interrupt Status.
#define uv_REGS_ALS_VIS_DATA0           (0x22)
#define uv_REGS_ALS_VIS_DATA1           (0x23)
#define uv_REGS_ALS_IR_DATA0            (0x24)
#define uv_REGS_ALS_IR_DATA1            (0x25)
#define uv_REGS_AUX_DATA0_UVIDX0        (0x2C)
#define uv_REGS_AUX_DATA1_UVIDX1        (0x2D)
#define uv_REGS_PARAM_RD                (0x2E)
#define uv_REGS_CHIP_STAT               (0x30)
#define uv_REGS_ANA_IN_KEY3             (0x3B)    // ANA_IN_KEY[31:24]
#define uv_REGS_ANA_IN_KEY2             (0x3C)    // ANA_IN_KEY[23:16]
#define uv_REGS_ANA_IN_KEY1             (0x3D)    // ANA_IN_KEY[15: 8]
#define uv_REGS_ANA_IN_KEY0             (0x3E)    // ANA_IN_KEY[ 7: 0]

#define uv_PARAM_I2C_ADDR               (0x00)
#define uv_PARAM_CHLIST                 (0x01)
#define uv_PARAM_ALS_ENCODING           (0x06)
#define uv_PARAM_ALS_IR_ADCMUX          (0x0E)
#define uv_PARAM_AUX_ADCMUX             (0x0F)
#define uv_PARAM_ALS_VIS_ADC_COUNTER    (0x10)
#define uv_PARAM_ALS_VIS_ADC_GAIN       (0x11)
#define uv_PARAM_ALS_VIS_ADC_MISC       (0x12)
#define uv_PARAM_ALS_IR_ADC_COUNTER     (0x1D)
#define uv_PARAM_ALS_IR_ADC_GAIN        (0x1E)
#define uv_PARAM_ALS_IR_ADC_MISC        (0x1F)

#define uv_RESPONSE_INV_SETTING         (0x80)
#define uv_RESPONSE_VIS_OVF             (0x8C)
#define uv_RESPONSE_IR_OVF              (0x8D)
#define uv_RESPONSE_AUX_OVF             (0x8E)

#define MUX_SMALL_IR                        (0x00)
#define RECCNT_511                          (0x70)

#define FLT_TO_FX20(x)                      ((I32S)((x*1048576)+.5))
#define FX20_ONE                            (FLT_TO_FX20( 1.000000))
#define FX20_BAD_VALUE                      (0xFFFFFFFF)


typedef enum {
	UV_STAT_IDLE,
	UV_STAT_DUTY_ON,
	UV_STAT_SAMPLE_READY,
	UV_STAT_DUTY_OFF,
} UV_STATES;

typedef struct tagUV_CTX {
	I32U measure_timebase;
	I8U  uv_index;
	UV_STATES state;
	BOOLEAN b_perform_measure;
	I8U max_uv;
	I8U max_UI_uv;
} UV_CTX;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void uv_Send_Command(I8U cmd);
void uv_Send_Parameter(I8U param);
void uv_Send_Command_Parameter(I8U cmd, I8U param);
void uv_Send_Command_Query(I8U offset, I8U param);
void uv_Send_Command_SET  (I8U offset, I8U param);     /* 
void uv_Send_Command_AND  (I8U offset, I8U param);
void uv_Send_Command_OR   (I8U offset, I8U param);     */
void uv_Send_Command_NOP(void);
void uv_Send_Command_Reset(void);
void uv_ALS_Force(void);                               // Forces a single ALS measurement
void uv_ALS_Pause(void);                               // Pauses autonomous ALS
void uv_ALS_Auto(void);                                /* 
void uv_PS_Force(void);
void uv_PS_ALS_Force(void);
void uv_PS_Pause(void);
void uv_PS_ALS_Pause(void);
void uv_PS_Auto(void);
void uv_PS_ALS_Auto(void);                             */
void uv_Get_Cal(void);                                 // Reports calibration data to I2C registers 0x22 to 0x2D
void uv_Modify_Command_I2C_Addr(I8U i2c_addr);
void uv_Reset(void);
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//
// UV index
void uv_Configure_Sensor(void);
void uv_Disable_Sensor(void);
BOOLEAN uv_Measure(void);

I8U UV_get_max_index_per_minute(void);

BOOLEAN _is_user_viewing_uv_index(void);

void UV_Init(void);
void UV_state_machine(void);


#endif            // #ifndef uv_H


