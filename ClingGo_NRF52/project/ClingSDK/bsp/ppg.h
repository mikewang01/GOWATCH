
#ifndef __PPG_H__
#define __PPG_H__

#include "standards.h"

#define _ENABLE_PPG_

#define ppg_I2C_ADDR    		(0xB4)            // 0x5A

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Internal sequencer, through a Command Protocol
//   1. COMMAND
//   2. RESPONSE
//   3. PARAM_WR
//   4. PARAM_RD

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// PARAM_QUERY                 8'b100a aaaa
// PARAM_SET                   8'b101a aaaa
// PARAM_AND                   8'b110a aaaa
// PARAM_OR                    8'b111a aaaa
// NOP                         8'b0000 0000
// RESET                       8'b0000 0001
// BUSADDR                     8'b0000 0010
// PS_FORCE                    8'b0000 0101
// ALS_FORCE                   8'b0000 0110
// PSALS_FORCE                 8'b0000 0111
// PS_PAUSE                    8'b0000 1001
// ALS_PAUSE                   8'b0000 1010
// PSALS_PAUSE                 8'b0000 1011
// PS_AUTO                     8'b0000 1101
// ALS_AUTO                    8'b0000 1110
// PSALS_PAUSE                 8'b0000 1111

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define I2C_ADDR               0x00
#define CHLIST                 0x01        // (EN_AUX, EN_ALS_IR, EN_ALS_VIS, EN_PS1)
#define PS_ENCODING            0x05        // (PS1_ALIGN)
#define ALS_ENCODING           0x06        // (ALS_IR_ALIGN, ALS_VIS_ALIGN)
#define PS1_ADCMUX             0x07        // (PS1 ADC Input Selection)
#define PS_ADC_COUNTER         0x0A        // (PS_ADC_REC)
#define PS_ADC_GAIN            0x0B        // (PS_ADC_GAIN)
#define PS_ADC_MISC            0x0C        // (PS_RANGE, PS_ADC_MODE)
#define ALS_IR_ADCMUX          0x0E        // (ALS_IR_ADCMUX)
#define AUX_ADCMUX             0x0F        // (AUX ADC Input Select)
#define ALS_VIS_ADC_COUNTER    0x10        // (VIS_ADC_REC)
#define ALS_VIS_ADC_GAIN       0x11        // (ALS_VIS_ADC_GAIN)
#define ALS_VIS_ADC_MISC       0x12        // (VIS_RANGE)
#define ALS_HYST               0x16        // (ALS Hysteresis)
#define PS_HYST                0x17        // (PS Hysteresis)
#define PS_HISTORY             0x18        // (PS History Setting)
#define ALS_HISTORY            0x19        // (ALS History Setting)
#define ADC_OFFSET             0x1A        // (ADC Offset)
#define LED_REC                0x1C        // (LED Recovery Time)
#define ALS_IR_ADC_COUNTER     0x1D        // (IR_ADC_REC)
#define ALS_IR_ADC_GAIN        0x1E        // (ALS_IR_ADC_GAIN)
#define ALS_IR_ADC_MISC        0x1F        // (IR_RANGE)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define REGS_PART_ID            (0x00)
#define REGS_REV_ID             (0x01)
#define REGS_SEQ_ID             (0x02)
#define REGS_INT_CFG            (0x03)
#define REGS_IRQ_ENABLE		      (0x04)
#define REGS_IRQ_MODE1		      (0x05)
#define REGS_IRQ_MODE2		      (0x06)
#define REGS_HW_KEY             (0x07)
#define REGS_MEAS_RATE		      (0x08)     // 0x00
#define REGS_ALS_RATE           (0x09)     // 0x00
#define REGS_PS_RATE            (0x0A)     // 0x00
#define REGS_ALS_LOW_TH0	      (0x0B)
#define REGS_ALS_LOW_TH1	      (0x0C)
#define REGS_ALS_HI_TH0		      (0x0D)
#define REGS_ALS_HI_TH1		      (0x0E)
#define REGS_PS_LED21		        (0x0F)
#define REGS_PS_LED3		        (0x10)
#define REGS_PS1_TH0		        (0x11)
#define REGS_PS1_TH1		        (0x12)
#define REGS_PS2_TH			        (0x13)
#define REGS_PS3_TH			        (0x15)
#define REGS_PARAM_WR		        (0x17)
#define REGS_COMMAND		        (0x18)     // 0x0b
#define REGS_RESPONSE		        (0x20)
#define REGS_IRQ_STATUS		      (0x21)
#define REGS_ALS_VIS_DATA0	    (0x22)
#define REGS_ALS_VIS_DATA1	    (0x23)
#define REGS_ALS_IR_DATA0	      (0x24)
#define REGS_ALS_IR_DATA1	      (0x25)
#define REGS_PS1_DATA0		      (0x26)
#define REGS_PS1_DATA1		      (0x27)
#define REGS_PS2_DATA0		      (0x28)
#define REGS_PS2_DATA1		      (0x29)
#define REGS_PS3_DATA0		      (0x2A)
#define REGS_PS3_DATA1		      (0x2B)
#define REGS_AUX_DATA0		      (0x2C)
#define REGS_AUX_DATA1		      (0x2D)
#define REGS_PARAM_RD		        (0x2E)
#define REGS_CHIP_STAT		      (0x30)

#define B_INT_STICKY			      (0x00)
#define B_INT_AUTO_CLEAR		    (0x02)
#define B_INT_OUTPUT_DISABLE    (0x00)
#define B_INT_OUTPUT_ENABLE	    (0x01)
                                
#define B_CMD_INT_ENABLE		    (0x20)
#define B_PS1_INT_ENBALE		    (0x04)
#define B_ALS_INT_DISABLE		    (0x00)
                                
#define B_PS1_IM_COMPLETE		    (0x00)
#define B_PS1_IM_CROSS_TH		    (0x10)                  // (1UL<<4)
#define B_PS1_IM_GT_TH			    (0x30)                  // (3UL<<4)
#define B_ALS_IM_NONE			      (0x00)
                                
#define B_CMD_INT_ANY			      (0x00)
#define B_CMD_INT_ERR			      (0x04)
                                
																
#define B_MEAS_RATE_FORCE       (0x00)
#define B_MEAS_RATE_3_375MS	    (0x6B)
#define B_MEAS_RATE_10MS		    (0x84)
#define B_MEAS_RATE_20MS		    (0x94)
#define B_MEAS_RATE_100MS		    (0xB9)
#define B_MEAS_RATE_496MS		    (0xDF)
#define B_MEAS_RATE_1984MS	    (0xFF)
                                
#define B_ALS_RATE_FORCE		    (0x00)
#define B_ALS_RATE_1		  	    (0x08)
#define B_ALS_RATE_10			      (0x32)
#define B_ALS_RATE_100			    (0x69)
                                
#define B_PS_RATE_FORCE			    (0x00)
#define B_PS_RATE_1			  	    (0x08)
#define B_PS_RATE_10			      (0x32)
#define B_PS_RATE_100			      (0x69)

#define B_PS_LED_NONE			      (0x00)
#define B_PS_LED_5_6MA			    (0x01)
#define B_PS_LED_11_2MA			    (0x02)
#define B_PS_LED_22_4MA			    (0x03)
#define B_PS_LED_44_8MA			    (0x04)

#define B_IRQ_CMD_FLAG			    (0x20)
#define B_IRQ_PS1_FLAG			    (0x04)
#define B_IRQ_ALS0_FLAG			    (0x01)
#define B_IRQ_ALS1_FLAG			    (0x02)

#define B_CHIP_RUNNING			    (0x04)
#define B_CHIP_SUSPEND			    (0x02)
#define B_CHIP_SLEEP			      (0x01)
//////ams 7000//////////////////////////////////////////////////////////////////////
#define CCU_GPIO                                  0x20
#define CCU_DEVICEID                              0x00
#define CCU_I2CM                                  0x2c
#define CCU_I2CS                                  0x30
#define CCU_UART                                  0x34
#define CCU_TMR                                   0x38
#define CCU_AFE                                   0x3c
#define CCU_WD_CTRL                               0x40
#define CCU_LP_CFG                                0x60

void ppg_Init_Sensor(void);
void ppg_Enable_Sensor(void);

void ppg_Send_Command            (I8U cmd);
void ppg_Send_Parameter          (I8U param);
void ppg_Send_Command_Parameter  (I8U cmd, I8U param);
void ppg_Send_Command_Query      (I8U offset, I8U param);
void ppg_Send_Command_SET        (I8U offset, I8U param);
void ppg_Send_Command_AND        (I8U offset, I8U param);
void ppg_Send_Command_OR         (I8U offset, I8U param);
void ppg_Modify_Command_I2C_Addr (I8U i2c_addr);

void ppg_Send_Command_NOP        (void);
void ppg_Send_Command_Reset      (void);
void ppg_PS_Force                (void);
void ppg_ALS_Force               (void);
void ppg_PS_ALS_Force            (void);
void ppg_PS_Pause                (void);
void ppg_ALS_Pause               (void);
void ppg_PS_ALS_Pause            (void);
void ppg_PS_Auto                 (void);
void ppg_ALS_Auto                (void);
void ppg_PS_ALS_Auto             (void);

#define PPG_MEASURING_PERIOD_FOREGROUND          1    // real time measuring
#define PPG_MEASURING_PERIOD_BACKGROUND_DAY          600    // real time measuring
#define PPG_MEASURING_PERIOD_BACKGROUND_NIGHT          1800    // real time measuring
#define PPG_SAMPLE_PROCESSING_PERIOD             1
#define PPG_HR_MEASURING_TIMEOUT                25
#define PPG_NEXT_CHECK_LATENCY                 180

#define PPG_WEARING_DETECTION_OVERALL_INTERVAL      180000   // 180 seconds
#define PPG_WEARING_DETECTION_LPS_INTERVAL          5000     //   5 seconds
#define PPG_WEARING_DETECTION_BACKIDLE_INTERVAL     30000    // 300 seconds

typedef enum {
	PPG_BODY_NOT_WEAR, 
	PPG_BODY_WEAR, 
	PPG_UNKNOWN
} PPG_WEARING_STATUS;

typedef enum {
	PPG_WEARING_IDLE,
	PPG_WEARING_MEASURING,
	PPG_WEARING_DETECTION,
	PPG_WEARING_CONFIRM
} PPG_WEARING_DETECTION_STATES;

#define PPG_WEAR_TH1    -1700000
#define PPG_WEAR_TH2    -1200000
#define PPG_WEAR_TH3    -1000000

typedef enum {
	PPG_SUCCESS,
	PPG_NO_SKIN_TOUCH,
	PPG_INIT_AFE_FAIL,
} PPG_STATUSCODE;

typedef enum {
	PPG_STAT_IDLE,
	PPG_STAT_WEARING_CONFIRMED,
	PPG_STAT_DUTY_ON,
	PPG_STAT_SAMPLE_READY,
	PPG_STAT_DUTY_OFF,
} PPG_STATES;

typedef struct tagHEARTRATE_CTX{
	I32U m_measuring_timer_in_s;
	I32U m_zero_point_timer;
	I32U m_no_skin_touch_in_s;
	I32U alert_ts;
	
	BOOLEAN sample_ready;
	BOOLEAN heart_rate_ready;

	// PPG state machine
	PPG_STATES state;

	// Heart rate statistics
	I8U current_rate;
	I8U update_cnt;
	I8U minute_rate;
	
	// Heart rate calculation variables
	I32U  m_pre_zero_point;
	I16S  m_pre_ppg_sample;
	I16U  m_pre_pulse_width;
		
	I8U   m_epoch_num[8];
	//BUTT_CTX butterworth_filter_context;
} HEARTRATE_CTX;


//
// BOOLEAN PPG_switch_to_duty_on_state(void);
//


BOOLEAN _is_user_viewing_heart_rate(void);
I16S    _get_light_strength_register(void);

void ppg_Disable_Sensor(void);
void ppg_configure_sensor(void);

void PPG_init(void);
void PPG_state_machine(void);
void PPG_wearing_detection_state_machine(void);
I8U PPG_minute_hr_calibrate(void);

#endif // __PPG_H__
/** @} */
