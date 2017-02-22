
#include "main.h"
#include "i2cdev_hal.h"
#include "clingtask.h"
#include "ppg_comm.h"
static EN_STATUSCODE ppg_read_reg(I8U *send_buf, I8U txbytes, I8U *pRegVal ,I8U rxbytes)
{
#ifndef _CLING_PC_SIMULATION_
	  CLASS(I2CDevHal) *p_instance = I2CDevHal_get_instance();
	  i2c_dev_handle_t t = p_instance->ppg_open(p_instance);
		//int err_code = p_instance->write(p_instance, t, send_buf, txbytes);
		//err_code = p_instance->read(p_instance, t, pRegVal, rxbytes);
	 int err_code = p_instance->write_read(p_instance, t, send_buf, txbytes, pRegVal, rxbytes);
		p_instance->close(p_instance);

#endif
}

static BOOLEAN ppg_write_reg(I8U *send_buf, I8U txbytes, I8U *pRegVal , I8U databytes)
{
#ifndef _CLING_PC_SIMULATION_
    I8U BUFF[7]; 
		CLASS(I2CDevHal) *p_instance = I2CDevHal_get_instance();
	  i2c_dev_handle_t t = p_instance->ppg_open(p_instance);
	  memcpy(BUFF, send_buf ,txbytes);
	  memcpy(BUFF+txbytes , pRegVal ,databytes);
	  txbytes= txbytes + databytes;	
		int err_code = p_instance->write(p_instance, t, BUFF, txbytes);
		p_instance->close(p_instance);

#endif
}

//static void _calc_heart_rate()
//{
//	HEARTRATE_CTX *h = &cling.hr;

//	I8U  epoch_num;
//	I16S pulse_width;
//	I8S  heart_rate;
//	I32S one_minute;
//	I32S sum;
//	I8S  non_num;
//	I8S  avg_epoch_num;
//	I8U  i;

//	// current pulse width
//	epoch_num = 0;
//	pulse_width = h->m_pre_pulse_width;
//	while (pulse_width>=0) {
//  	pulse_width -= 20;        // 20 milliseconds per sample
//		epoch_num++;             // samples number in one heart rate pulse
//	}

//	N_SPRINTF("[PPG] sample num: %d", epoch_num);

//	// 50bpm: 60 samples
//	// 60bpm: 50 samples
//	// 70bpm: 43 samples
//	// 80bpm: 38 samples
//	// 90bpm: 33 samples
//	// 100bpm: 30 samples

//	for (i=7; i>0; i--)
//	  h->m_epoch_num[i] = h->m_epoch_num[i-1];

////h->m_epoch_num[0] = epoch_num;
//	h->m_epoch_num[0] = Kalman_Filter(epoch_num);

//  //-------------------------------------------------------
//	// calculate average
//	sum = 0;
//	non_num = 0;
//	for (i=0; i<8; i++)
//	{
//		if (h->m_epoch_num[i]!=0)
//		{
//			non_num++;
//			sum += h->m_epoch_num[i];
//		}
//	}

//	avg_epoch_num = 0;
//	while (sum>=0)
//	{
//		sum -= non_num;
//		avg_epoch_num++;
//	}
//  //-------------------------------------------------------

//  one_minute = 3000;
//	heart_rate = 0;
//	while (one_minute>=0)
//	{
//		one_minute -= avg_epoch_num;
//		heart_rate++;
//	}

//	N_SPRINTF("%d", heart_rate);

//	h->update_cnt++;

//	if ((h->update_cnt)&0x1) {
//		h->current_rate = heart_rate;
//		N_SPRINTF("[PPG] current rate: %d", h->current_rate);
//	}
//}

//static void _reset_heart_rate_calculator()
//{
//	HEARTRATE_CTX *h = &cling.hr;
//	I8U  i;
//	I8U  epoch_num;
//	I16S pulse_width;

//	epoch_num = 0;
//	pulse_width = h->m_pre_pulse_width;
//	while (pulse_width>=0) {
//		pulse_width -= 20;
//		epoch_num++;
//	}
//	for (i=0; i<8; i++) h->m_epoch_num[i] = epoch_num;
//}

//static BOOLEAN _heart_rate_overflow(I16S epoch_num)
//{
//	HEARTRATE_CTX *h = &cling.hr;
//	I32S t_curr;

//	t_curr = CLK_get_system_time();

////if ( epoch_num<(15*20) || epoch_num>(67*20) ) {      // 15=(60*50)/(200rpm), 67=(60*50)/(45rpm)
//	if ( epoch_num<(  300) || epoch_num>( 1340) ) {      //  6=(60*25)/(200rpm), 27=(60*25)/(45rpm)
//		h->m_pre_zero_point  = t_curr;
////	h->m_pre_pulse_width = 0;
//		return TRUE;
//	}
//	return FALSE;
//}

//static void _seek_cross_point(I16S instSample)
//{
//	HEARTRATE_CTX *h = &cling.hr;
//	I32S t_curr;
//	I32S t_diff;
//	I32S t_peak_diff;

//	I32U pulse_width_th1 = 0;
//	I32U pulse_width_th2 = 0;
//	I32U pulse_width_th3 = 0;

//	t_curr = CLK_get_system_time();
//	t_diff = t_curr - h->m_zero_point_timer;
//	t_peak_diff = t_curr - h->m_pre_zero_point;

//	if (h->m_pre_ppg_sample==0) {
//		h->m_pre_ppg_sample = instSample;
//		h->m_zero_point_timer = CLK_get_system_time();
//		return;
//	}

//  if (t_diff>(20*2)) {
//  	h->m_zero_point_timer = CLK_get_system_time();

//  	if (h->m_pre_ppg_sample<0 && instSample>0) {
//  		if (h->m_pre_zero_point==0) {
//  			h->m_pre_zero_point = t_curr;
//  			h->m_pre_ppg_sample = instSample;
//  			return;
//  		} else {
//  			if (h->m_pre_pulse_width==0) {
//  				h->m_pre_ppg_sample = instSample;

//  				if (_heart_rate_overflow(t_peak_diff))
//  					return;

//  				h->m_pre_zero_point  = t_curr;
//  				h->m_pre_pulse_width = t_peak_diff;
//					h->heart_rate_ready  = TRUE;
//					// The time of latest heart rate measured
//  				_reset_heart_rate_calculator();
//  				_calc_heart_rate();
//  				return;
//  			} else {
//					pulse_width_th1 = (h->m_pre_pulse_width) - ((h->m_pre_pulse_width)>>3);
//					pulse_width_th2 = (h->m_pre_pulse_width) + ((h->m_pre_pulse_width)>>3);
//					pulse_width_th3 = (h->m_pre_pulse_width) + ((h->m_pre_pulse_width)>>1);
//  			}

//  			if (
//  				   (t_peak_diff<pulse_width_th1) ||
//  				   (t_peak_diff>pulse_width_th2 && t_peak_diff<pulse_width_th3)
//  				 )
//  			{
//  				h->m_pre_ppg_sample = instSample;
//  				return;
//  			}
//  			else if (t_peak_diff>pulse_width_th3) {
//  				h->m_pre_ppg_sample = instSample;
//  				t_peak_diff >>= 1;

//  				if (_heart_rate_overflow(t_peak_diff))
//  					return;

//  				h->m_pre_zero_point = t_curr;
//  				h->m_pre_pulse_width = t_peak_diff;
//  				_calc_heart_rate();
//  				return;
//  			}

//  			if (_heart_rate_overflow(t_peak_diff))
//  				return;

//  			h->m_pre_zero_point = t_curr;
//  			h->m_pre_pulse_width = t_peak_diff;
//  			_calc_heart_rate();
//  		}
//  	}
//  	h->m_pre_ppg_sample = instSample;
//  }
//}

//static void _ppg_sample_proc()
//{
//	HEARTRATE_CTX *h = &cling.hr;

//	double high_pass_filter_val = 0.0;
//	double low_pass_filter_val  = 0.0;
//	I16S sample = 0;
//	I16S filt_sample = 0;
//	I32U t_sec = cling.time.system_clock_in_sec;
//	I32U t_diff;

//	t_diff = t_sec - h->m_measuring_timer_in_s;

//	sample = _get_light_strength_register();

//	high_pass_filter_val = Butterworth_Filter_HP( (double) sample);
//	low_pass_filter_val  = Butterworth_Filter_LP(high_pass_filter_val);
//	filt_sample = (I16S)low_pass_filter_val;

//  if (t_diff> 6) {
////if (TRUE) {
//		N_SPRINTF("%d  %d", filt_sample, sample);
//		N_SPRINTF("%d",      sample);

//		/* check whether current epoch ppg sample is zero-crossing */
//  	_seek_cross_point(filt_sample);
//	}
//}
///*------------ !!! PPG SUBROUTINES END   !!! ------------*/



///*------------ !!! BASIC SUBROUTINES FOR LT1PH03 DRIVER START !!! ------------*/
//void ppg_Send_Command(I8U cmd)      { ppg_write_reg(REGS_COMMAND, cmd); }
//void ppg_Send_Parameter(I8U param)  { ppg_write_reg(REGS_PARAM_WR, param); }
//void ppg_Send_Command_Parameter(I8U cmd, I8U param)
//{
//	ppg_Send_Parameter(param);
//	ppg_Send_Command(cmd);
//}
//void ppg_Send_Command_Query(I8U offset, I8U param) { ppg_Send_Command_Parameter((offset+0x80), param); }
//void ppg_Send_Command_SET  (I8U offset, I8U param) { ppg_Send_Command_Parameter((offset+0xA0), param); }
//void ppg_Send_Command_AND  (I8U offset, I8U param) { ppg_Send_Command_Parameter((offset+0xC0), param); }
//void ppg_Send_Command_OR   (I8U offset, I8U param) { ppg_Send_Command_Parameter((offset+0xE0), param); }
//void ppg_Send_Command_NOP()    { ppg_Send_Command(0x00); }
//void ppg_Send_Command_Reset()  { ppg_Send_Command(0x01); }
//void ppg_PS_Force()            { ppg_Send_Command(0x05); }
//void ppg_ALS_Force()           { ppg_Send_Command(0x06); }
//void ppg_PS_ALS_Force()        { ppg_Send_Command(0x07); }
//void ppg_PS_Pause()            { ppg_Send_Command(0x09); }
//void ppg_ALS_Pause()           { ppg_Send_Command(0x0a); }
//void ppg_PS_ALS_Pause()        { ppg_Send_Command(0x0B); }
//void ppg_PS_Auto()             { ppg_Send_Command(0x0D); }
//void ppg_ALS_Auto()            { ppg_Send_Command(0x0E); }
//void ppg_PS_ALS_Auto()         { ppg_Send_Command(0x0F); }
//void ppg_Modify_Command_I2C_Addr(I8U i2c_addr)
//{
//	ppg_Send_Command_SET(I2C_ADDR, i2c_addr);
//	ppg_Send_Command(0x02);
//}
///*------------ !!! BASIC SUBROUTINES FOR LT1PH03 DRIVER END   !!! ------------*/

//void ppg_Init_Sensor()
//{
//  I8U reg_defaults[] = {
//  	0x00,                                                                 // part id
//	  0x00,                                                                 // rev id
//	  0x00,                                                                 // seq id
//  	B_INT_STICKY      | B_INT_OUTPUT_DISABLE,                             // int cfg
//  	B_CMD_INT_ENABLE  | B_PS1_INT_ENBALE       | B_ALS_INT_DISABLE,       // irq enable
//  	B_PS1_IM_COMPLETE | B_ALS_IM_NONE,                                    // int mode1
//  	B_CMD_INT_ERR,                                                        // int mode2
//  	0x17,                                                                 // hw key
//	  B_MEAS_RATE_10MS,                                                     // meas rate
//  	B_ALS_RATE_FORCE,                                                     // als rate
//  	B_PS_RATE_1,                                                          // ps rate
//	  0x00,                                                                 // als low  th  7:0
//  	0x00,                                                                 // als low  th 15:8
//  	0x00,                                                                 // als high th  7:0
//  	0x00,                                                                 // als high th 15:8
////  B_PS_LED_NONE,                                                        // ps led drive current
////  B_PS_LED_5_6MA,
////  B_PS_LED_11_2MA,
////  B_PS_LED_22_4MA,
//    B_PS_LED_44_8MA,
//  };

//	BASE_delay_msec(50);

//	ppg_Send_Command_Reset();

//  BASE_delay_msec(10);

//  ppg_write_reg(REGS_HW_KEY, reg_defaults[REGS_HW_KEY]);

//	ppg_Send_Command_SET(CHLIST,0x01);        //  enble ps only

//	ppg_write_reg(REGS_INT_CFG,    reg_defaults[REGS_INT_CFG]);
//	ppg_write_reg(REGS_IRQ_ENABLE, reg_defaults[REGS_IRQ_ENABLE]);
//	ppg_write_reg(REGS_IRQ_MODE1,  reg_defaults[REGS_IRQ_MODE1]);
//	ppg_write_reg(REGS_IRQ_MODE2,  reg_defaults[REGS_IRQ_MODE2]);
//	ppg_write_reg(REGS_MEAS_RATE,  reg_defaults[REGS_MEAS_RATE]);
//	ppg_write_reg(REGS_ALS_RATE,   reg_defaults[REGS_ALS_RATE]);
//	ppg_write_reg(REGS_PS_RATE,    reg_defaults[REGS_PS_RATE]);
//	ppg_write_reg(REGS_PS_LED21,   reg_defaults[REGS_PS_LED21]);
//}

//void ppg_Enable_Sensor()
//{
//	ppg_write_reg(REGS_IRQ_STATUS, 0x27);  // Clear all int
//	ppg_write_reg(REGS_INT_CFG, B_INT_STICKY|B_INT_OUTPUT_ENABLE);

//	ppg_Send_Command_SET(CHLIST,0x01);         // Enble ps only
//	ppg_PS_Auto();

//#if 1
//	// TBD: Need Tuning...
//  ppg_Send_Command_SET(PS_ENCODING,  0x60);       // LSB 16 bits of 17bit ADC
//  ppg_Send_Command_SET(PS_ADC_GAIN,  0x03);
//  ppg_Send_Command_SET(PS1_ADCMUX,   0x00);
//  ppg_Send_Command_SET(PS_ADC_MISC,  0x04);

////ppg_Send_Command_SET(ALS_ENCODING, 0x30);
////ppg_Send_Command_SET(PS1_ADCMUX,   0x00);
//#endif
//}

//void ppg_Disable_Sensor()
//{
//	HEARTRATE_CTX *h = &cling.hr;

//	/* pause */
////ppg_PS_Pause();
////ppg_ALS_Pause();
//  ppg_PS_ALS_Pause();

//	/* disable mearing rate */
//	ppg_write_reg(REGS_MEAS_RATE, 0);
//	ppg_write_reg(REGS_ALS_RATE,  0);
//	ppg_write_reg(REGS_PS_RATE,   0);

//	h->heart_rate_ready  = FALSE;
//}

//void ppg_configure_sensor()
//{
//	HEARTRATE_CTX *h = &cling.hr;

//	// 1. Initlize and Enable sensor.
//  ppg_Init_Sensor();
//  ppg_Enable_Sensor();

//	// 2. Initlize ppg context variables each measuring stage.
//	h->m_pre_zero_point  = 0;
//	h->m_pre_ppg_sample  = 0;
//	h->m_pre_pulse_width = 0;
//	h->update_cnt        = 0;
//}

//I16S _get_light_strength_register()
//{
//	I16S proxData;
//	I8U  buf[2];

//	ppg_read_reg(REGS_PS1_DATA0,  1, (buf+0));
//	ppg_read_reg(REGS_PS1_DATA1,  1, (buf+1));

//	proxData = ((I16S)buf[1]<<8) | buf[0];

//	return proxData;
//}
	I8U buf[64];
void PPG_init()
{
//	pin_set_low(GPIO_PPG_ON);
//	BASE_delay_msec(100);
//		uint32_t cmd = 0x40000020;
//	uint8_t cmd2[] = {0x40,0x00,0x00,0x00};
//		int i = 0;
//		 uint32_t data = 0x00000003;
//	  ppg_write_reg((uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&data, sizeof(data));
//   	ppg_read_reg((uint8_t *)&cmd, sizeof(cmd), buf ,4);

////		buf[i++] = 0x01;
////		buf[i++] = 0x01;
////		buf[i++] = 0x01;
////		buf[i++] = 0x01;
//	// cmd = 0x40000060;
//	// data = (1<<17);
//	
//	
//  //		ppg_write_reg((uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&data, sizeof(data));
// //  		ppg_read_reg((uint8_t *)&cmd, sizeof(cmd), buf ,4);

//	  cmd = 0x40010008;
//	  ppg_read_reg((uint8_t *)&cmd, sizeof(cmd), buf ,4);
as7000Info_t infoRecord;
comOpen ( &infoRecord );
 }

//void PPG_state_machine()
//{
//	HEARTRATE_CTX *h = &cling.hr; 
//	I32U t_curr, t_diff, t_threshold, t_sec;

//	if (OTA_if_enabled()) {
//		return;
//	}

//	t_sec = cling.time.system_clock_in_sec;

//	// PPG measuring main state machine
//	switch (h->state) {

//		case PPG_STAT_IDLE:
//			h->state = PPG_STAT_DUTY_OFF;
//		  break;

//		case PPG_STAT_DUTY_ON:
//		{
//			// Update time stamp to start measuring right away
//			h->m_measuring_timer_in_s = t_sec;
//			h->m_no_skin_touch_in_s = t_sec;
//			h->state = PPG_STAT_SAMPLE_READY;
//			N_SPRINTF("[PPG] duty on: %d", h->m_measuring_timer_in_s);
//  	  ppg_configure_sensor();			// initialize the PPG module
//		}
//		break;

//		case PPG_STAT_SAMPLE_READY:
//		{

//			// If skin touch is positive, go ahead to measure PPG
//			if (!TOUCH_is_skin_touched()) {
//				t_diff = t_sec - h->m_no_skin_touch_in_s;

//				if (t_diff > 2) {
//          ppg_Disable_Sensor();					       // Turn off ppg sensor module
//          h->state = PPG_STAT_DUTY_OFF;
//          N_SPRINTF("[PPG] PPG (No skin touch) State off, %d, %d", t_diff, cling.ui.state);
//				}

//				break;
//			}

//			h->m_no_skin_touch_in_s = t_sec;

//		  RTC_start_operation_clk();
//		  if (h->sample_ready) {
//			  h->sample_ready = FALSE;
//				N_SPRINTF("[PPG] wearing confirmed, %d, %d", t_sec, h->m_measuring_timer_in_s);

//				// Check if measuring is timed up
//				t_diff = t_sec - h->m_measuring_timer_in_s;
//				if (t_diff > PPG_SAMPLE_PROCESSING_PERIOD) {
//          _ppg_sample_proc();				    // Process current sample
//        }

//  			if (t_diff > PPG_HR_MEASURING_TIMEOUT && !_is_user_viewing_heart_rate() ) {
//          ppg_Disable_Sensor();					       // Turn off ppg sensor module
//          h->state = PPG_STAT_DUTY_OFF;
//          N_SPRINTF("[PPG] PPG (TIME OUT) State off, %d, %d", t_diff, cling.ui.state);
//				}
//			}
//		}
//		break;

//		case PPG_STAT_DUTY_OFF:
//		{
//			// If skin touch is positive, go ahead to measure PPG
//			if (!TOUCH_is_skin_touched())
//				break;

//			t_diff = t_sec - h->m_measuring_timer_in_s;
//			if (_is_user_viewing_heart_rate() || cling.activity.b_workout_active) {
//				t_threshold = PPG_MEASURING_PERIOD_FOREGROUND;
//				if ( t_diff > t_threshold ) {
//						h->state = PPG_STAT_DUTY_ON;
//						N_SPRINTF("[PPG] PPG State on (force)");
//				}
//			} else {
//				//t_threshold = cling.user_data.ppg_day_interval>>10;
//				t_threshold = PPG_MEASURING_PERIOD_BACKGROUND_DAY;

//				// If user is asleep, we reduce the duty cycle by 2
//				if ((cling.sleep.state == SLP_STAT_LIGHT) ||
//						(cling.sleep.state == SLP_STAT_SOUND) ||
//						(cling.sleep.state == SLP_STAT_REM))
//				{
//					//t_threshold = cling.user_data.ppg_night_interval>>10;
//					t_threshold = PPG_MEASURING_PERIOD_BACKGROUND_NIGHT;
//					N_SPRINTF("[PPG] sleeping mode: %d", t_threshold);
//				}

//				// Normal background heart detection requires low power stationary mode
//				if (t_diff < t_threshold) {
//					break;
//				}

//				// Make sure it is in a low power stationary mode
//				if (!cling.lps.b_low_power_mode) {
//					break;
//				}

//				// State in low power stationary for more than 5 seconds
//				t_curr = CLK_get_system_time();
//				if (t_curr < (cling.lps.ts + PPG_WEARING_DETECTION_LPS_INTERVAL)) {
//					break;
//				}

//				N_SPRINTF("[PPG] start ppg, %d, %d", t_sec, h->m_measuring_timer_in_s);

//				// otherwise, using optical sensor to double check
//				h->state = PPG_STAT_DUTY_ON;
//			}
//		}
//		break;

//		default:
//			break;
//	}
//}


//I8U PPG_minute_hr_calibrate()
//{
//	I8U hr_diff;

//	if ((cling.hr.minute_rate < 90) && (cling.hr.current_rate < 90))
//		return cling.hr.current_rate;

//	if (cling.hr.current_rate > cling.hr.minute_rate) {
//		hr_diff = cling.hr.current_rate - cling.hr.minute_rate;
//		if (hr_diff < 10) {
//			return cling.hr.current_rate;
//		} else {
//			return (cling.hr.minute_rate+(hr_diff&0x07));
//		}
//	} else {
//		hr_diff = cling.hr.minute_rate-cling.hr.current_rate;

//		if (hr_diff < 16)
//			return cling.hr.current_rate;
//		else
//			return (cling.hr.minute_rate - (hr_diff&0x07));
//	}
//}
