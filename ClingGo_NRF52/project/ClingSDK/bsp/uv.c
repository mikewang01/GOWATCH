
#include "main.h"

#ifdef _CLINGBAND_UV_MODEL_
#define UV_MEASURING_PERIOD_FOREGROUND 1
#define UV_MEASURING_PERIOD_BACKGROUND 100

static EN_STATUSCODE uv_read_reg(I8U cmdID, I8U bytes, I8U *pRegVal)
{
#ifndef _CLING_PC_SIMULATION_

	I32U err_code;
	const nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(1);

	if (pRegVal==NULL) return STATUSCODE_NULL_POINTER;

	if (!cling.system.b_twi_1_ON) {
		GPIO_twi_enable(1);
	}

	err_code = nrf_drv_twi_tx(&twi, (uv_I2C_ADDR>>1), &cmdID, 1, true);
	if (err_code == NRF_SUCCESS) {
		N_SPRINTF("[UV] Read TX PASS: ");
	} else {
		Y_SPRINTF("[UV] Read TX FAIL - %d", err_code);
		APP_ERROR_CHECK(err_code);
		return STATUSCODE_FAILURE;
	}
	err_code = nrf_drv_twi_rx(&twi, (uv_I2C_ADDR>>1), pRegVal, bytes, false);
	if (err_code == NRF_SUCCESS) {
		N_SPRINTF("[UV] Read RX PASS: ");
		return STATUSCODE_SUCCESS;
	} else {
		Y_SPRINTF("[UV] Read RX FAIL: %d", err_code);
		APP_ERROR_CHECK(err_code);
		return STATUSCODE_FAILURE;
	}
#else
	return STATUSCODE_SUCCESS;
#endif
}

static EN_STATUSCODE uv_write_reg(I8U cmdID, I8U regVal)
{
#ifndef _CLING_PC_SIMULATION_

	I32U err_code;
	const nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(1);
	I8U acData[2];
	
	acData[0] = cmdID;
	acData[1] = regVal;
	
	if (!cling.system.b_twi_1_ON) {
		GPIO_twi_enable(1);
	}
	
	err_code = nrf_drv_twi_tx(&twi, (uv_I2C_ADDR>>1), acData, 2, false);
	if (err_code == NRF_SUCCESS) {
		N_SPRINTF("[UV] Write PASS: 0x%02x  0x%02x", cmdID, regVal);
		return STATUSCODE_SUCCESS;
	} else {
		Y_SPRINTF("[UV] Write FAIL(%d): 0x%02x  0x%02x", err_code, cmdID, regVal);
		APP_ERROR_CHECK(err_code);
		return STATUSCODE_FAILURE;
	}	
#else
	return STATUSCODE_SUCCESS;
#endif
}

/*------------ !!! BASIC SUBROUTINE FOR SI1132 DRIVER START !!! ------------*/
void uv_Send_Command(I8U cmd)      { uv_write_reg(uv_REGS_COMMAND, cmd); }
void uv_Send_Parameter(I8U param)  { uv_write_reg(uv_REGS_PARAM_WR, param); }
void uv_Send_Command_Parameter(I8U cmd, I8U param)
{
	uv_Send_Parameter(param);
	uv_Send_Command(cmd);
}
void uv_Send_Command_Query(I8U offset, I8U param) { uv_Send_Command_Parameter((offset+0x80), param); }
void uv_Send_Command_SET  (I8U offset, I8U param) { uv_Send_Command_Parameter((offset+0xA0), param); }    /* 
void uv_Send_Command_AND  (I8U offset, I8U param) { uv_Send_Command_Parameter((offset+0xC0), param); }
void uv_Send_Command_OR   (I8U offset, I8U param) { uv_Send_Command_Parameter((offset+0xE0), param); }    */
void uv_Send_Command_NOP()                        { uv_Send_Command(0x00); }
void uv_Send_Command_Reset()                      { uv_Send_Command(0x01); }
void uv_ALS_Force()                               { uv_Send_Command(0x06); }    // Forces a single ALS measurement
void uv_ALS_Pause()                               { uv_Send_Command(0x0a); }    // Pauses autonomous ALS
void uv_ALS_Auto()                                { uv_Send_Command(0x0E); }    /* 
void uv_PS_Force()                                { uv_Send_Command(0x05); }
void uv_PS_ALS_Force()                            { uv_Send_Command(0x07); }
void uv_PS_Pause()                                { uv_Send_Command(0x09); }
void uv_PS_ALS_Pause()                            { uv_Send_Command(0x0B); }
void uv_PS_Auto()                                 { uv_Send_Command(0x0D); }
void uv_PS_ALS_Auto()                             { uv_Send_Command(0x0F); }    */
void uv_Get_Cal()                                 { uv_Send_Command(0x12); }    // Reports calibration data to I2C registers 0x22 to 0x2D
void uv_Modify_Command_I2C_Addr(I8U i2c_addr)
{
  uv_Send_Command_SET(uv_PARAM_I2C_ADDR, i2c_addr);
  uv_Send_Command(0x02);
}
/*------------ !!! BASIC SUBROUTINE FOR SI1132 DRIVER END   !!! ------------*/

void uv_Reset()
{	
	BASE_delay_msec(50);

  uv_write_reg(uv_REGS_MEAS_RATE0, 0x00);
  uv_write_reg(uv_REGS_MEAS_RATE1, 0x00);
	
	uv_ALS_Pause();

  uv_write_reg(uv_REGS_MEAS_RATE0, 0x00);
  uv_write_reg(uv_REGS_MEAS_RATE1, 0x00);
	uv_write_reg(uv_REGS_IRQ_ENABLE, 0x00);
	uv_write_reg(uv_REGS_INT_CFG,    0x00);
	uv_write_reg(uv_REGS_IRQ_STATUS, 0xFF);
	
	uv_Send_Command_Reset();
	BASE_delay_msec(10);

	/* Hardware Reset. */
  uv_write_reg(uv_REGS_HW_KEY, 0x17);
}

void UV_Init()
{
	UV_CTX *uv = &cling.uv;

	I8U partid = 0xaa;
	I8U revid  = 0xbb;
	I8U seqid  = 0xcc;
	
	I8U coef0 = 0x11;
	I8U coef1 = 0x22;
	I8U coef2 = 0x33;
	I8U coef3 = 0x44;
	
  N_SPRINTF("[UV] initialization");
	
	uv_Reset();

//uv_Get_Calibration(1);
//uv_Set_UCoef(&si1132_cal);
	
	/* Set CHLIST. */
//uv_Send_Command_SET(uv_PARAM_CHLIST, 0x80);      // Enable UV Index only.
  uv_Send_Command_SET(uv_PARAM_CHLIST, 0x90);

	/* Set measure rate.                                                          */
	/* MEAS_RATE1 and MEAS_RATE0 together form a 16-bit value: MEAS_RATE[15:0].   */
	/* The 16-bit value, when multiplied by 31.25us, represents the time duration */
	/* between wake-up periods where measurements are made. Once the device wakes */
	/* up, all measurements specified in CHLIST are made                          */
  uv_write_reg(uv_REGS_MEAS_RATE0, 0x80);        // 50 samples per second
  uv_write_reg(uv_REGS_MEAS_RATE1, 0x02);

	/* Verify SI1132 ID registers. */
	uv_read_reg(uv_REGS_PART_ID, 1, &partid);
	uv_read_reg(uv_REGS_REV_ID,  1, &revid);
	uv_read_reg(uv_REGS_SEQ_ID,  1, &seqid);	
  N_SPRINTF("SI1132  0x%02x  0x%02x  0x%02x", partid, revid, seqid);
	BASE_delay_msec(1);

	/* Set Defualt Coefficient. */
  uv_write_reg(uv_REGS_MEAS_UCOEF0, 0x7B);
  uv_write_reg(uv_REGS_MEAS_UCOEF1, 0x6B);
  uv_write_reg(uv_REGS_MEAS_UCOEF2, 0x01);
  uv_write_reg(uv_REGS_MEAS_UCOEF3, 0x00);
  BASE_delay_msec(1);

  uv_read_reg(uv_REGS_MEAS_UCOEF0, 1, &coef0);        // BASE_delay_msec(10);
	uv_read_reg(uv_REGS_MEAS_UCOEF1, 1, &coef1);        // BASE_delay_msec(10);
	uv_read_reg(uv_REGS_MEAS_UCOEF2, 1, &coef2);        // BASE_delay_msec(10);
	uv_read_reg(uv_REGS_MEAS_UCOEF3, 1, &coef3);        // BASE_delay_msec(10);
  N_SPRINTF("SI1132 first  time  0x%02x  0x%02x  0x%02x  0x%02x", coef0, coef1, coef2, coef3);

// TBD: debugging...
  uv_Send_Command_SET(uv_PARAM_ALS_IR_ADCMUX,       MUX_SMALL_IR);       // BASE_delay_msec(10);
  uv_Send_Command_SET(uv_PARAM_ALS_IR_ADC_GAIN,     0);
  uv_Send_Command_SET(uv_PARAM_ALS_VIS_ADC_GAIN,    0);
  uv_Send_Command_SET(uv_PARAM_ALS_IR_ADC_COUNTER,  RECCNT_511);         // 0x70
  uv_Send_Command_SET(uv_PARAM_ALS_VIS_ADC_COUNTER, RECCNT_511);         // 0x70
  uv_Send_Command_SET(uv_PARAM_ALS_IR_ADC_MISC,     0x20);               // 0x20, high signal range
  uv_Send_Command_SET(uv_PARAM_ALS_VIS_ADC_MISC,    0x20);               // 0x20, high signal range

	/* Set SI1132 autonomous mode. */
  uv_Disable_Sensor();
//uv_ALS_Auto();

#if 0
  I8U vis_d0, vis_d1;
  I8U ir_d0,  ir_d1;
	I8U uv_d0,  uv_d1;
	
	I16S vis;
	I16S ir;
	I16S uv;

  uv_Get_Cal();
	BASE_delay_msec(1);

	uv_read_reg(uv_REGS_ALS_VIS_DATA0, 1, &vis_d0);
	uv_read_reg(uv_REGS_ALS_VIS_DATA1, 1, &vis_d1);
	uv_read_reg(uv_REGS_ALS_IR_DATA0,  1, &ir_d0);
	uv_read_reg(uv_REGS_ALS_IR_DATA1,  1, &ir_d1);
	uv_read_reg(uv_REGS_AUX_DATA0_UVIDX0,  1, &uv_d0);
	uv_read_reg(uv_REGS_AUX_DATA1_UVIDX1,  1, &uv_d1);

	vis = vis_d1 * 256 + vis_d0;
	ir  = ir_d1  * 256 + ir_d0;
	uv  = uv_d1  * 256 + uv_d0;
	
	N_SPRINTF("%04x %02x %02x", vis, vis_d1, vis_d0);
	N_SPRINTF("%04x %02x %02x", ir,  ir_d1,  ir_d0);
	N_SPRINTF("%04x %02x %02x", uv,  uv_d1,  uv_d0);
#endif

  /* UV state machine context intilization */
	uv->uv_index = 0;
	uv->state = UV_STAT_IDLE;
	uv->measure_timebase = cling.time.system_clock_in_sec;
}

void uv_Configure_Sensor()
{
  uv_write_reg(uv_REGS_MEAS_RATE0, 0x80);        // 50 samples per second
  uv_write_reg(uv_REGS_MEAS_RATE1, 0x02);
	uv_ALS_Auto();
}

void uv_Disable_Sensor()
{
	/* disable mearing rate */
  uv_write_reg(uv_REGS_MEAS_RATE0, 0x00);
  uv_write_reg(uv_REGS_MEAS_RATE1, 0x00);

	/* pause */
	uv_ALS_Pause();
}

BOOLEAN uv_Measure()
{
	I8U  buf[2];
//I8U  chip_stat;

	I32S proxData;
	I32S realUV;
	I32S uv_idx;

	UV_CTX *uv = &cling.uv;
	
	buf[0] = 0x11;
	buf[1] = 0x22;

#if 1
  uv_read_reg(uv_REGS_AUX_DATA0_UVIDX0,  1, (buf+0));
  uv_read_reg(uv_REGS_AUX_DATA1_UVIDX1,  1, (buf+1));
  
  proxData = buf[1] * 256 + buf[0];
  N_SPRINTF("%d", proxData);
	
	// 
	// assuming our Lemon_Band's cover lens will cause 
	// 65% lost of the measured uv index, 
	// that is, 
	// proxData = real_uv * 0.35
	// a.k.a 
	// real_uv = proxData / 0.35
	//
	
	realUV = proxData * 100;
	uv_idx = 0;
	while ( realUV>=35 ) {
		uv_idx++;
		realUV -= 35;
	}

	uv_idx /= 10;
  uv->uv_index = (I8U)uv_idx;
	
	if (uv->max_uv < uv->uv_index)
		uv->max_uv = uv->uv_index;
	
	if (UI_is_idle()) {
		uv->max_UI_uv = uv->uv_index;
	} else {
		if (uv->uv_index > uv->max_UI_uv)
			uv->max_UI_uv = uv->max_uv;
	}
  N_SPRINTF("%d    %d    %d", uv->uv_index, uv_idx, proxData);
#endif

#if 0
  uv_read_reg(uv_REGS_ALS_VIS_DATA0,  1, (buf+0));
  uv_read_reg(uv_REGS_ALS_VIS_DATA0,  1, (buf+1));
  uv_read_reg(uv_REGS_ALS_VIS_DATA1,  1, (buf+2));
  uv_read_reg(uv_REGS_RESPONSE,       1, (buf+3));
  
  proxData = buf[2] * 256 + buf[1];
  
  N_SPRINTF("0x%02x  0x%02x  0x%02x  0x%02x", buf[3], buf[2], buf[1], buf[0]);
  N_SPRINTF("%d", proxData);

  chip_stat = 0x55;
  uv_read_reg(uv_REGS_CHIP_STAT,  1, &chip_stat);
  N_SPRINTF("0x%02x", chip_stat);
#endif

  return TRUE;
}

I8U UV_get_max_index_per_minute()
{
	I8U max = cling.uv.max_uv;
	cling.uv.max_uv = cling.uv.uv_index;
	
	return max;
}

BOOLEAN _is_user_viewing_uv_index()
{
	if (UI_is_idle()) {
		return FALSE;
	}
	
	if (cling.ui.frame_index != UI_DISPLAY_TRACKER_UV_IDX) {
		return FALSE;
	}

	return TRUE;
}

void UV_state_machine()
{
	UV_CTX *uv = &cling.uv;
	I32U t_diff;
	
	// No need to run UV state machine if user is in a sleep state
	if ( SLEEP_is_sleep_state(SLP_STAT_LIGHT) || SLEEP_is_sleep_state(SLP_STAT_SOUND) ) {
		uv->state = UV_STAT_IDLE;
		return;
	}

	switch (uv->state) {

		case UV_STAT_IDLE : 
			uv->state = UV_STAT_DUTY_OFF;
			break;

		case UV_STAT_DUTY_ON : 
			uv_Configure_Sensor();
		  uv->state = UV_STAT_SAMPLE_READY;
			break;

		case UV_STAT_SAMPLE_READY : 
      uv_Measure();
		  uv_Disable_Sensor();
			uv->state = UV_STAT_DUTY_OFF;
			break;

		case UV_STAT_DUTY_OFF : 
			t_diff = cling.time.system_clock_in_sec - uv->measure_timebase;
			
	  	if (_is_user_viewing_uv_index()) {
				N_SPRINTF("[UV] duty off view -> %d, %d", t_diff, cling.user_data.skin_temp_day_interval);
	  		if (t_diff > UV_MEASURING_PERIOD_FOREGROUND) {
	  			uv->state = UV_STAT_DUTY_ON;
					N_SPRINTF("[UV] duty off -> ON, view screen, %d", cling.ui.state);
	  		}
	  	} else {
				N_SPRINTF("[UV] duty off normal-> %d, %d", t_diff, cling.user_data.skin_temp_day_interval);
    		if ( t_diff >  UV_MEASURING_PERIOD_BACKGROUND ) {
					if (!uv->b_perform_measure)
						break;
					uv->b_perform_measure = FALSE;
					uv->measure_timebase = cling.time.system_clock_in_sec;
					uv->state = UV_STAT_DUTY_ON;
					N_SPRINTF("[UV] duty off -> ON, normal");
	  		}
	  	}

			break;

		default:
			break;
	}
}

#endif
