/***************************************************************************/
/**
 * File: gpio.c
 * 
 * Description: All GPIO related for Wristband device
 *
 * Created on Jan 6, 2014
 *
 ******************************************************************************/

#include "main.h"
#include "pinmap.h"
void GPIO_vibrator_set(BOOLEAN b_on);
void GPIO_system_powerup()
{
#ifndef _CLING_PC_SIMULATION_	
//	_gpio_cfg_output(GPIO_CHARGER_SD      , TRUE );   // Charger Shut down pin, pull UP to power up system
	
	cling.system.b_powered_up = TRUE;
#endif	
}

void GPIO_system_powerdown()
{
#ifndef _CLING_PC_SIMULATION_
	// System power down.
	//_gpio_cfg_output(GPIO_CHARGER_SD      , FALSE );  // Charger Shut down pin, pull down

  //cling.system.b_powered_up = FALSE;
	
  // Disconnecting all of the GPIO pin,in addition to CHARGER SD pin.

#endif
}

/*
 * this procedure is gonna be done in system initiate process before schedule triggered
 * */
void GPIO_api_init()
{
#ifndef _CLING_PC_SIMULATION_

#endif
}


void GPIO_interrupt_enable()
{
#ifndef _CLING_PC_SIMULATION_
#endif
}

void GPIO_twi_init(I8U twi_master_instance)
{

}
void GPIO_twi_enable(I8U twi_master_instance)
{
}

void GPIO_twi_disable(I8U twi_master_instance)
{
}

I16U GPIO_vbat_adc_reading()
{    
	I32U adc_sample = 0;
//
//	adc_calibrate();
//
//	adc_init(GP_ADC_SE, GP_ADC_SIGN, GP_ADC_ATTN3X);
//	adc_usDelay(20);
//
//	adc_enable_channel(ADC_CHANNEL_P01);
//
//	adc_sample = adc_get_sample();
//
//  adc_disable();
	
	return adc_sample;
}


void GPIO_therm_adc_config(void)
{
#ifndef _CLING_PC_SIMULATION_
#if 0	
  const nrf_adc_config_t nrf_adc_config = {
		.resolution = NRF_ADC_CONFIG_RES_10BIT,
		.scaling = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD,
		.reference = NRF_ADC_CONFIG_REF_VBG};

	// Initialize and configure ADC
	nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
#endif	
#endif
}

void GPIO_vibrator_on_block(I8U latency)
{
#ifndef _CLING_PC_SIMULATION_
	I32U t_curr = CLK_get_system_time();
	I32U t_diff = 0;
	RTC_start_operation_clk();
	//_gpio_cfg_output(GPIO_VIBRATOR_EN, TRUE);
	//GPIO_vibrator_set(TRUE);
	while (t_diff < latency) {
		// Feed watchdog every 4 seconds upon RTC interrupt
		//Watchdog_Feed();

		//sd_app_evt_wait();
		t_diff = CLK_get_system_time();
		t_diff -= t_curr;
		if (t_diff > latency)
			break;
	}
	
	//_gpio_cfg_output(GPIO_VIBRATOR_EN, FALSE);
	GPIO_vibrator_set(FALSE);
#endif
}

void GPIO_vibrator_set(BOOLEAN b_on)
{
        if (b_on)
                pin_set_high(GPIO_VIBRATOR_EN);
        else
                pin_set_low(GPIO_VIBRATOR_EN);
}

void GPIO_charger_reset()
{
#ifndef _CLING_PC_SIMULATION_

#endif
}

void GPIO_interrupt_handle()
{

}
