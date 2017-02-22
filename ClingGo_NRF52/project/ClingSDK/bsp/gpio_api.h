/***************************************************************************//**
 * File gpio.h
 * 
 * Description: User Interface header
 *
 *
 ******************************************************************************/

#ifndef __GPIO_HEADER__
#define __GPIO_HEADER__

#include "standards.h"



// Initialization
void GPIO_api_init(void);

// ADC analog pin configuration for battery
I16U GPIO_vbat_adc_reading(void);

// TWI pin configuration
void GPIO_twi_init(I8U twi_master_instance);
void GPIO_twi_enable(I8U twi_master_instance);
void GPIO_twi_disable(I8U twi_master_instance);

// Vibrator control
void GPIO_vibrator_set(BOOLEAN b_on);
void GPIO_vibrator_on_block(I8U latency);

// Interrupt control
void GPIO_interrupt_enable(void);

// Charger reset
void GPIO_charger_reset(void);
void GPIO_system_powerup(void);
void GPIO_system_powerdown(void);

// Interrupt handle 
void GPIO_interrupt_handle(void);

#endif
