/******************************************************************************
 * Copyright 2013-2016 hicling Systems (MikeWang)
 *
 * FileName: pinmap.c
 *
 * Description:  abstract a layer between rtos and cling sdk for pin driver.
 *
 * Modification history:
 *     2016/7/24, v1.0 create this file mike.
 *******************************************************************************/
#include "pinmap.h"
				
static const PinMap cling_pin_assign[] =
        {
                { .pin = GPIO_SPI_SCK, .direction = PIN_OUTPUT_PUSH_PULL, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true , .int_level = NRF_GPIO_PIN_NOSENSE},
                { .pin = GPIO_SPI_MISO, .direction = PIN_INPUT, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true, .int_level = NRF_GPIO_PIN_NOSENSE},
                { .pin = GPIO_SPI_MOSI, .direction = PIN_OUTPUT_PUSH_PULL, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true, .int_level = NRF_GPIO_PIN_NOSENSE },
								{ .pin = GPIO_FLASH_CS, .direction = PIN_OUTPUT_PUSH_PULL, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true, .int_level = NRF_GPIO_PIN_NOSENSE},
                { .pin = GPIO_SPI_CS_ACC, .direction = PIN_OUTPUT, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true, .int_level = NRF_GPIO_PIN_NOSENSE },
                { .pin = GPIO_OLED_RST, .direction = PIN_OUTPUT_PUSH_PULL, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true, .int_level = NRF_GPIO_PIN_NOSENSE },
#ifdef CONFIG_RETARGET
                { .pin = GPIO_UART_DBG_TX, .direction = PIN_OUTPUT_OPEN_DRAIN, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true, .int_level = NRF_GPIO_PIN_NOSENSE },
                { .pin = GPIO_UART_DBG_RX, .direction = PIN_INPUT_PULL_UP, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true, .int_level = NRF_GPIO_PIN_NOSENSE },
#endif
								{ .pin = GPIO_OLED_A0, .direction = PIN_OUTPUT_OPEN_DRAIN, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true, .int_level = NRF_GPIO_PIN_NOSENSE },
								{ .pin = GPIO_OLED_VPP_ON, .direction = PIN_INPUT_PULL_UP, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true, .int_level = NRF_GPIO_PIN_NOSENSE },
                { .pin = ACC_SENSOR_INT, .direction = PIN_INPUT_PULL_UP, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true, .int_level = GPIO_PIN_CNF_SENSE_Low },

                { .pin = GPIO_TOUCH_INT, .direction = PIN_INPUT_PULL_UP, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true, .int_level = GPIO_PIN_CNF_SENSE_Low },
                { .pin = GPIO_CHG_IN, .direction = PIN_INPUT_PULL_UP, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true, .int_level = GPIO_PIN_CNF_SENSE_Low },
								
                { .pin = GPIO_PPG_ON, .direction = PIN_OUTPUT_PUSH_PULL,  .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = false, .int_level = NRF_GPIO_PIN_NOSENSE },


                { .pin = GPIO_TWI_CLK, .direction = PIN_OUTPUT_OPEN_DRAIN, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = false, .int_level = NRF_GPIO_PIN_NOSENSE },

                { .pin = GPIO_TWI_DATA, .direction = PIN_OUTPUT_OPEN_DRAIN, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = false, .int_level = NRF_GPIO_PIN_NOSENSE },

                { .pin = GPIO_VIBRATOR_EN, .direction = PIN_OUTPUT_PUSH_PULL, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = false, .int_level = NRF_GPIO_PIN_NOSENSE },
                { .pin = GPIO_TOUCH_PWR_EN, .direction = PIN_OUTPUT_PUSH_PULL, .function = PIN_FUNC_GPIO,
                        .voltage = PIN_POWER_V33, .high = true, .int_level = NRF_GPIO_PIN_NOSENSE },

        };

/*
 *
 *
 * Attenion: this functioon must obly be called by dialog preph initiated procedure
 *
 *
 * */
void pin_map_init()
{
        for (int i = 0; i < sizeof(cling_pin_assign) / sizeof(PinMap); i++)
        {
                if (cling_pin_assign[i].pin != NC)
                {
									
												nrf_gpio_pin_dir_t   dir;
												nrf_gpio_pin_pull_t  pull;
									      switch(cling_pin_assign[i].direction){
													case PIN_INPUT_PULL_UP: dir = NRF_GPIO_PIN_DIR_INPUT;  pull = NRF_GPIO_PIN_PULLUP; break;
													case PIN_INPUT: dir = NRF_GPIO_PIN_DIR_INPUT;  pull = NRF_GPIO_PIN_NOPULL;break;
													case PIN_INPUT_PULL_DOWN: dir = NRF_GPIO_PIN_DIR_INPUT;  pull = NRF_GPIO_PIN_PULLDOWN;break;
													case PIN_OUTPUT: dir = NRF_GPIO_PIN_DIR_OUTPUT;  pull = NRF_GPIO_PIN_NOPULL;break;
													case PIN_OUTPUT_PUSH_PULL: dir = NRF_GPIO_PIN_DIR_OUTPUT;  pull = NRF_GPIO_PIN_PULLUP;break;
													case PIN_OUTPUT_OPEN_DRAIN:dir = NRF_GPIO_PIN_DIR_OUTPUT;  pull = NRF_GPIO_PIN_NOPULL; break;
												
												}
												
												nrf_gpio_cfg(cling_pin_assign[i].pin, dir, NRF_GPIO_PIN_INPUT_CONNECT, pull, NRF_GPIO_PIN_S0S1, cling_pin_assign[i].int_level);
												nrf_gpio_pin_write(cling_pin_assign[i].pin, (cling_pin_assign[i].high == true));

                }
        }
}

void pin_set_high(PinName pin)
{
	nrf_gpio_pin_write(pin, 1);
//hw_gpio_set_active(GET_PORT_NUM(pin), GET_PIN_NUM(pin));
}

void pin_set_low(PinName pin)
{
	nrf_gpio_pin_write(pin, 0);
//hw_gpio_set_inactive(GET_PORT_NUM(pin), GET_PIN_NUM(pin));
}

int pin_get_status(PinName pin)
{
	return nrf_gpio_pin_read(pin);//(int)hw_gpio_get_pin_status(GET_PORT_NUM(pin), GET_PIN_NUM(pin));
}

void pin_toggle(PinName pin)
{
 nrf_gpio_pin_toggle(pin);
}
