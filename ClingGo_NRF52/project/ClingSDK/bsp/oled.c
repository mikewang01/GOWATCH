#include "main.h"
#include "pinmap.h"
#include "spidev_hal.h"
//#include "sys_power_mgr.h"

#define X_RESOLUTION     128
#define Y_RESOLUTION     128
#define OLED_I2C_RETRY_THREAHOLD   5

static OS_TIMER oled_timer;

static CLASS(SpiDevHal)* p_spi_instance = NULL;
extern void UI_task_notify_oled(void);
extern void ALERT_task_notific_notify(void);
static uint8_t g_i2c_tx_buf[65];


void OLED_power_on(void);
//I2C_SLAVE_ADDRESS:0x78
// Function for OLED write command.
extern void UI_task_notify_rendering(void);

uint16_t oled_get_x_solution()
{
        return X_RESOLUTION;
}
uint16_t oled_get_y_solution()
{
        return Y_RESOLUTION;
}

static void oled_tx_rx(I8U * tx_data_buf, I16U tx_data_size, I8U * rx_data_buf, I16U rx_data_size)
{

	CLASS(SpiDevHal)* p =  SpiDevHal_get_instance();
	spi_dev_handle_t t = p->oled_open(p);
	if(t != NULL){
			p->write_read(p, t, tx_data_buf, tx_data_size, rx_data_buf, rx_data_size);
	}
	p->close(p, t);

}

static void _set_reg(uint8_t command)
{
	uint8_t g_spi_tx_buf[16];
	uint8_t g_spi_rx_buf[16];
#ifndef _CLING_PC_SIMULATION_
	g_spi_tx_buf[0] = command;

	pin_set_low(GPIO_OLED_A0);

	oled_tx_rx(g_spi_tx_buf, 1, g_spi_rx_buf, 0);
#endif
}

// Function for OLED write data.
static void _set_data(I16U num, I8U *data)
{
	uint8_t g_spi_rx_buf[16];
#ifndef _CLING_PC_SIMULATION_
	pin_set_high(GPIO_OLED_A0);
	oled_tx_rx(data, num, NULL,0);  
#endif
}

void OLED_hw_reset(void)
{
#ifndef _CLING_PC_SIMULATION_
				OLED_power_off();
        BASE_delay_msec(10);
        OLED_power_on();
				BASE_delay_msec(10);
#endif
}

void OLED_power_on(void)
{
#ifndef _CLING_PC_SIMULATION_
        pin_set_high(GPIO_OLED_RST);
				pin_set_high(GPIO_OLED_VPP_ON);
#endif
}

void OLED_power_off()
{
#ifndef _CLING_PC_SIMULATION_
        // Pull down reset pin
        pin_set_low(GPIO_OLED_RST);
				pin_set_low(GPIO_OLED_VPP_ON);
#endif
}


void OLED_full_scree_show(I8U *pram)
{

#ifndef _CLING_PC_SIMULATION_
	I8U i;
	I8U *data;
	
	data = pram;
	
	for (i = 0; i < 4; i++) {
		_set_reg(0xb0+i);
		_set_reg(0x10);  // Starting address
		_set_reg(0x00);  // Bug fix - start from 0
		// Set "A0" pin for data buffer refresh
		_set_data(64,data);
		data += 64;
	}
#endif
}

void OLED_im_show(OLED_DISPLAY_MODE mode, I8U *pram, I8U offset)
{
        _set_reg(0xb0 + offset);
        _set_reg(0x10);  // Starting address
        _set_reg(0x02);  //
//		// Set "A0" pin for data buffer refresh
        _set_data(512, pram);
}

void OLED_set_display(I8U on_off)
{
        I8U buff[12];
        int k = 0;
        if (on_off) {
						_set_reg(0xAF);
						OLED_power_on();
						BASE_delay_msec(100);
            N_SPRINTF("OLED: display is set on ---- %d ---", CLK_get_system_time());
        }
        else {
							_set_reg(0xAE);
						  OLED_power_off();
						  BASE_delay_msec(100);
						  
        }
}

void OLED_set_contrast(I8U step)
{
        _set_reg(0x81);
        _set_reg(step);
}


void OLED_init(I8U contrast, BOOLEAN b_reverse)
{
        // Get instance
        //CLING_I2C_DEV_HAL_INIT();
        if (p_spi_instance == NULL) {
                p_spi_instance = SpiDevHal_get_instance();
        }

        //SH1106
        _set_reg(0xAE); //Set Display Off

        //display divide ratio/osc. freq. mode
        _set_reg(0xd5);
        _set_reg(0x50); // data set: 0%, 105Hz  ???? 470 KHz 0x50(???????)

        //multiplex ration mode:127
        _set_reg(0xA8);
        _set_reg(0x7f); //32

        //Set Display Offset
        _set_reg(0xD3);
        _set_reg(0x00);

        // _set_reg(0x01 );

        //Set Display Start Line (0x40--0x7F)
        _set_reg(0xdc);
				_set_reg(0x00);
				
				_set_reg(0x20);// Set Page Addressing Mode
        // _set_reg(0x7F );
        // _set_reg(0x90 );

        //Set DC/DC control mode set external vaoltage
        _set_reg(0xAD);
        _set_reg(0x8a);

        //for enabling charge pump 0x32: 8Volts
        //_set_reg(0x33);
				 _set_reg(0xa4); // Set Entire Display OFF/ON
				 _set_reg(0xa6); // Set Normal/Reverse Display 
        if (b_reverse) {
                //Segment Remap / Reverse
                _set_reg(0xA0); // A0

                //Set COM Output Scan Direction
                _set_reg(0xC0); // C0
        } else {
                //Segment Remap / Reverse
                _set_reg(0xA1);

                //Set COM Output Scan Direction
                _set_reg(0xC8);
        }
        //common pads hardware: alternative
        _set_reg(0xDA);
        _set_reg(0x12);

        //contrast control
        _set_reg(0x81);
        _set_reg(contrast);

        //set pre-charge period
        _set_reg(0xD9);
        _set_reg(0x22); //0x28(???????)

        //VCOM deselect level mode
        _set_reg(0xDB);
        _set_reg(0x35);  //set Vvcomh=0.83*Vcc 0x35(???????)

        //Set Entire Display On/Off - Normal display
        _set_reg(0xA4);

        //Set Normal Display
        _set_reg(0xA6);

        BASE_delay_msec(20);

        // Clean up frame buffer
        _set_reg(0xAF); //Set Display On
}

void init_program()
{
 _set_reg(0xae); // Display OFF
 _set_reg(0xa8); // Set Multiplex Ration
 _set_reg(0x3f);
 _set_reg(0xd5); // Set Frame Frequency
 _set_reg(0x51); // 104Hz
 _set_reg(0xc0); // Set Common scan direction
 _set_reg(0xd3); // Set Display Offset
 _set_reg(0x60);
 _set_reg(0xdc); // Set Display Start Line
 _set_reg(0x00);
 _set_reg(0x20); // Set Page Addressing Mode
 _set_reg(0x81); // Set Contrast Control
 _set_reg(0x90);
 _set_reg(0xa0); // Set Segment Re-map
 _set_reg(0xa4); // Set Entire Display OFF/ON
 _set_reg(0xa6); // Set Normal/Reverse Display
 _set_reg(0xad); // Set External VPP
 _set_reg(0x8a);
 _set_reg(0xd9); // Set Phase Leghth
 _set_reg(0x22);
 _set_reg(0xdb); // Set Vcomh voltage
 _set_reg(0x35);
 //clear_screen();
 _set_reg(0xaf); //Display ON
} 

uint8_t tmp[128*128/8];
void  oled_test()
{
	for (int t = 0; t < Y_RESOLUTION/8; t++) {
	for (int i = 0; i < X_RESOLUTION; i++) {
		tmp[t*X_RESOLUTION +i] = 0x55;//1 << (i%8);
	}
}
	
		OLED_hw_reset();
	  OLED_init(0xCF, TRUE);
//init_program();
		OLED_full_scree_show(tmp);
	
}
void OLED_set_panel_off()
{
        CLING_OLED_CTX *o = &cling.oled;

        o->state = OLED_STATE_GOING_OFF;
        cling.ui.display_active = FALSE;
        UI_task_notify_oled();

        N_SPRINTF("OLED: panel is set off");
}

BOOLEAN OLED_set_panel_on()
{
        CLING_OLED_CTX *o = &cling.oled;

        // We are about to turn on OLED, if BLE is in idle mode, we should start advertising
//	if (BTLE_is_idle()) {
//		BTLE_execute_adv(TRUE);
//	}

        if (OLED_STATE_IDLE == o->state) {
                if (o->module_state == OLED_MODULE_TURNED_OFF) {
                        o->state = OLED_STATE_APPLYING_POWER;
                        /*change adv frequncy every time when screen is actived*/
                        extern void notify_ble_to_switch_adv_frequnce();
                        notify_ble_to_switch_adv_frequnce();
                        N_SPRINTF("[OLED] Applying power on oled panel");
                        UI_task_notify_oled();

                        return TRUE;
                }
        }
        N_SPRINTF("[OLED] Panel power is on: %d, %d", o->state, o->module_state);

        return FALSE;
}

BOOLEAN OLED_is_panel_idle()
{
        CLING_OLED_CTX *o = &cling.oled;

        if (o->module_state == OLED_MODULE_TURNED_ON)
                return FALSE;

        if (o->state == OLED_STATE_IDLE) {
                return TRUE;
        }

        return FALSE;
}

extern void UI_oled_timer_callback(OS_TIMER tmr);
extern void SYSCLK_restart_timer(OS_TIMER timer, I32U period_in_ms);
extern OS_TIMER SYSCLK_get_timer(I32U period_in_ms, TimerCallbackFunction_t callback);
extern int CLNGPAY_balance_callback(uint32_t balance);

//#if 0
//static I8U oledstate;
//#endif
//// This routine implements the main display state machine.  If someone wants to write something to the
//// display, it first issues the command SSD1306_SetDisplay(
////static I8U oledstate;
int  OLED_state_machine(void)
{
        I32U t_curr   __attribute__((unused));
        CLING_OLED_CTX *o = &cling.oled;
        BOOLEAN b_notify_task = FALSE;

        Y_SPRINTF("[OLED] new oled state: %d @ %d", o->state, CLK_get_system_time());

        t_curr = CLK_get_system_time();

        switch (o->state) {
        case OLED_STATE_IDLE:
                break;
        case OLED_STATE_APPLYING_POWER:
        {
                // turn on the power pins and wait the appropriate time
                o->state = OLED_STATE_RESET_OLED;

//                pm_set_sleep_mode(pm_mode_active);

                if (!oled_timer) {
                        // Make sure alert timer module is created!
                        oled_timer = SYSCLK_get_timer(OLED_POWER_START_DELAY_TIME,
                                UI_oled_timer_callback);
                        if (!oled_timer) {
                                N_SPRINTF("Reminder: Timer Started Failed ");
                                cling.reminder.state = REMINDER_STATE_IDLE;
                                break;
                        }
                } else {
                        SYSCLK_restart_timer(oled_timer, OLED_POWER_START_DELAY_TIME);
                }
                break;
        }
        case OLED_STATE_RESET_OLED:
        {
								OLED_set_display(1); /*display off*/
                OLED_hw_reset();
                o->state = OLED_STATE_INIT_REGISTERS;
                N_SPRINTF("OLED: reset OLED module at %d", CLK_get_system_time());
                SYSCLK_restart_timer(oled_timer, OLED_RESET_INIT_LATENCY);
                break;
        }
        case OLED_STATE_INIT_REGISTERS:
        {
								
                OLED_init(0xCF, TRUE);
                o->state = OLED_STATE_INIT_UI;
                N_SPRINTF("OLED: initialize all registers at %d", CLK_get_system_time());
                SYSCLK_restart_timer(oled_timer, OLED_UI_START_LATENCY);
                break;
        }
        case OLED_STATE_INIT_UI:
        {
                // CLING screen timeout
                cling.ui.display_active = TRUE;

                // Update display timeout base.
                cling.ui.display_to_base = CLK_get_system_time();

                Y_SPRINTF("0LED: ready for display at: %d (%d)", CLK_get_system_time(), cling.ui.display_active);

                o->state = OLED_STATE_ON;
                o->module_state = OLED_MODULE_TURNED_ON;

                // Reset blinking state
                cling.ui.b_icon_blinking = TRUE;

                // register callback to balance check
//                get_balance(CLNGPAY_balance_callback);


                // If screen is turned, dismiss the secondary reminder vibration
                if ((cling.reminder.state >= REMINDER_STATE_ON)
                        && (cling.reminder.state <= REMINDER_STATE_SECOND_REMINDER)) {
                        cling.ui.notif_type = NOTIFICATION_TYPE_REMINDER;
                        UI_switch_state(UI_STATE_NOTIFICATIONS, 0);
                        cling.reminder.ui_hh = cling.time.local.hour;
                        cling.reminder.ui_mm = cling.time.local.minute;
                        cling.reminder.ui_alarm_on = TRUE; // Indicate this is a active alarm reminder
                        N_SPRINTF("[OLED] state reminder: %d, %d, %d", cling.reminder.state, cling.reminder.ui_hh, cling.reminder.ui_mm);
                        break;
                }

                if (cling.notific.state != NOTIFIC_STATE_IDLE) {
                        UI_switch_state(UI_STATE_NOTIFICATIONS, 0);
                        N_SPRINTF("[OLED] state notific: %d, %d", cling.notific.state, CLK_get_system_time());
                        break;
                }

                // Notify UI rendering state machine as OLED panel is turned on
                UI_task_notify_rendering();
                break;
        }
        case OLED_STATE_ON:
        {
                break;
        }
        case OLED_STATE_GOING_OFF:
        {
                                OLED_set_display(0); /*display off*/
                                //OLED_power_off();
//                                pm_set_sleep_mode(pm_mode_deep_sleep);
                                SYSCLK_restart_timer(oled_timer, OLED_POWER_OFF_DELAY_TIME);
                                o->state = OLED_STATE_OFF;
                break;
        }
        case OLED_STATE_OFF:
        {
                                // don't let power come back on for 100ms
                                o->state = OLED_STATE_IDLE;
                                o->module_state = OLED_MODULE_TURNED_OFF;
                                N_SPRINTF("OLED: power off at %d", CLK_get_system_time());
                break;
        }
        default:
                o->state = OLED_STATE_IDLE;
                break;
        }
        return b_notify_task;
        if (b_notify_task) {
                UI_task_notify_oled();
        }
}

