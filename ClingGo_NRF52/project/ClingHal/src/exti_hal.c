/******************************************************************************
* Copyright 2013-2014 hicling Systems (MikeWang)
*
* FileName: rtc_hal.c
*
* Description: abstract layer between cling sdk and freertos.
*
* Modification history:
*     2016/7/22, v1.0 create this file mike.
*******************************************************************************/


#include "oop_hal.h"
//#include "ad_flash.h"
#include "rtc_hal.h"
#include "osal.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "exti_hal.h"
#include "nrf_drv_gpiote.h"
/*********************************************************************
* MACROS
*/
#ifdef __cplusplus
extern "C" {
#endif

#define EVENTS_INT_TRIGGER_THREAHOLD  1
#define DEBOUNCE_TIME                 0//(ms)


/*********************************************************************
* TYPEDEFS
*/
struct exti_pin_mgr{
     PinName pin;
     bool    enable;
		 uint8_t pull;
     uint8_t trigger_level;
};



/*********************************************************************
* GLOBAL VARIABLES
*/

/*********************************************************************
* LOCAL VARIABLES
*/
static CLASS(HalExti)* p_instance = NULL;


/*********************************************************************
* LOCAL DECLARATION
*/
static const struct exti_pin_mgr wu_pins[]={
        {.pin = ACC_SENSOR_INT, .enable = false, .pull = GPIO_PIN_CNF_PULL_Disabled, .trigger_level = NRF_GPIOTE_POLARITY_HITOLO},/*cpresponded interrupt is disabled default which means  you gonna need call enbale function independly*/
        {.pin = GPIO_TOUCH_INT, .enable = false, .pull = GPIO_PIN_CNF_PULL_Pullup, .trigger_level = NRF_GPIOTE_POLARITY_HITOLO},
				{.pin = GPIO_CHG_IRQ, .enable = false, .pull = GPIO_PIN_CNF_PULL_Pullup, .trigger_level = NRF_GPIOTE_POLARITY_HITOLO},
};

static void (*pin_int_callback)(PinName pin) = NULL;
CLASS(HalExti)* HalExti_get_instance(void);
static int reg_isr(CLASS(HalExti) *arg, void (*cb)(PinName));
static int HalExti_init(CLASS(HalExti) *arg);
static int disable_all(CLASS(HalExti) *arg);
static int enable_all(CLASS(HalExti) *arg);

static void hw_exti_init(void);
static void check_triggered_pins(void);
static void power_mode_interrupt_cb(void);

static void wkup_int1_z32_cb(void);
static void wkup_int1_touch_cb(void);
static void wkup_int1_acc_cb(void);




/*
 * Local functions definitions
 */
static void wkup_int1_acc_cb(void)
{
        if(pin_int_callback != NULL){
                pin_int_callback(ACC_SENSOR_INT);
        }
}

/*
 * Local functions definitions
 */
static void wkup_int1_touch_cb(void)
{
        if(pin_int_callback != NULL){
                pin_int_callback(GPIO_TOUCH_INT);
        }
}

/*
 * Local functions definitions
 */
static void wkup_int1_z32_cb(void)
{
        if(pin_int_callback != NULL){
                
        }
}

/******************************************************************************
* FunctionName :HalExti_get_instance
* Description  : get hal rtc instance
* Parameters   : write: write call back function
* Returns          : 0: sucess
                     -1: error
*******************************************************************************/
CLASS(HalExti)* HalExti_get_instance(void)
{
        static CLASS(HalExti) p;
        if (p_instance == NULL) {
                p_instance = &p;
                memset(&p, 0, sizeof(p));
                HalExti_init(p_instance);
        }
        return p_instance;
}

/******************************************************************************
* FunctionName :HalExti_init
* Description  : init hal rtc instance
* Parameters   : object pointer
* Returns          : 0: sucess
                     -1: error
*******************************************************************************/
static int HalExti_init(CLASS(HalExti) *arg)
{
        arg->isr_reg = reg_isr;
        arg->enable_all = enable_all;
        arg->disable_all = disable_all;
        nrf_drv_gpiote_init();
				hw_exti_init();
        return 0;
}


void nrf_drv_gpiote_callback(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	 if(pin_int_callback != NULL){
						pin_int_callback(pin);
		}
}

static void hw_exti_init()
{
         nrf_drv_gpiote_in_config_t  p_config ;
 
        for(int i= 0; i< sizeof(wu_pins)/sizeof(wu_pins[0]); i++){
					p_config.hi_accuracy = true;
					p_config.is_watcher = false;
					p_config.pull = wu_pins[i].pull;
					p_config.sense = wu_pins[i].trigger_level;
					int ret_code =  nrf_drv_gpiote_in_init(wu_pins[i].pin, &p_config, nrf_drv_gpiote_callback);
        }


}


/******************************************************************************
* FunctionName : reg_isr
* Description  : register rtc callback
* Parameters   : write: write call back function
* Returns          : 0: sucess
                     -1: error
*******************************************************************************/

static int reg_isr(CLASS(HalExti) *arg, void (*cb)(PinName))
{
        if(cb != NULL){
                pin_int_callback = cb;
        }
        return 1;
}

static int enable_all(CLASS(HalExti) *arg)
{
	for(int i= 0; i< sizeof(wu_pins)/sizeof(wu_pins[0]); i++){
		 nrf_drv_gpiote_in_event_enable(wu_pins[i].pin, true);
	}

        return 0;
}

static int disable_all(CLASS(HalExti) *arg)
{
	for(int i= 0; i< sizeof(wu_pins)/sizeof(wu_pins[0]); i++){
		 nrf_drv_gpiote_in_event_disable(wu_pins[i].pin);
	}
        return 0;
}


/******************************************************************************
* FunctionName :power_mode_interrupt_cb
* Description  : int calback from hw ;ayer
* Parameters   :void
* Returns          : 0: sucess
                     -1: error
*******************************************************************************/
static void power_mode_interrupt_cb(void)
{

}


#ifdef __cplusplus
}
#endif
