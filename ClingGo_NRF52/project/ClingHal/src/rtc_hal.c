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
#include "rtc_hal.h"
#include "osal.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
/*********************************************************************
* MACROS
*/
#ifdef __cplusplus
extern "C" {
#endif


#define RTC_PERIOD 2000  //MS - 2 Second RTC timer
                        // Looks like this RTC clock affects acceleromter evetns

/*********************************************************************
* TYPEDEFS
*/


/*********************************************************************
* GLOBAL VARIABLES
*/

/*********************************************************************
* LOCAL VARIABLES
*/
static CLASS(HalRtc)* p_instance = NULL;
static OS_TIMER rtc_timer;
/*********************************************************************
* LOCAL DECLARATION
*/
CLASS(HalRtc)* HalRtc_get_instance(void);
static int reg_isr(CLASS(HalRtc) *arg, void (*isr)(void));
static void isr_routine(OS_TIMER t);
static int HalRtc_init(CLASS(HalRtc) *arg);
static int start(CLASS(HalRtc) *arg);
/******************************************************************************
* FunctionName :HalRtc_get_instance
* Description  : get hal rtc instance
* Parameters   : write: write call back function
* Returns          : 0: sucess
                     -1: error
*******************************************************************************/
CLASS(HalRtc)* HalRtc_get_instance(void)
{
        static CLASS(HalRtc) p;
        if (p_instance == NULL) {
                p_instance = &p;
                memset(&p, 0, sizeof(p));
                HalRtc_init(p_instance);
        }
        return p_instance;
}

/******************************************************************************
* FunctionName :HalRtc_init
* Description  : init hal rtc instance
* Parameters   : object pointer
* Returns          : 0: sucess
                     -1: error
*******************************************************************************/
static int HalRtc_init(CLASS(HalRtc) *arg)
{
        arg->isr_reg = reg_isr;
        arg->start = start;
        arg->stop = NULL;
        rtc_timer = NULL;
        rtc_timer = OS_TIMER_CREATE("rtc_timer", OS_MS_2_TICKS(RTC_PERIOD), pdTRUE, (void*)0, isr_routine);
        if(rtc_timer == NULL){
                Y_SPRINTF("[HAL_TIMER] Rtc Timer Created Failed ");
                return -1;
        }
        return 0;
}
int rtc_init()
{
        rtc_timer = NULL;
        rtc_timer = OS_TIMER_CREATE("rtc_timer", OS_MS_2_TICKS(RTC_PERIOD), pdTRUE, (void*)0, isr_routine);
        if(rtc_timer == NULL){
                Y_SPRINTF("[HAL_TIMER] Rtc Timer Created Failed ");
                return -1;
        }
        int status = OS_TIMER_START(rtc_timer, 0);
        if(status != OS_TIMER_SUCCESS){
                Y_SPRINTF("[HAL_TIMER] Rtc Timer Started Failed ");
                OS_TIMER_DELETE(rtc_timer,0);
                rtc_timer = NULL;
                return -1;
        }

        return 0;
//        arg->isr_reg;
}
/******************************************************************************
* FunctionName : reg_isr
* Description  : register rtc callback
* Parameters   : write: write call back function
* Returns          : 0: sucess
                     -1: error
*******************************************************************************/

static void (*rtc_isr_call_back)(void) = NULL;
static int reg_isr(CLASS(HalRtc) *arg, void (*isr)(void))
{
        if(isr != NULL){
                rtc_isr_call_back = isr;
        }
        return 0;
}

static int start(CLASS(HalRtc) *arg)
{
        if(rtc_timer == NULL)
        {
                return -1;
        }
        int status = OS_TIMER_START(rtc_timer, 0);
        if(status != OS_TIMER_SUCCESS){
                Y_SPRINTF("[HAL_TIMER] Rtc Timer Started Failed ");
                OS_TIMER_DELETE(rtc_timer,0);
                rtc_timer = NULL;
                return -1;
        }
        return 0;
}
/******************************************************************************
* FunctionName :isr_routine
* Description  : get hal rtc instance
* Parameters   : write: write call back function
* Returns          : 0: sucess
                     -1: error
*******************************************************************************/
static void isr_routine(OS_TIMER t)
{
        N_SPRINTF("[HAL_TIMER] Rtc Isr Routine Entered \r\n");
        if(rtc_isr_call_back != NULL){
                rtc_isr_call_back ();
         }
        return ;
}



#ifdef __cplusplus
}
#endif
