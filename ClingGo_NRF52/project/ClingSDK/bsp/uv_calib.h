
#ifndef __uv_CALIB_H__
#define __uv_CALIB_H__

#include "standards.h"

//#define _ENABLE_UV_CALIBRATION_

/***************************************************************************//**
 * @brief
 * This compile switch used only to experiment with 
 * various rounding precisions. The flexibility has
 * a small performance price. 
 ******************************************************************************/
#define     FORCE_ROUND_16


typedef struct 
{
    I32U    vispd_correction;  /**< VIS Photodiode Correction        */ 
    I32U    irpd_correction;   /**< IR  Photodiode Correction        */
    I32U    adcrange_ratio;    /**< _RANGE Ratio                     */
    I32U    irsize_ratio;      /**< Large IR vs Small IR Ratio       */
    I32U    ledi_ratio;        /**< LED Drive Current Correction     */
    I8U*    ucoef_p;           /**< Pointer to UV Coefficients       */
} uv_CAL_S;

typedef struct
{
  I32U op1;                      /**< Operand 1 */ 
  I32U op2;                      /**< Operand 2 */
} operand_t;

typedef struct
{
  I32U     sirpd_adchi_irled;    /**< Small IR PD gain, IR LED, Hi ADC Range     */
  I32U     sirpd_adclo_irled;    /**< Small IR PD gain, IR LED, Lo ADC Range     */
  I32U     sirpd_adclo_whled;    /**< Small IR PD gain, White LED, Lo ADC Range  */
  I32U     vispd_adchi_whled;    /**< VIS PD gain, White LED, Lo ADC Range       */
  I32U     vispd_adclo_whled;    /**< VIS PD gain, White LED, Lo ADC Range       */
  I32U     lirpd_adchi_irled;    /**< Large IR PD gain, IR LED, Hi ADC Range     */
  I32U     ledi_65ma;            /**< LED Current Ratio at 65 mA                 */
  I8U      ucoef[4];             /**< UV Coefficient Storage                     */
} cal_ref_t;


I16S uv_Get_Cal_Index( I8U* buffer );
I16S uv_Get_Calibration(I8U security);

#endif
