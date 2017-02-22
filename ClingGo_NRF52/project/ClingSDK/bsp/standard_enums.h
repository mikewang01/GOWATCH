/***************************************************************************//**
 * @file standard_enums.h
 * @brief Standard enum definitions for the framework.
 *
 * @details This file defines the standard enums used by RHT projects.  
 *          Each enum ends with a <c>_MEMBERCOUNT</c> entry;
 *          this allows easy creation of <c>for</c> loops and the like.
 *
 * @note As with standard_enums.h and project.h, this may be better being split
 *       into more specific files.  For now, however, I'm sticking all the enums
 *       here, and we can make the decision to drill down a bit later.
 *
 * Copyright &copy; 2011 RHT as unpublished work.  All rights reserved.
 *
 * @author  Greg Lindsey
 * @date 10/05/2009
 * @version 1.0
 *
 * @note Do not include this file directly; it is included by standards.h.
 ******************************************************************************/

#ifndef __STANDARD_ENUMS_H__
#define __STANDARD_ENUMS_H__
/** @addtogroup STANDARDS */
/** @{ */

/** Enumeration of possible pin signal directions. **/
typedef enum
{
  STANDARD_PINDIR_INPUT,
  STANDARD_PINDIR_OUTPUT,
  STANDARD_PINDIR_MEMBERCOUNT
} EN_STANDARD_PINDIR;

/** Enumeration of possible input pin mode configurations. **/
typedef enum
{
  STANDARD_INPUTMODE_PULLUP,
  STANDARD_INPUTMODE_FLOATING,
  STANDARD_INPUTMODE_PULLDOWN,
  STANDARD_INPUTMODE_MEMBERCOUNT
} EN_STANDARD_INPUTMODE;

/** Enumeration of possible output pin mode configurations. **/
typedef enum
{
  STANDARD_OUTPUTMODE_PUSHPULL,
  STANDARD_OUTPUTMODE_OPENDRAIN,
  STANDARD_OUTPUTMODE_MEMBERCOUNT
} EN_STANDARD_OUTPUTMODE;

/** Enumeration of possible input trigger modes. **/
typedef enum
{
  STANDARD_INTERRUPTMODE_NONE,
  STANDARD_INTERRUPTMODE_EDGE_RISING,
  STANDARD_INTERRUPTMODE_EDGE_FALLING,
  STANDARD_INTERRUPTMODE_EDGE_BOTH,
  STANDARD_INTERRUPTMODE_LEVEL_LOW,
  STANDARD_INTERRUPTMODE_LEVEL_HIGH,
  STANDARD_INTERRUPTMODE_LEVEL_BOTH,
  STANDARD_INTERRUPTMODE_MEMBERCOUNT
} EN_STANDARD_INTERRUPTMODE;

/** Enumeration of possible peripheral ports per processor. **/
typedef enum
{
  STANDARD_PORT_NULL = 0x00,
  STANDARD_PORT_A,
  STANDARD_PORT_B,
  STANDARD_PORT_C,
  STANDARD_PORT_D,
  STANDARD_PORT_E,
  STANDARD_PORT_F,
  STANDARD_PORT_G,
  STANDARD_PORT_H,
  STANDARD_PORT_I,
  STANDARD_PORT_J,
  STANDARD_PORT_MEMBERCOUNT
} EN_STANDARD_PORT;

/** Enumeration of SI prefixes. 
    This enum does not contain a MEMBERCOUNT entry since each entry is
    explicitly defined.
 **/
typedef enum
{
  STANDARD_MAGNITUDE_YOTTA = 24,
  STANDARD_MAGNITUDE_ZETTA = 21,
  STANDARD_MAGNITUDE_EXA = 18,
  STANDARD_MAGNITUDE_PETA = 15,
  STANDARD_MAGNITUDE_TERA = 12,
  STANDARD_MAGNITUDE_GIGA = 9,
  STANDARD_MAGNITUDE_MEGA = 6,
  STANDARD_MAGNITUDE_KILO = 3,
  STANDARD_MAGNITUDE_HECTO = 2,
  STANDARD_MAGNITUDE_DECA = 1,
  STANDARD_MAGNITUDE_UNIT = 0,
  STANDARD_MAGNITUDE_DECI = -1,
  STANDARD_MAGNITUDE_CENTI = -2,
  STANDARD_MAGNITUDE_MILLI = -3,
  STANDARD_MAGNITUDE_MICRO = -6,
  STANDARD_MAGNITUDE_NANO = -9,
  STANDARD_MAGNITUDE_PICO = -12,
  STANDARD_MAGNITUDE_FEMTO = -15,
  STANDARD_MAGNITUDE_ATTO = -18,
  STANDARD_MAGNITUDE_ZEPTO = -21,
  STANDARD_MAGNITUDE_YOCTO = -24
} EN_STANDARD_MAGNITUDE;

/** @} */
#endif /* __STANDARD_ENUMS_H__ */
