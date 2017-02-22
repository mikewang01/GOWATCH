/***************************************************************************//**
 * @file standard_macros.h
 * @brief Macro definitions for SSI projects
 *
 * @details This file defines the standard macros used by SSI projects
 *          conforming to the SSI coding standard.
 *
 * Copyright &copy; 2011 RHT as unpublished work.  All rights reserved.
 *
 * @author  Greg Lindsey
 * @date 03/14/2011
 * @version 1.3
 *
 * @note Do not include this file directly; it is included by standards.h.
 ******************************************************************************/

#ifndef __STANDARD_MACROS_H__
#define __STANDARD_MACROS_H__
/** @addtogroup STANDARDS */
/** @{ */

/* Generic macros (do not change across platforms) */
#define LOOP_FOREVER    1         /* makes while() statements more legible */
#define DO_ONCE         0         /* makes while() statements more legible */
#define ALWAYS  1                 /* makes if() statements more legible */
#define NEVER   0                 /* makes if() statements more legible */
#define TRUE  1
#define FALSE 0
#define HIGH  1
#define LOW   0
#define RISING 1
#define FALLING 0
#define ENABLED 1
#define DISABLED 0

#define BOOLEAN_TRUE  (0xFF)
#define BOOLEAN_FALSE (0)
#define BOOLEAN_NOT(x)  ((BOOLEAN_TRUE == x) ? BOOLEAN_FALSE : BOOLEAN_TRUE)

#define BIT_VALUE_0   0
#define BIT_VALUE_1   1

#define FLAG_SET      (-1)
#define FLAG_CLEAR    (0)

/* BV gets a particular bit value; adopted from TI BSP */
#ifndef BV
#define BV(n)      (1 << (n))
#endif

/* st is similar to Kari's do{}while(0) code, but even more robust.
   Adopted from TI BSP. */
#ifndef st
/* GSL: CCS complains with traditional st() macro. */
//#define st(x)      do { x } while (__LINE__ == -1)
#define st(x)        if(1) { x }
#endif

/* SET_MASK, CLEAR_MASK, SET_AND_CLEAR help with Boolean logic. */
#define   SET_MASK(port, set)           st(port |= (set);)
#define CLEAR_MASK(port, clear)         st(port &= ~(clear);)
#define SET_AND_CLEAR(port, set, clear) st(                     \
                                            I8U p = port;       \
                                            p |= (set);         \
                                            p &= ~(clear);      \
                                            port = p;           \
                                        )

/* NOWHERE ensures greater type-matching safety for ADDRESS types */
#ifndef NOWHERE
#define NOWHERE ((ADDRESS)0)
#endif

/* NULL is used for other pointer types */
#ifndef NULL
#define NULL 0
#endif

/* Sanity check macro calls a sanity check handler if __DEBUG__ is defined. 
 * Redefine to do something else (like flash an LED) if desired. 
 */
#ifdef __DEBUG__
#ifndef SANITY_CHECK_HANDLER
#define SANITY_CHECK_HANDLER()      st( DISABLE_INTERRUPTS();  while(1); )
#endif
#define SANITY_CHECK(expr)          st( if (!(expr)) SANITY_CHECK_HANDLER(); )
#define FORCE_SANITY_CHECK()        SANITY_CHECK_HANDLER()
#define SANITY_CHECKS_ARE_ON
#else
#define SANITY_CHECK(expr)          /* empty */
#define FORCE_SANITY_CHECK()        /* empty */
#endif

/* Definition of __func__ for older pre-C99 GCC versions */
#if __STDC_VERSION__ < 199901L
# if __GNUC__ >= 2
#  define __func__ __FUNCTION__
# else
#  define __func__ "<unknown>"
# endif
#endif

/* Specific macros (implementation may change across platforms) */

/* PIC18 implementations ******************************************************/
#ifdef _PIC18   /* What is the compiler define for PIC18? */
#define _MACROS_DEFINED

/** Declare that a particular function is an ISR */
#ifndef _ISR
#define _ISR __attribute__((interrupt))
#endif // _ISR

/** Tell the compiler not to optimize this function */
#define DO_NOT_OPTIMIZE     st(__asm__ volatile("nop");)

/** Do a software reset */
#define RESET()             st(__asm__ volatile("reset");)

/** Do a NOP. */
#define NOP()               st(__asm__ volatile("nop");)

/** Disable interrupts */
#define DISABLE_INTERRUPTS  st(__asm__ volatile("disi #0x3fff");)

/** Enable interrupts */
#define ENABLE_INTERRUPTS   st(__asm__ volatile("disi #0x0000");)

/** Perform an instruction with interrupts disabled.
 *  Useful for atomic operations inside ISRs. */
#define INTERRUPT_PROTECT(x) do {           \
   char saved_ipl;                          \
                                            \
   SET_AND_SAVE_CPU_IPL(saved_ipl,7);       \
   x;                                       \
   RESTORE_CPU_IPL(saved_ipl); } while (__LINE__ == -1);

#endif // _PIC18

/* MSP430 implementations *****************************************************/
#if defined(__msp430) || defined(__MSP430__)

#define _MACROS_DEFINED

/** Declare that a particular function is an ISR */
/* GSL NOTE: CCS requires a #pragma to specify the vector! */
#ifndef _ISR
#define _ISR(func, vect)      interrupt void func(void)
#endif // _ISR

/** Tell the compiler not to optimize this function */
#define DO_NOT_OPTIMIZE       // no equivalent as far as I know

/** Do a software reset.
    @warning This macro is currently MSP430-specific!
 */
#define RESET()               st(PMMCTL0_H = PMMPW_H; PMMCTL0 |= PMMSWPOR;)

/** Do a NOP. */
#define NOP()                 _no_operation()

/** Disable interrupts.
 *  NOP is inserted as per MSP430F530x errata CPU39 for single-stepping.
 **/
#define DISABLE_INTERRUPTS()    st(_disable_interrupts(); _no_operation();)

/** Enable interrupts */
#define ENABLE_INTERRUPTS     _enable_interrupts

/** Perform an instruction with interrupts disabled.
 *  Useful for atomic operations inside ISRs. */
#define INTERRUPT_PROTECT(x)  if (1) { \
  _disable_interrupts();               \
  x;                                   \
  _enable_interrupts();                \
} 

#endif // __msp430


/* TI BSP implementations *****************************************************/
/* These macros map to the appropriate macros in the SimpliciTI BSP. */
#ifdef USE_TI_BSP

#include "bsp.h" // if not already included
#define _MACROS_DEFINED

/** Declare that a particular function is an ISR */
#ifndef _ISR
#define _ISR(func, vect)      BSP_ISR_FUNCTION(func, vect)
#endif // _ISR

/** Tell the compiler not to optimize this function */
#define DO_NOT_OPTIMIZE       // no equivalent as far as I know

/** Do a software reset.
    @warning This macro is currently MSP430-specific!
 */
#define RESET()               st(PMMCTL0_H = PMMPW_H; PMMCTL0 |= PMMSWPOR;)

/** Do a NOP. */
#define NOP()                 __no_operation()

/** Disable interrupts */
#define DISABLE_INTERRUPTS    BSP_DISABLE_INTERRUPTS

/** Enable interrupts */
#define ENABLE_INTERRUPTS     BSP_ENABLE_INTERRUPTS

/** Perform an instruction with interrupts disabled.
 *  Useful for atomic operations inside ISRs. */
#define INTERRUPT_PROTECT(x)  BSP_CRITICAL_STATEMENT(x)

#endif // USE_TI_BSP


/* Null implementations (if none of the above defines are set) ****************/
#ifndef _MACROS_DEFINED

//#pragma warning "UNSAFE -- Compiler/Architecture unknown, so standard macros have been NULLed out; please see standard_macros.h."

/** Declare that a particular function is an ISR */
#ifndef _ISR
#define _ISR
#endif // _ISR

#define DO_NOT_OPTIMIZE
#define RESET()
#define NOP()
#define DISABLE_INTERRUPTS
#define ENABLE_INTERRUPTS
#define INTERRUPT_PROTECT(x) do { x; } while (0)

#endif // _MACROS_DEFINED

/** @} */
#endif /* __STANDARD_MACROS_H__ */
