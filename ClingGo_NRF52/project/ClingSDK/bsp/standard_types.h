/***************************************************************************//**
 * @file standard_types.h
 * @brief Type definitions for RHT projects
 *
 * @details This file defines the standard types used by RHT projects conforming
 *          to the RHT coding standard.
 *
 * Copyright &copy; 2011 RHT as unpublished work.  All rights reserved.
 *
 * @author  Greg Lindsey
 * @date 03/08/2011
 * @version 1.2
 *
 * @note Do not include this file directly; it is included by standards.h.
 ******************************************************************************/

#ifndef __STANDARD_TYPES_H__
#define __STANDARD_TYPES_H__
/** @addtogroup STANDARDS */
/** @{ */

/* Minimum and maximum values for types (do not change across platforms) */
#define I8U_MIN     ((I8U)0)
#define I8U_MAX     ((I8U)0xff)
#define I16U_MIN    ((I16U)0)
#define I16U_MAX    ((I16U)0xffff)
#define I24U_MIN    ((I24U)0)
#define I24U_MAX    ((I24U)0xffffff)
#define I32U_MIN    ((I32U)0)
#define I32U_MAX    ((I32U)0xffffffff)
#define I64U_MIN    ((I64U)0)
#define I64U_MAX    ((I64U)0xffffffffffffffff)
#define I8S_MIN     ((I8S)-128)
#define I8S_MAX     ((I8S)127)
#define I16S_MIN    ((I16S)-32768)
#define I16S_MAX    ((I16S)32767)
#define I24S_MIN    ((I24S)-8388608)
#define I24S_MAX    ((I24S)8388607)
#define I32S_MIN    ((I32S)-2147483648)
#define I32S_MAX    ((I32S)2147483647)
#define I64S_MIN    ((I64S)-9223372036854775808)
#define I64S_MAX    ((I64S)9223372036854775807)

/* Type definitions (may change across platforms) */
/* Note: Do not use int in typedefs whenever possible, as it is the most likely 
         of all of the base types to change sizes.  Stick with char, short and 
         long, as these are "supposed" to be defined as 8, 16, and 32 bits 
         respectively.              
 */

typedef void                        VOID;       /**< void */
typedef signed char                 I8S;        /**<  8-Bit Signed Integer */
typedef signed short int            I16S;       /**< 16-Bit Signed Integer */
typedef signed      int             I32S;       /**< 32-Bit Signed Integer */
typedef unsigned char               I8U;        /**<  8-Bit Unsigned Integer */
typedef unsigned short int          I16U;       /**< 16-Bit Unsigned Integer */
typedef unsigned      int           I32U;       /**< 32-Bit Unsigned Integer */
typedef float                       FP32S;      /**< 32-Bit Signed Floating Point Number */

#ifdef UNICODE
typedef I16S                        CHARS;      /**<  16-Bit Signed Unicode Character */
typedef I16U                        CHARU;      /**<  16-Bit Unsigned Unicode Character */
#else
typedef I8S                         CHARS;      /**<  8-Bit Signed Character */
typedef I8U                         CHARU;      /**<  8-Bit Unsigned Character */
#endif

/* Semantic types */
typedef I32U *                      ADDRESS;    /**<  Memory address */
typedef I8U                         BOOLEAN;    /**<  Boolean value */
typedef I8U                         BYTE;       /**<  Data byte (arbitrary) */
typedef I8S                         FLAG;       /**<  Flag (set/clear) */

typedef void    (*VOIDFUNCPTR) (void);          /**<  Pointer to void(void) function */
typedef int     (*INTFUNCPTR)  (void);          /**<  Pointer to a int (void) function */
typedef BOOLEAN (*BOOLFUNCPTR) (void);          /**<  Pointer to a BOOLEAN (void) function */

/* The following types are NOT supported by this compiler */
//typedef long long int               I64S;
//typedef unsigned long long int      I64U;
//typedef short long              I24S;     /* 24-Bit Signed Integer: Microchip C18 compiler */
//typedef unsigned short long     I24U;     /* 24-Bit Unsigned Integer: Microchip C18 compiler */
//typedef double                                FP64S;


/* Bitfield types */
typedef I32U BITS32;
typedef I16U BITS16;
typedef I8U  BITS8;
#ifndef SET_BIT
#define SET_BIT(bitfield, bitnumber) do { bitfield |= (1 << bitnumber); } while (0)
#endif
#define CLEAR_BIT(bitfield, bitnumber) do { bitfield = ~(~bitfield | (1 << bitnumber)); } while (0)
#define CHECK_BIT(bitfield, bitnumber) ( 0 != (bitfield & (1 << bitnumber) ) )

/* "Sanity-check" code  (does not change across platforms) */
/* This code will cause the preprocessor to fail if any of these typedefs */
/* are not defined correctly, by causing an invalid array definition.  It */
/* was gleaned from Wikipedia and adapted to our typenames by Greg.       */
#ifdef DEBUG_SIZE_CHECK
static union
{
        char   I8S_incorrect       [sizeof(I8S) == 1];
        char   I8U_incorrect       [sizeof(I8U) == 1];
        char   I16S_incorrect      [sizeof(I16S) == 2];
        char   I16U_incorrect      [sizeof(I16U) == 2];
#ifdef I24S
        char   I24S_incorrect      [sizeof(I24S) == 3];
#endif
#ifdef I24U
        char   I24U_incorrect      [sizeof(I24U) == 3];
#endif        
        char   I32S_incorrect      [sizeof(I32S) == 4];
        char   I32U_incorrect      [sizeof(I32U) == 4];
#ifdef I64S        
        char   I64S_incorrect      [sizeof(I64S) == 8];
#endif
#ifdef I64U
        char   I64U_incorrect      [sizeof(I64U) == 8];
#endif
}
_DUMMY_SANITY_CHECK;
#endif

typedef union _BYTE_VAL
{
    I8U Val;
    struct
    {
        I8U b0:1;
        I8U b1:1;
        I8U b2:1;
        I8U b3:1;
        I8U b4:1;
        I8U b5:1;
        I8U b6:1;
        I8U b7:1;
    } bits;
} BYTE_VAL;

typedef union _CHAR_VAL
{
    I8S Val;
    struct
    {
        I8U b0:1;
        I8U b1:1;
        I8U b2:1;
        I8U b3:1;
        I8U b4:1;
        I8U b5:1;
        I8U b6:1;
        I8U b7:1;
    } bits;
} CHAR_VAL;

typedef union _WORD_VAL
{
    I16U Val;
    I8U v[2];
    struct
    {
        I8U LB;
        I8U HB;
    } byte;
    struct
    {
        I8U b0:1;
        I8U b1:1;
        I8U b2:1;
        I8U b3:1;
        I8U b4:1;
        I8U b5:1;
        I8U b6:1;
        I8U b7:1;
        I8U b8:1;
        I8U b9:1;
        I8U b10:1;
        I8U b11:1;
        I8U b12:1;
        I8U b13:1;
        I8U b14:1;
        I8U b15:1;
    } bits;
} WORD_VAL;

typedef union _SHORT_VAL
{
    I16S Val;
    I8U  v[2];
    struct
    {
        I8U LB;
        I8U HB;
    } byte;
    struct
    {
        I8U b0:1;
        I8U b1:1;
        I8U b2:1;
        I8U b3:1;
        I8U b4:1;
        I8U b5:1;
        I8U b6:1;
        I8U b7:1;
        I8U b8:1;
        I8U b9:1;
        I8U b10:1;
        I8U b11:1;
        I8U b12:1;
        I8U b13:1;
        I8U b14:1;
        I8U b15:1;
    } bits;
} SHORT_VAL;

typedef union _DWORD_VAL
{
    I32U Val;
    I16U w[2];
    I8U v[4];
    struct
    {
        I16U LW;
        I16U HW;
    } word;
    struct
    {
        I8U LB;
        I8U HB;
        I8U UB;
        I8U MB;
    } byte;
    struct
    {
        I8U b0:1;
        I8U b1:1;
        I8U b2:1;
        I8U b3:1;
        I8U b4:1;
        I8U b5:1;
        I8U b6:1;
        I8U b7:1;
        I8U b8:1;
        I8U b9:1;
        I8U b10:1;
        I8U b11:1;
        I8U b12:1;
        I8U b13:1;
        I8U b14:1;
        I8U b15:1;
        I8U b16:1;
        I8U b17:1;
        I8U b18:1;
        I8U b19:1;
        I8U b20:1;
        I8U b21:1;
        I8U b22:1;
        I8U b23:1;
        I8U b24:1;
        I8U b25:1;
        I8U b26:1;
        I8U b27:1;
        I8U b28:1;
        I8U b29:1;
        I8U b30:1;
        I8U b31:1;
    } bits;
} DWORD_VAL;

typedef union _LONG_VAL
{
    I32S Val;
    I16U w[2];
    I8U v[4];
    struct
    {
        I16U LW;
        I16U HW;
    } word;
    struct
    {
        I8U LB;
        I8U HB;
        I8U UB;
        I8U MB;
    } byte;
    struct
    {
        I8U b0:1;
        I8U b1:1;
        I8U b2:1;
        I8U b3:1;
        I8U b4:1;
        I8U b5:1;
        I8U b6:1;
        I8U b7:1;
        I8U b8:1;
        I8U b9:1;
        I8U b10:1;
        I8U b11:1;
        I8U b12:1;
        I8U b13:1;
        I8U b14:1;
        I8U b15:1;
        I8U b16:1;
        I8U b17:1;
        I8U b18:1;
        I8U b19:1;
        I8U b20:1;
        I8U b21:1;
        I8U b22:1;
        I8U b23:1;
        I8U b24:1;
        I8U b25:1;
        I8U b26:1;
        I8U b27:1;
        I8U b28:1;
        I8U b29:1;
        I8U b30:1;
        I8U b31:1;
    } bits;
} LONG_VAL;

typedef union _QWORD_VAL
{
#ifdef I64U
    I64U Val;
#endif
    I32U d[2];
    I16U w[4];
    I8U v[8];
    struct
    {
        I32U LD;
        I32U HD;
    } dword;
    struct
    {
        I16U LW;
        I16U HW;
        I16U UW;
        I16U MW;
    } word;
    struct
    {
        I8U b0:1;
        I8U b1:1;
        I8U b2:1;
        I8U b3:1;
        I8U b4:1;
        I8U b5:1;
        I8U b6:1;
        I8U b7:1;
        I8U b8:1;
        I8U b9:1;
        I8U b10:1;
        I8U b11:1;
        I8U b12:1;
        I8U b13:1;
        I8U b14:1;
        I8U b15:1;
        I8U b16:1;
        I8U b17:1;
        I8U b18:1;
        I8U b19:1;
        I8U b20:1;
        I8U b21:1;
        I8U b22:1;
        I8U b23:1;
        I8U b24:1;
        I8U b25:1;
        I8U b26:1;
        I8U b27:1;
        I8U b28:1;
        I8U b29:1;
        I8U b30:1;
        I8U b31:1;
        I8U b32:1;
        I8U b33:1;
        I8U b34:1;
        I8U b35:1;
        I8U b36:1;
        I8U b37:1;
        I8U b38:1;
        I8U b39:1;
        I8U b40:1;
        I8U b41:1;
        I8U b42:1;
        I8U b43:1;
        I8U b44:1;
        I8U b45:1;
        I8U b46:1;
        I8U b47:1;
        I8U b48:1;
        I8U b49:1;
        I8U b50:1;
        I8U b51:1;
        I8U b52:1;
        I8U b53:1;
        I8U b54:1;
        I8U b55:1;
        I8U b56:1;
        I8U b57:1;
        I8U b58:1;
        I8U b59:1;
        I8U b60:1;
        I8U b61:1;
        I8U b62:1;
        I8U b63:1;
    } bits;
} QWORD_VAL;
/** @} */
#endif /* __STANDARD_TYPES_H__ */
