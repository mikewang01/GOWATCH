/***************************************************************************//**
 * @file lis3dh.h
 * @brief Driver interface for ST lis3dh accelerometer.
 *
 ******************************************************************************/

#ifndef __LIS3DH_H__
#define __LIS3DH_H__

#include "standards.h"

#ifdef _SLEEP_SIMULATION_
extern int LIS3DH_is_awake;
#endif

#define ODR1                                    0x10  /* 1Hz output data rate */
#define ODR10                                   0x20  /* 10Hz output data rate */
#define ODR25                                   0x30  /* 25Hz output data rate */
#define ODR50                                   0x40  /* 50Hz output data rate */
#define ODR100                                  0x50  /* 100Hz output data rate */
#define ODR200                                  0x60  /* 200Hz output data rate */
#define ODR400                                  0x70  /* 400Hz output data rate */
#define ODR1250                                 0x90  /* 1250Hz output data rate */
 

 
#define SENSITIVITY_2G                  1       /**     mg/LSB  */
#define SENSITIVITY_4G                  2       /**     mg/LSB  */
#define SENSITIVITY_8G                  4       /**     mg/LSB  */
#define SENSITIVITY_16G                 12      /**     mg/LSB  */
 
#define HIGH_RESOLUTION                 0x08
 
/* Accelerometer Sensor Full Scale */
 
#define LIS3DH_ACC_FS_MASK              0x30
 #define LIS3DH_ACC_G_2G                 0x00
 #define LIS3DH_ACC_G_4G                 0x10
 #define LIS3DH_ACC_G_8G                 0x20
 #define LIS3DH_ACC_G_16G                0x30
 
 
#define WHO_AM_I                          0x0F
#define WHOAMI_LIS3DH_ACC               0x33
#define AXISDATA_REG                    0x28
 

 
#define I2C_AUTO_INCREMENT              0x80
#define I2C_RETRY_DELAY                 5
#define I2C_RETRIES                             5
 

 
#define RESUME_ENTRIES                  17
 

 
#define RES_CTRL_REG1                   0
#define RES_CTRL_REG2                   1
#define RES_CTRL_REG3                   2
#define RES_CTRL_REG4                   3
#define RES_CTRL_REG5                   4
#define RES_CTRL_REG6                   5
#define RES_INT_CFG1                    6
#define RES_INT_THS1                    7
#define RES_INT_DUR1                    8
#define RES_TT_CFG                              9
#define RES_TT_THS                              10
#define RES_TT_LIM                              11
#define RES_TT_TLAT                             12
#define RES_TT_TW                               13
 
#define TT_CFG                                  0x38    /*      tap config              */
#define TT_SRC                                  0x39    /*      tap source              */
#define TT_THS                                  0x3A    /*      tap threshold           */
#define TT_LIM                                  0x3B    /*      tap time limit          */
#define TT_TLAT                                 0x3C    /*      tap time latency        */
#define TT_TW                                   0x3D    /*      tap time window */

#define RES_TEMP_CFG_REG                14
#define RES_REFERENCE_REG               15
#define RES_FIFO_CTRL_REG               16

#define CTRL_REG1                               0x20    /*      control reg 1           */
#define CTRL_REG2                               0x21    /*      control reg 2           */
#define CTRL_REG3                               0x22    /*      control reg 3           */
#define CTRL_REG4                               0x23    /*      control reg 4           */
#define CTRL_REG5                               0x24    /*      control reg 5           */
#define CTRL_REG6                               0x25    /*      control reg 6           */

#define REF_DATACAPTURE									0x26

#define STATUS_REG                      0x27

#define TEMP_CFG_REG                    0x1F    /*      temper sens control reg */

#define FIFO_CTRL_REG                   0x2E    /*      FiFo control reg        */
#define FIFO_SRC_REG                    0x2F    /* FIFO resources reg */
#define INT_CFG1                                0x30    /*      interrupt 1 config      */
#define INT_SRC1                                0x31    /*      interrupt 1 source      */
#define INT_THS1                                0x32    /*      interrupt 1 threshold   */
#define INT_DUR1                                0x33    /*      interrupt 1 duration    */

//
// Tap detection
//
#define TAP_CFG           0x38
#define TAP_SRC           0x39
#define TAP_THS           0x3A
#define TAP_TIME_LIMIT    0x3B
#define TAP_TIME_LATENCY  0x3C
#define TAP_TIME_WINDOW   0x3D

 
/* Default register settings */
#define RBUFF_SIZE                              12      /* Rx buffer size */
 
#define STIO                                    0xA1
 
 
/*rate*/
 
#define LIS3DH_RATE_800                 0
#define LIS3DH_RATE_400                 1
#define LIS3DH_RATE_200                 2
#define LIS3DH_RATE_100                 3
#define LIS3DH_RATE_50                  4
#define LIS3DH_RATE_12P5                5
#define LIS3DH_RATE_6P25                6
#define LIS3DH_RATE_1P56                7
#define LIS3DH_RATE_SHIFT               3
#define LIS3DH_ASLP_RATE_50             0
#define LIS3DH_ASLP_RATE_12P5   1
#define LIS3DH_ASLP_RATE_6P25   2
#define LIS3DH_ASLP_RATE_1P56   3
#define LIS3DH_ASLP_RATE_SHIFT  6
 
#define ACTIVE_MASK                             1
#define FREAD_MASK                              2
 

 
/*status*/
 
#define LIS3DH_SUSPEND                  2
#define LIS3DH_OPEN                             1
#define LIS3DH_CLOSE                    0
#define LIS3DH_SPEED                    200 * 1000
 
#define LIS3DH_ACC_ENABLE_ALL_AXES      0x07

#define LIS3DH_RANGE                    2000000
#define LIS3DH_PRECISION                16 //8bit data
#define LIS3DH_BOUNDARY                 (0x1 << (LIS3DH_PRECISION - 1))
#define LIS3DH_GRAVITY_STEP             LIS3DH_RANGE / LIS3DH_BOUNDARY  //2g full scale range
 
typedef struct tagACC_AXIS {
		I16S x; 
    I16S y;
    I16S z;
} ACC_AXIS;

EN_STATUSCODE LIS3DH_init(void);
EN_STATUSCODE LIS3DH_normal_FIFO(void);
BOOLEAN LIS3DH_is_data_ready(ACC_AXIS *xyz);
I8U LIS3DH_who_am_i(void);
I8U LIS3DH_get_interrupt(void);
I8U LIS3DH_get_tap(void);
EN_STATUSCODE LIS3DH_inertial_wake_up_init(void);
I8U LIS3DH_is_FIFO_ready(void);
void LIS3DH_retrieve_data(ACC_AXIS *xyz);

#endif // __MMA845XQ_H__
/** @} */
