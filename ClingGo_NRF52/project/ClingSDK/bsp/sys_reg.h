/***************************************************************************//**
 * 
 * File: sys_reg.h
 *
 * Description: system register definition.
 *
 *
 ******************************************************************************/

/** @{ */
#ifndef __SYSTEM_REGISTER_HEADER__
#define __SYSTEM_REGISTER_HEADER__

enum {
	REGISTER_MCU_CTL = 0,
	REGISTER_MCU_CLEAR,
	REGISTER_MCU_REVH,
	REGISTER_MCU_REVL,
	REGISTER_MCU_HWINFO,
	REGISTER_MCU_HW_REV,
	REGISTER_MCU_BATTERY,
	REGISTER_MCU_MUTEX,
	REGISTER_MCU_PROTECT,
	SYSTEM_REGISTER_MAX
};

// Command definitions
#define COMMAND_MASK                      (I8U)b11100000
#define COMMAND_RETURN_FIFO_DATA          (I8U)b00000000
#define COMMAND_CONTINUATION              (I8U)b00100000
#define COMMAND_READ_MCU_REGISTER         (I8U)b01000000
#define COMMAND_READ_I2C_REGISTER         (I8U)b01100000
#define COMMAND_RESERVED                  (I8U)b10000000
#define COMMAND_CONTINUATION2             (I8U)b10100000
#define COMMAND_WRITE_MCU_REGISTER        (I8U)b11000000
#define COMMAND_WRITE_I2C_REGISTER        (I8U)b11100000

#define EXTENDED_MASK                     (I8U)b00011000
#define EXTENDED_DATA_NOT_READY           (I8U)b00000000
#define EXTENDED_SELF_TEST_DATA_RETURN    (I8U)b00000100
#define EXTENDED_CLOCK_DATA1              (I8U)b00001000
#define EXTENDED_CLOCK_DATA2              (I8U)b00010000
#define EXTENDED_ERRSTAT                  (I8U)b00011000
#define EXTENDED_STATUS                   (I8U)b00011100
#define EXTENDED_SPURIOUS_INT             (I8U)b00000110
#define EXTENDED_STATUS_PED               (I8U)b00011101          

#define ERRSTAT_MASK                      (I8U)b00000100
#define ERRSTAT_ERROR                     (I8U)b00000000
#define ERRSTAT_STATUS                    (I8U)b00000100

// Error definitions
#define ERROR_COMM_I2C                    (I8U)b00000000
#define ERROR_COMM_SPI                    (I8U)b00000001
#define ERROR_INVALID_REGISTER            (I8U)b00000010
#define ERROR_COMM_I2C_SINGLE             (I8U)b00000011
#define ERROR_WRITE_ONLY                  (I8U)b00000100
#define ERROR_READ_ONLY                   (I8U)b00000101
#define ERROR_INVALID_VALUE               (I8U)b00000110
#define ERROR_SPURIOUS_INT                (I8U)b00000111
#define ERROR_SPI_INPUT_OVERFLOW          (I8U)b00010000
#define ERROR_SPI_OUTPUT_OVERFLOW         (I8U)b00010001

// Status code defintions
#define STATUS_FRAMING_MARKER             (I8U)b00000000
#define STATUS_FIFO_EMPTY                 (I8U)b11111111

// MCU Register definitions
#define REGISTER_MASK                   (I8U)0x1F

// -----------------------------------------------------------------------------------
// MCU Control register
#define CTL_IM            b10000000
#define CTL_A2            b01000000
#define CTL_A1            b00100000
#define CTL_TS            b00010000
#define CTL_DF            b00001000
#define CTL_A0            b00000100
#define CTL_FA            b00000010
#define CTL_RFNU          b00000001

// ------------------------------------------------------------------------------------
// MCU Status register
#define STAT_USBPP        b10000000
#define STAT_BACK         b01000000
#define STAT_HOME         b00100000
#define STAT_AHWM         b00010000
#define STAT_FOFL         b00001000
#define STAT_WUT          b00000100
#define STAT_INT2         b00000010
#define STAT_INT1         b00000001

// ------------------------------------------------------------------------------------
// The combination of {MCU_REVH, MCU_REVL} make up a 16-bit register. It has 3 fields: 
#define REV_DEV_MASK			(I16U)0x8000
#define REV_MAJOR_MASK			(I16U)0x7E00
#define REV_MINOR_MASK			(I16U)0x01FF
#define REV_DEV_POSITION		7 /* this is within REVH */
#define REV_MAJOR_POSITION		1 /* this is within REVH */
#define REV_MINOR_POSITION		8 /* this is within {REVH,REVL} */
#define REV_MINOR_MSB_MASK		(I16U)0x0100 /* used to extract MSB of Minor rev to include in REVH */
#define REV_MINOR_REVL_MASK		(I16U)0x00FF

#define REGISTER_MCU_HWM                (I8U)0x02

#define REGISTER_MCU_WUT                (I8U)0x03

#define REGISTER_MCU_FL                 (I8U)0x04

#define REGISTER_MCU_I2CADDR            (I8U)0x05
#define I2CADDR_CN        b00010000
#define CN_MMA845XQ       b00000000
#define CN_BMP085         b00010000
#define I2CADDR_RA5       b00000001

#define REGISTER_MCU_CLS                (I8U)0x06
#define CLS_UC            b00001000
#define CLS_RESERVED      b00000100
#define CLS_CL_MASK       b00000011

#define REGISTER_MCU_TS                 (I8U)0x07

#define REGISTER_MCU_CLKYEARH           (I8U)0x08
#define REGISTER_MCU_CLKYEARL           (I8U)0x09
#define REGISTER_MCU_CLKMON             (I8U)0x0a
#define REGISTER_MCU_CLKDAY             (I8U)0x0b
#define REGISTER_MCU_CLKDOW             (I8U)0x0c
#define REGISTER_MCU_CLKHOUR            (I8U)0x0d
#define REGISTER_MCU_CLKMIN             (I8U)0x0e
#define REGISTER_MCU_CLKSEC             (I8U)0x0f

#define REGISTER_MCU_FIFO0              (I8U)0x10

#define REGISTER_MCU_FIFO1              (I8U)0x11

#define REGISTER_MCU_CTL1        (I8U)0x14
#define CTL1_DBG_EN       b00000001

// -----------------------------------------------------------------------------
// MCU hardware info
#define HWINFO_ALTID      b00001100
#define ALTID_NONE        b00000000
#define ALTID_BMP085      b00000100
#define ALTID_MPL3115     b00001000
#define HWINFO_MMAID      b00000011
#define MMAID_NONE        b00000000
#define MMAID_MMA8451     b00000001
#define MMAID_MMA8452     b00000010

#define REGISTER_MCU_DEVADDR            (I8U)0x16
#define REGISTER_MCU_DEVDATA            (I8U)0x17

#define REGISTER_MCU_TEST_XH            (I8U)0x18
#define REGISTER_MCU_TEST_XL            (I8U)0x19
#define REGISTER_MCU_TEST_YH            (I8U)0x1a
#define REGISTER_MCU_TEST_YL            (I8U)0x1b
#define REGISTER_MCU_TEST_ZH            (I8U)0x1c
#define REGISTER_MCU_TEST_ZL            (I8U)0x1d

#define REGISTER_MCU_RESERVED_1E        (I8U)0x1e

#define REGISTER_MCU_TEST_COMMAND       (I8U)0x1f

#define REGISTER_FILE_SIZE              32

#endif // __SYSTEM_REGISTER_HEADER__

/** @} */
