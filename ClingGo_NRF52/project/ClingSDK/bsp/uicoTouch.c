/********************************************************************************

 **** Copyright (C), 2015, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : uicoTouch.c
 * Author        : MikeWang
 * Date          : 2015-10-09
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 *
 * Record        :
 * 1.Date        : 2015-10-09
 *   Author      : MikeWang
 *   Modification: Created file

  1.Date         : 20151015
    Author       : MikeWang
    Modification : add lenth into uico firmware
*************************************************************************************************************/
#include "main.h"
#include "uicoTouch.h"

#include "uicoData.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
// When you are writing these:
//
//     _i2c_write_reg(0x00, COMMAND_GET_DATA);
//     _i2c_write_reg(0x01, SYSTEM_INFO);
//
//     _i2c_read_reg(0x20, 1, (Data+0));
//     _i2c_read_reg(0x21, 1, (Data+1));
//
// Are you using the 0x48 address? These should be using the 0x48 address because this is trying to get the Revision of firmware from
// the application code. Since the application code doesn't exist it will just NAK, and then it will be done. However, if you are trying
// to use the bootloader address 0x58, you are getting 0x10 returned, which means that the bootloader does not understand what you are
// trying to send it.
//
// From the note above, it says the following:
//
// Main application address for PSOC4 controller is 0x48 (7-bit).
// Bootloader address for PSOC1 controller is 0x38 (7-bit). This will not be used.
// Bootloader address for PSOC4 controller is 0x58 (7-bit).
//
// The function 'try_firmware_update' will use both addresses, as it needs to access information from both the 0x48 main application,
// and 0x58 the bootloader. When it tries to access the System information, it will be using the 0x48 address, but when it tries to
// access bootloader commands, by sending the bootloader bin file lines via I2C, it will use the 0x58 address.
//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define TUNING                            0x01

#define DURATOUCH_I2C_ADDRESS_MAIN_0x48            0x90        // 0x48
#define DURATOUCH_I2C_ADDRESS_BOOTLOADER_0x58      0xB0        // 0x58

#define COMMAND_GET_DATA                  0x85        // Get Signature from Controller
#define COMMAND_READPACKET                0x81
#define SYSTEM_INFO                       0x01        // System Information

#define PRODUCT_ID_4_3in                  0xFF
#define PRODUCT_ID_7_0in                  0xFE
#define PRODUCT_ID_10_1in                 0xFD

#define USED_PRODUCT_ID_4_3in             0x03
#define USED_PRODUCT_ID_7_0in             0x07
#define USED_PRODUCT_ID_10_1in            0x0A

#define CUSTOM_PRODUCT_ID_7_0in           0x01

#define MAX_REFRESH_PER_FINGER_4_3in      16000
#define MAX_REFRESH_PER_FINGER_7_0in      18500
#define MAX_REFRESH_PER_FINGER_10_1in     22500
#define MAX_REFRESH_PER_FINGER_default    25000

#define TRACKID_MAX                       32000
#define PRESSURE_MAX                      0xFF

#define SIGNATURE_RESPONSE_LENGTH	        18          // SIGNATURE returns 13 bytes
#define PAD_LENGTH			                   0          // No padding needed.
#define UPGRADE_SIGNATURE_LENGTH	         8
#define	FLASH_ROW_SIZE			              64          // ** Set to 64 for <Redacted>

#define UPGRADE_PREAMBLE_LENGTH           10
#define UPGRADE_BLOCK_LENGTH             (12+ FLASH_ROW_SIZE +2)		   // 78
#define UPGRADE_SEGMENT_LENGTH		        32
#define UPGRADE_EXIT_LENGTH		            10

#define UICO_BIN_SIGNATURE		            'I'                          // binary always begins with this first byte

// Bootloader commands (FF xx)
#define COMMAND_ENTER_BOOTLOAD           0x38
#define COMMAND_FLASH_WRITE              0x39
#define COMMAND_FLASH_VERIFY             0x3A                         // Obsolete, automatically done when flash writen
#define COMMAND_BOOTLOAD_DONE            0x3B
#define COMMAND_BOOTLOAD_SELECT          0x3C

// Bootloader status code
#define BOOT_COMPLETED_OK                0x01                        // Never sent because it reboots
#define IMAGE_VERIFY_ERROR               0x02
#define FLASH_CHECKSUM_ERROR             0x04
#define FLASH_PROTECTION_ERROR           0x08
#define COMM_CHECKSUM_ERROR              0x10
#define BOOTLOAD_MODE                    0x20
#define INVALID_KEY                      0x40
#define INVALID_COMMAND_ERROR            0x80

#define LARGE_FORMAT                        0

#define UICO_ACCESS_REG_COMMAND  0X00
#define UICO_USER_COMMAND 0x8C
#define UICO_USER_SPECIFIC_COMMAND_CALIBRATION_WHEN_FLOATIING 0XA0
#define UICO_USER_SPECIFIC_COMMAND_CALIBRATION_WHEN_FIXTURE   0XA5
#define UICO_CALIBRATION_LENTH   3
#define UICO_ERROR_CODE 0xff
#define UICO_CALIBRATION_PROCCES_LENTH  2
#define UICO_CALIBRATION_SUCESS_CODE  0X00
#define UICO_CALIBRATION_FAILED_CODE  0XFE
#define UICO_DEBUG 

/*======LOCAL VAR=======================================================================================*/
static uint8_t is_calibration_sucess =  FALSE;/*INFICATE IF MANUAL CALIBRATION HAS BEEN EXCEXUTED SUCESSFULLY*/
static UICOTOUCH_RESPONSE_CTX res[5];
static I8U prev_code;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#if 0
static EN_STATUSCODE _i2c_btld_read(I8U *pi8uRegValue, I8U number_of_bytes)
{
    I32U err_code;
    const nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(1);

    if (NULL == pi8uRegValue) {
        return STATUSCODE_NULL_POINTER;
    }
    if (!cling.system.b_twi_1_ON) {
        GPIO_twi_enable(1);
    }

    err_code = nrf_drv_twi_rx(&twi, (DURATOUCH_I2C_ADDRESS_BOOTLOADER_0x58 >> 1), pi8uRegValue, number_of_bytes, FALSE);
 
    if (err_code == BLE_STATUS_OK) {
        N_SPRINTF("[UICO] BOOTLOADER ADDR READ PASS.");

        return STATUSCODE_SUCCESS;
    } else {
        N_SPRINTF("[UICO] BOOTLOADER ADDR READ FAIL - %d", err_code);
				APP_ERROR_CHECK(err_code);
        return STATUSCODE_FAILURE;
    }
}
#endif
/*****************************************************************************
 * Function      : _i2c_btld_write
 * Description   : used to write specific number of data into touch ic
 * Input         : I8U *pi8uRegValue
                   I8U number_of_bytes
 * Output        : None
 * Return        : static
 * Others        :
 * Record
 * 1.Date        : 20151009
 *   Author      : MikeWang
 *   Modification: Created function

*****************************************************************************/
#if 0
static BOOLEAN _i2c_btld_write(I8U *pi8uRegValue, I8U number_of_bytes)
{
    I32U err_code;
    const nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(1);
    if (!cling.system.b_twi_1_ON) {
        GPIO_twi_enable(1);
    }
    err_code = nrf_drv_twi_tx(&twi, (DURATOUCH_I2C_ADDRESS_BOOTLOADER_0x58 >> 1), pi8uRegValue, number_of_bytes, FALSE);
    BASE_delay_msec(1);


    if (err_code == BLE_STATUS_OK) {
        N_SPRINTF("[UICO] BOOTLOADER ADDR WRITE DATA PASS.");
    } else {
        Y_SPRINTF("[UICO] BOOTLOADER ADDR WRITE DATA FAIL-%d", err_code);
    }
    return err_code;
}
#endif
/*****************************************************************************
 * Function      : _i2c_main_read
 * Description   : i2c read driver finction
 * Input         : I8U *pi8uRegValue
                I8U number_of_bytes
 * Output        : None
 * Return        : static
 * Others        :
 * Record
 * 1.Date        : 20151015
 *   Author      : MikeWang
 *   Modification: Created function

*****************************************************************************/
static EN_STATUSCODE _i2c_main_read(I8U *pi8uRegValue, I8U number_of_bytes)
{
//    I32U err_code;
//    const nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(1);
//
//    if (NULL == pi8uRegValue) {
//        return STATUSCODE_NULL_POINTER;
//    }
//
//    if (!cling.system.b_twi_1_ON) {
//        GPIO_twi_enable(1);
//    }
//
////    err_code = nrf_drv_twi_rx(&twi, (DURATOUCH_I2C_ADDRESS_MAIN_0x48 >> 1), pi8uRegValue, number_of_bytes, FALSE);
//
//    if (err_code == BLE_STATUS_OK) {
//        N_SPRINTF("[UICO] MAIN ADDR READ PASS.");
//        return STATUSCODE_SUCCESS;
//    } else {
//        Y_SPRINTF("[UICO] MAIN ADDR READ FAIL: %d", err_code);
//				APP_ERROR_CHECK(err_code);
//        return STATUSCODE_FAILURE;
//    }
        return STATUSCODE_SUCCESS;
}

static BOOLEAN _i2c_main_write(I8U *pi8uRegValue, I8U number_of_bytes)
{
//    I32U err_code;
//    const nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(1);
//
//    if (!cling.system.b_twi_1_ON) {
//        GPIO_twi_enable(1);
//    }

//    err_code = nrf_drv_twi_tx(&twi, (DURATOUCH_I2C_ADDRESS_MAIN_0x48 >> 1), pi8uRegValue, number_of_bytes, FALSE);
//    BASE_delay_msec(1);

//    if (err_code == BLE_STATUS_OK) {
//        N_SPRINTF("[UICO] MAIN ADDR WRITE DATA PASS.");
//    } else {
//        Y_SPRINTF("[UICO] MAIN ADDR WRITE DATA FAIL - %d", err_code);
//				APP_ERROR_CHECK(err_code);
//    }
//    return err_code;
        return STATUSCODE_SUCCESS;
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*****************************************************************************
 * Function      : _write_stop_acknowledge
 * Description   : i2c write acknolege function
 * Input          : None
 * Output        : None
 * Return        : static
 * Others        :
 * Record
 * 1.Date        : 20151015
 *   Author      : MikeWang
 *   Modification: Created function

*****************************************************************************/
static void _write_stop_acknowledge()
{
    I8U buf[2];

    // Write Stop Acknowledge (0x20, 0x01)
    buf[0] = 0x20;
    buf[1] = 0x01;
    _i2c_main_write(buf, 2);
}
#if 0
static void _write_clear_response()
{
    I8U buf[2];

    // Write Start Acknowledge (0x20)
    buf[0] = 0x20;
    _i2c_main_write(buf, 1);
}
#endif

static void _write_start_acknowledge(I8U *buf)
{
    // Write Start Acknowledge (0x20)
    buf[0] = 0x20;
    _i2c_main_write(buf, 1);
}

#if defined(_ENABLE_BLE_DEBUG_) || defined(_ENABLE_UART_)
void UICO_dbg_write_read(I8U *obuf, I8U len, I8U *ibuf)
{
    // Write Start Acknowledge (0x20)
    _i2c_main_write(ibuf, len);

    BASE_delay_msec(50);
   // _write_clear_response();

    BASE_delay_msec(1);

    // Read out 10 bytes
    _i2c_main_read(obuf, len);

    _write_stop_acknowledge();
}
#endif
/*****************************************************************************
 * Function      : uico_touch_ic_floating_calibration_start
 * Description   : i2c read driver finction
 * Input         : I8U *pi8uRegValue
                I8U number_of_bytes
 * Output        : None
 * Return        : static
 * Others        :
 * Record
 * 1.Date        : 20151015
 *   Author      : MikeWang
 *   Modification: Created function

*****************************************************************************/

void GPIO_interrupt_handle(void);
bool uico_touch_is_floating_calibration_sucessfully(void);
#define MAX_CYCLES_WAITING_FOR_CALIBRATION_RESPONSE  2000

bool uico_touch_ic_floating_calibration_start()
{

    uint32_t i = 0;
    uint8_t command_buffer[UICO_CALIBRATION_LENTH] = {UICO_ACCESS_REG_COMMAND, UICO_USER_COMMAND, UICO_USER_SPECIFIC_COMMAND_CALIBRATION_WHEN_FLOATIING};
    if(_i2c_main_write(command_buffer, UICO_CALIBRATION_LENTH) == BLE_STATUS_OK) {
#ifdef UICO_DEBUG
        while(uico_touch_is_floating_calibration_sucessfully() != TRUE) {
            GPIO_interrupt_handle();
            i++;
            if(i > MAX_CYCLES_WAITING_FOR_CALIBRATION_RESPONSE) {
                return FALSE;
            }
        }
#endif
        return TRUE;

    } else {
        return FALSE;
    }
}
/*****************************************************************************
 * Function      : uico_touch_ic_floating_calibration_response_process
 * Description   : process ucalibration response from touch ic
 * Input         : I8U *buffer
                   I8U lenth
 * Output        : None
 * Return        : static
 * Others        :
 *  
 * 1.Date        : 20151015
 *   Author      : MikeWang
 *   Modification: Created function

*****************************************************************************/
static bool uico_touch_ic_floating_calibration_response_process(uint8_t *buffer, uint16_t lenth)
{
    uint8_t i = 0;
    N_SPRINTF("[UICO] calibration_response:0x%02x, 0x%02x, 0x%02x, 0x%02x", buffer[0], buffer[1], buffer[2], buffer[3]);
    /*This means there is something wrong with tthe lenth of data*/
    if(lenth <= UICO_CALIBRATION_PROCCES_LENTH) {
        return FALSE;
    }
    /*buffer proess eraea*/
    if(buffer[i] == UICO_USER_COMMAND) {
        N_SPRINTF("[UICO] calibration_response:0x%02x", buffer[i]);
        i++;
        if(buffer[i] == UICO_ERROR_CODE) {
            /*Customer-Specific Commands is Not Supported*/
            return FALSE; 
        } else if(buffer[i] == UICO_CALIBRATION_PROCCES_LENTH) {
            N_SPRINTF("[UICO] calibration_response:0x%02x", buffer[i]);
            i++;
            if(buffer[i] == UICO_USER_SPECIFIC_COMMAND_CALIBRATION_WHEN_FLOATIING) {
                N_SPRINTF("[UICO] calibration_response:0x%02x", buffer[i]);
                i++;
                if(buffer[i] == UICO_ERROR_CODE) {
                    /*The Customer-Specific Command Code was Not Recognized*/
                    return FALSE;
                } else {
                    N_SPRINTF("[UICO] calibration_response:0x%02x", buffer[i]);
                    if((buffer[i] == UICO_CALIBRATION_SUCESS_CODE)) {
                        /*The Touch Controller Successfully Processed the Customer-Specific Command*/
                        N_SPRINTF("[UICO] calibration_response sucess");
                        is_calibration_sucess = TRUE;
                        return TRUE;
                    } else {
                        /*The Touch Controller Failed to Process the Customer-Specific Command*/
                        return FALSE;
                    }

                }
            } else {
                /**/
                return FALSE;
            }

        }

    }
		return FALSE;
}
bool uico_touch_is_floating_calibration_sucessfully()
{
    return is_calibration_sucess;
}
/*****************************************************************************
 * Function      : UICO_set_power_mode
 * Description   : set touchi ic power mode function
 * Input         : I8U *pi8uRegValue
                I8U number_of_bytes
 * Output        : None
 * Return        : static
 * Others        :
 * Record
 * 1.Date        : 20151015
 *   Author      : MikeWang
 *   Modification: Created function

*****************************************************************************/
void UICO_set_power_mode(UICO_POWER_MODE mode)
{
    I8U buf[2];

    buf[0] = 0x00;
    if (mode == UICO_POWER_DEEP_SLEEP) {
        buf[1] = 0x83;
    } else {
        buf[1] = 0x81;
    }
    _i2c_main_write(buf, 2);
}

#ifdef UICO_INCLUDE_FIRMWARE_BINARY_UPDATE_ENABLE
static void _setup_buffer(I8U* dest, I32S start_byte, I32S length, unsigned char *s)
{
    I32S i;

#if	PAD_LENGTH
    for (i = 0; i < PAD_LENGTH; i++)
        dest[i] = 0x00;
#endif

    for (i = 0; i < length; i++)
        dest[i + PAD_LENGTH] = s[start_byte + i];
}

static I32S _execute_bootloader(I32S payload_length, unsigned char *s)
{
    I8U  buffer[16 + UPGRADE_BLOCK_LENGTH];
    I8U  response[16];
    I32S index = UPGRADE_SIGNATURE_LENGTH + 8;
    I32S sending = 1;
    I32S length;
    I32S retry_count;
    //unsigned char* uicoDat = s;

    while ( sending ) {
        Watchdog_Feed();

        // every string starts with FF3x.  If first byte isn't FF, something is wrong
        if (s[index] != 0xFF) {
            N_SPRINTF("[UICO] ERROR: execute_bootload has lost sync with the file (0x%02x)", uicoDat[index]);
            return -1;
        } else {

            switch ( s[index + 1] ) {

                case COMMAND_ENTER_BOOTLOAD:
                    length = 2 + 8;
                    N_SPRINTF("[UICO] COMMAND_ENTER_BOOTLOAD.");
                    break;

                case COMMAND_FLASH_WRITE:
                    length = UPGRADE_BLOCK_LENGTH;
                    N_SPRINTF("[UICO] UPGRADE_BLOCK_LENGTH.");
                    break;

                case COMMAND_FLASH_VERIFY:
                    length = 2 + 8;
                    N_SPRINTF("[UICO] COMMAND_FLASH_VERIFY.");
                    break;

                case COMMAND_BOOTLOAD_DONE:
                    length = 2 + 8;
                    sending = 0;
                    N_SPRINTF("[UICO] COMMAND_BOOTLOAD_DONE.");
                    break;

#if LARGEFORMAT
                case COMMAND_BOOTLOAD_SELECT:
                    length = 2 + 8 + 2;
                    N_SPRINTF("[UICO] COMMAND_BOOTLOAD_SELECT.");
                    break;
#else
                case COMMAND_BOOTLOAD_SELECT:
                    length = 0;
                    N_SPRINTF("[UICO] COMMAND_BOOTLOAD_SELECT.");
                    break;
#endif

                default:
                    N_SPRINTF("[UICO] ERROR: execute_bootload has lost sync with the file (0x%02x)", uicoDat[index]);
                    return -1;
            }
        }

        N_SPRINTF("[UICO] Writing block 0x%02x...", uicoDat[index + 11]);

        _setup_buffer(buffer, index, length, s);
        retry_count = 3;

        if ( !(length == 0) ) {
            while (retry_count > 0 && (length != 0)) {
                retry_count--;

                N_SPRINTF("[UICO] Sent Data: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x length: %d ...", \
                          buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], \
                          buffer[6], buffer[7], buffer[8] , buffer[9], buffer[10], buffer[11], length);

                _i2c_btld_write(buffer, length);
                BASE_delay_msec(100);					// 100 msec wait after burn

                if (s[index + 1] == COMMAND_BOOTLOAD_DONE) {                       	// No response expected becauase chip will reboot
                    break;
                }

                _i2c_btld_read(&response[0], 1);
//		  N_SPRINTF("[UICO] Received Data: %02x %02x", buffer[0], response[0]);

                if (response[0] == BOOTLOAD_MODE)	{           // Success!
                    N_SPRINTF("[UICO] Success!");
                    break;
                }

                if ((response[0] & (COMM_CHECKSUM_ERROR | FLASH_CHECKSUM_ERROR)) == 0) {              // Only retry on checksum errors
                    N_SPRINTF("[UICO] Success!");
                    break;
                }
            }
        }

//	N_SPRINTF("[UICO] Returned Data: %02x %02x %02x %02x %02x %02x %02x %02x", response[0], response[1], response[2], response[3], response[4], response[5], response[6], response[7]);

        if ((response[0] != BOOTLOAD_MODE) && (s[index + 1] != COMMAND_BOOTLOAD_DONE) && (length != 0)) {
            N_SPRINTF("[UICO] FAIL on block 0x%02x write (response 0x%02x 0x%02x)  0x%02x  %d", uicoDat[index + 11], response[0], response[1], uicoDat[index + 1], length);
            return response[0];
        }
        // block was acknowledged, move to next block and loop

        index = index + length;
        if(length == 0) {
            index += 2 + 8 + 2;
        }

        // make sure we don't leave the buffer.  If we overflow, there may be problems.
        if (index > payload_length) {
            sending = 0;
        }

        // take a short pause before starting the next block
        BASE_delay_msec(5);
    }                   // end while (sending)

    N_SPRINTF("[UICO] Upgrade completed, touchscreen is restarting");
    BASE_delay_msec(526);
    return 0;                     // Success!
}
#endif

static I32S _try_firmware_update()
{
    /*read touch ic failure uo threhold*/
#define TOUCH_IC_BRICKED_THREHOLD  10
    I8U  data[128];
    volatile I16S i = 0;
    volatile bool is_touchic_bricked = false;

    /*obtian firmware lenth*/
#if ((defined UICO_FORCE_BURN_FIRMWARE) || (defined UICO_INCLUDE_FIRMWARE_BINARY))
		uint16_t uico_binary_lenth = 0;
    uico_binary_lenth = UICO_GET_BINARY_LENTH();
    s = UICO_GET_BINARY_BUFFER();
#endif

#ifdef UICO_FORCE_BURN_FIRMWARE
    return _execute_bootloader(uico_binary_lenth, s);
#endif

    /*set buffer data*/
    memset(data, 0xff, sizeof(data));
    i = 0;
    while ( !((data[0] == COMMAND_GET_DATA) && (data[1] == 17)) ) {

        data[0] = 0x00;
        data[1] = COMMAND_GET_DATA;
        data[2] = SYSTEM_INFO;
        _i2c_main_write(data, 3);
        data[0] = 0x20;
        _i2c_main_write(data, 1);
        _i2c_main_read (data, 2);
        N_SPRINTF("[UICO]: 0x%02x 0x%02x", data[0], data[1]);

#if 1
        if ( !((data[0] == COMMAND_GET_DATA) && (data[1] == 17)) ) {
            // Write Stop Acknowledge
            _write_stop_acknowledge();
        }
        i++;
        /*if read spedic data error happened more than up threhold then we take the ic as a brick one or even not no touchi ic connected to */
        if(i > TOUCH_IC_BRICKED_THREHOLD) {
            is_touchic_bricked = true;
            break;
        }
#endif

    }

    /*if this ic is  not bricked or unconnected*/
    if(is_touchic_bricked == false) {
        /*clear the temp varies*/
        i = 0;
        // Read 2 + temp[1] bytes
        _i2c_main_read (data, 2 + data[1]);

        // Write Stop Acknowledge
        _write_stop_acknowledge();

        /*revision format major version:minor version: revison version*/
        N_SPRINTF("[UICO] Firmware version found: %d.%d.%d", data[10],   data[11],   data[18]);
        N_SPRINTF("[UICO] Firmware version got: %d.%d.%d", s[4], s[5], s[9]);
        cling.whoami.touch_ver[0] = data[10];
        cling.whoami.touch_ver[1] = data[11];
        cling.whoami.touch_ver[2] = data[18];

#ifdef UICO_INCLUDE_FIRMWARE_BINARY_UPDATE_ENABLE
        // Now look in the box and check software versions.
        //
        // *** NOTE: We don't check if the version # is going UP.  This check will let any non-matching file burn.
        //           Future versions should check for an upgrade and not a downgrade.
        // Compare Version Major, Version Minor, and Tuning Revision for match
#if 1
        if((data[10] == s[4]) && (data[11] == s[5]) && (data[18] == s[9])) {
            N_SPRINTF("[UICO] 1 No new firmware version found, no upgrade necessary. %d %d %d", uicoDat[4], uicoDat[5], uicoDat[3]);
            return -1;
        }
#else
        if (data[10] > s[4]) {
            N_SPRINTF("[UICO] 1 No new firmware version found, no upgrade necessary.");
            return -1;
        } else if (data[10] == s[4]) {
            if (data[11] > s[5]) {
                N_SPRINTF("[UICO] 2 No new firmware version found, no upgrade necessary.");
                // return -1;
            } else if (data[11] == s[5]) {
                if ((data[18] >= s[9]) /* && (!bForce_flash) */   ) {
                    N_SPRINTF("[UICO] 3 No new firmware version found, no upgrade necessary.");
                    return -1;
                }

//#ifndef UICO_FORCE_BURN_FIRMWARE
//    		N_SPRINTF("[UICO] 4 No new firmware version found, no upgrade necessary.");
//    		return -1;
            }
        }
#endif

        _execute_bootloader(uico_binary_lenth, (unsigned char*)s);

        SYSTEM_reboot();
#else
        return 0;
#endif
    } else {
#if ((defined UICO_FORCE_BURN_FIRMWARE) || (defined UICO_INCLUDE_FIRMWARE_BINARY_UPDATE_ENABLE)) 
        N_SPRINTF("[UICO] uico ic has been bricked force to update it .");
        _execute_bootloader(uico_binary_lenth, (unsigned char*)s);
        SYSTEM_reboot();
#endif
    }

	return 0;
} 



void UICO_init()
{
    I8U  buf[128];
    I16S i = 0;
    I8U total_sensor_cnt   __attribute__((unused)), column_sensor_cnt   __attribute__((unused));
    I32S resolution_x   __attribute__((unused)) = 0;
    I32S resolution_y    __attribute__((unused))= 0;
    I32S screen_size_code  = 0;
    I32S refresh_rate = 0;
    I8S touch_num_supported   __attribute__((unused)) = 0;

    // Initialize static valables
    for (i = 0; i < 5; i++) {
        res[i].ts = 0;
        res[i].type = 0;
        res[i].x = 0;
    }

    // Start firmware update
    for (i = 0; i < 128; i++) buf[i] = 0xFF;

    _try_firmware_update();


    while( !((buf[0] == COMMAND_GET_DATA) && (buf[1] == 0x11)) ) {
        /*--------------------------- Write Command ---------------------------*/
        buf[0] = 0x00;
        buf[1] = COMMAND_GET_DATA;
        buf[2] = SYSTEM_INFO;

        _i2c_main_write(buf, 3);

        /*--------------------------- Acknowledge Any Response ---------------------------*/
        // Write Start Acknowledge (0x20)
        buf[0] = 0x20;
        _i2c_main_write(buf, 1);

        /*--------------------------- Read Response  ---------------------------*/
        // Read First 2 bytes
        _i2c_main_read (buf, 2);


        if( !((buf[0] == COMMAND_GET_DATA) && (buf[1] == 0x11)) ) {
            /*--------------------------- Acknowledge for Correct Command  ---------------------------*/
            // Write Stop Acknowledge
            _write_stop_acknowledge();
        }

    }
    N_SPRINTF("[UICO init]: 0x%02x 0x%02x 0x%02x", buf[0], buf[1], 0x55);
    // Read 2 + buf[1] bytes
    _i2c_main_read(buf, 2 + buf[1]);

    // Write Stop Acknowledge (0x20, 0x01)
    _write_stop_acknowledge();

    resolution_x = (buf[13] << 8) + buf[14];
    resolution_y = (buf[15] << 8) + buf[16];

    touch_num_supported = buf[7];
    screen_size_code    = buf[6];

    N_SPRINTF("[UICO]: %d %d %d %d", resolution_x, resolution_y, touch_num_supported, screen_size_code);

    if( refresh_rate == 0 ) {
        switch(screen_size_code) {
            case PRODUCT_ID_4_3in:
                refresh_rate = MAX_REFRESH_PER_FINGER_4_3in;
                break;
            case PRODUCT_ID_7_0in:
                refresh_rate = MAX_REFRESH_PER_FINGER_7_0in;
                break;
            case PRODUCT_ID_10_1in:
                refresh_rate = MAX_REFRESH_PER_FINGER_10_1in;
                break;
            case USED_PRODUCT_ID_4_3in:
                refresh_rate = MAX_REFRESH_PER_FINGER_4_3in;
                break;
            case USED_PRODUCT_ID_7_0in:
                refresh_rate = MAX_REFRESH_PER_FINGER_7_0in;
                break;
            case USED_PRODUCT_ID_10_1in:
                refresh_rate = MAX_REFRESH_PER_FINGER_10_1in;
                break;
            case CUSTOM_PRODUCT_ID_7_0in :
                refresh_rate = MAX_REFRESH_PER_FINGER_7_0in;
                break;
            default:
                refresh_rate = MAX_REFRESH_PER_FINGER_default;
                break;
        }
    }
    total_sensor_cnt = buf[4];
    column_sensor_cnt = buf[5];
    N_SPRINTF("[UICO]: Total Sensor Count: %d Column Sensor Count: %d", total_sensor_cnt, column_sensor_cnt);
    N_SPRINTF("[UICO]: Product Screen Size Code: %x", screen_size_code);
    N_SPRINTF("[UICO]: Number of Touches Supported: %d", touch_num_supported);
    N_SPRINTF("[UICO]: X Resolution: %d Y Resolution: %d", resolution_x, resolution_y);

}

BOOLEAN _is_finger_touched(I8U index)
{
    // ------ NOTHING ----------
    // ------ SKIN ----------
    if ((res[index].type == 0) || (res[index].type == 2))
        return FALSE;
    else
        return TRUE;
}

BOOLEAN _is_skin_touched(I8U index)
{
    // ------ NOTHING ----------
    // ------ FINGER ----------
    if ((res[index].type == 0) || (res[index].type == 1))
        return FALSE;
    else
        return TRUE;
}

//#define UICO_INT_MESSAGE

I8U UICO_main()
{
#ifdef UICO_INT_MESSAGE
    I32S  a, b, c, d, e, f, g, h, i, j, coor;
#endif
    I8U  buf[32]  = {0};
    I8U opcode, opdetail, len;
    I8U left_cnt, right_cnt;
    I32S dist_pos;
    I32S dist_neg;

    opcode = 0;
    opdetail = 0;

    /*--------------------------- Acknowledge Any Response ---------------------------*/
    // Write Start Acknowledge (0x20)
    _write_start_acknowledge(buf);

    /*--------------------------- Read Response  ---------------------------*/
    // Read First 2 bytes
    _i2c_main_read (buf, 2);
    /////

#ifdef UICO_INT_MESSAGE
    a = buf[0];
    b = buf[1];
    N_SPRINTF("[UICO] Packet 2 bytes %d, %d, ", a, b);
#endif

    len =  buf[1];
    if (len == 0) {
        _write_stop_acknowledge();
        return 0xff;
    }
    // Read the remaining bytes
    len += 2;
    _i2c_main_read(buf, len);
    // Write Stop Acknowledge
    _write_stop_acknowledge();

    if (buf[0] != COMMAND_READPACKET) {

#ifdef UICO_INT_MESSAGE
        N_SPRINTF("[UICO] NOT a read packet: %d, %d, ", a, b);
#endif
			        /*process user command individually*/
        if(buf[0] == UICO_USER_COMMAND) {
            uico_touch_ic_floating_calibration_response_process(buf, len);
        }
				
        return 0xff;
    }

    // Make sure the length is what we expected
    len = buf[1];
    if( len < 6 ) {

#ifdef UICO_INT_MESSAGE
        N_SPRINTF("[UICO] wrong len (%d) %d, %d", COMMAND_READPACKET, a, b);
#endif
        return 0xff;
    }

#ifdef UICO_INT_MESSAGE
    a = buf[0];
    b = buf[1];
    c = buf[2];
    d = buf[3];
    e = buf[4];
    f = buf[5];
    g = buf[6];
    h = buf[7];
    i = buf[8];
    j = buf[9];
    N_SPRINTF("[UICO] Packet (%d) %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", a,
              a, b, c, d, e, f, g, h, i, j);
#endif

    // Put response in a circular buffer
    res[0].type = res[1].type;
    res[0].x = res[1].x;
    res[1].type = res[2].type;
    res[1].x = res[2].x;
    res[2].type = res[3].type;
    res[2].x = res[3].x;
    res[3].type = res[4].type;
    res[3].x = res[4].x;
    res[4].type = buf[3];
    res[4].x = buf[5];

    // type: 0 ------ NOTHING ----------
    // type: 1 ------ FINGER ----------
    // type: 2 ------ SKIN ----------
    // type: 3 ------ SKIN + FINGER ----------
    if (_is_skin_touched(4)) {
        N_SPRINTF("SKIN TOUCH DETECTED");
        opcode |= 0x04;
    }

    if (!_is_finger_touched(4)) {
        // Reset previous code cache
        prev_code = 0;
    }

    // Finger down
    if (!_is_finger_touched(4)) {
        if (_is_finger_touched(3)) {
            if ((!_is_finger_touched(2)) ||
                (!_is_finger_touched(1))) {
                opcode |= 0x02;
                if (res[3].x > 84)
                    opdetail = 0;
                else if (res[3].x > 42)
                    opdetail = 1;
                else
                    opdetail = 2;
#ifdef UICO_INT_MESSAGE
                coor = res[3].x;
                N_SPRINTF("FINGER DOWN DETECTED 1(%d), coor: %d", prev_code, coor);
#endif
            } else {
#if 0 // Do not send a finger touch event if previously swipe gesture has been sent
                opcode |= 0x02;
                if (res[3].x > 84)
                    opdetail = 0;
                else if (res[3].x > 42)
                    opdetail = 1;
                else
                    opdetail = 2;
#endif
#ifdef UICO_INT_MESSAGE
                coor = res[3].x;
                N_SPRINTF("FINGER DOWN DETECTED 2(%d), coor: %d", prev_code, coor);
#endif
            }
        }
    } else {
        if (_is_finger_touched(3) && _is_finger_touched(2))
//		if (_is_finger_touched(3) && _is_finger_touched(2) && _is_finger_touched(1))
        {
            left_cnt = 0;
            right_cnt = 0;
            dist_pos = 0;
            dist_neg = 0;

            if (res[4].x > res[3].x) {
                dist_pos += res[4].x - res[3].x;
                left_cnt ++;
            } else {
                dist_neg += res[3].x - res[4].x;
                right_cnt ++;
            }
            if (res[3].x > res[2].x) {
                dist_pos += res[3].x - res[2].x;
                left_cnt ++;
            } else {
                dist_neg += res[2].x - res[3].x;
                right_cnt ++;
            }

            if (_is_finger_touched(1)) {
                if (res[2].x > res[1].x) {
                    dist_pos += res[2].x - res[1].x;
                    left_cnt ++;
                } else {
                    dist_neg += res[1].x - res[2].x;
                    right_cnt ++;
                }
            }
#ifdef UICO_INT_MESSAGE

            N_SPRINTF("SWIPE DETECTED (%d)", prev_code);
#endif
            opcode |= 0x01;
#if 0
            if (left_cnt > right_cnt)
                opdetail = 0;
            else
                opdetail = 1;
#else
            if (dist_pos > dist_neg)
                opdetail = 0;
            else
                opdetail = 1;
#endif
        }
    }

    N_SPRINTF("[UICO] code: %d, detail: %d", opcode, opdetail);
    opcode <<= 3;
    opcode |= opdetail;

    if (prev_code != opcode) {
        prev_code = opcode;

        opcode |= 0x40;
    }

    return  opcode;
}

/******************************************************************************
 * FunctionName : transfer_data_to_uico_touch
 * Description	: tranfer specific  data to uico ic
 * Parameters	: tx_buffer_p : tx data buffer
 *                tx_lenth: tx data lenth
 *                rx_buffer_p   : rx bbuffer pointer
 *                rx_lenth :  data lenth recievied
 * Returns		: true: sucess false£º failed
*******************************************************************************/

#ifdef _TOUCH_DEBUG_OVER_BLE_
#define UICO_BUFFER_MAX_SIZE  30
#define UICO_START_ACK_LEN  1
int transfer_data_to_uico_touch(uint8_t *buffer_p, uint16_t tx_lenth, uint16_t *rx_lenth_p)
{
#if 0
    uint8_t *buffer = buffer_p; //[100];
    //  N_SPRINTF("[UICO]: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
    while( !((buffer[0] == COMMAND_GET_DATA) && (buffer[1] == 0x11)) ) {
        /*--------------------------- Write Command ---------------------------*/
        /*
          buffer[0] = 0x00;
          buffer[1] = 0x8a;
          buffer[2] = 0x01;
          buffer[3] = 0x02;
          buffer[4] = 0x03;
        */
        _i2c_main_write(buffer, tx_lenth);

        /*--------------------------- Acknowledge Any Response ---------------------------*/
        // Write Start Acknowledge (0x20)
        buffer[0] = 0x20;
        _i2c_main_write(buffer, 1);
        /*--------------------------- Read Response  ---------------------------*/
        // Read First 2 bytes
        _i2c_main_read (buffer, 2);
        N_SPRINTF("[UICO]: 0x%02x 0x%02x", buffer[0], buffer[1]);
        // Read 2 + buf[1] bytes
        _i2c_main_read(buffer, 2 + buffer[1]);

        N_SPRINTF("[UICO]: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
        // Write Stop Acknowledge (0x20, 0x01)
        _write_stop_acknowledge();

    }
#else
    uint8_t *buffer = buffer_p; //[100];
    *rx_lenth_p = 0;
    uint8_t i = 0;
    if(buffer == NULL || rx_lenth_p == NULL) {
        return  false;
    }

    /*send data buffer to uico ic*/
    _i2c_main_write(buffer, tx_lenth);
    //BASE_delay_msec(1000);
    /*
        interrupt synchronize is needed here
    */
    BASE_delay_msec(1);
    /*--------------------------- Acknowledge Any Response ---------------------------*/
    // Write Start Acknowledge (0x20)
    buffer[0] = 0x20;
    _i2c_main_write(buffer, 1);

    /*--------------------------- Read Response  ---------------------------*/
    // Read First 2 bytes

    _i2c_main_read (buffer, 2);
    N_SPRINTF("[UICO]: 0x%02x 0x%02x", buffer[0], buffer[1]);

    /*--------------------------- Acknowledge for Correct Command  ---------------------------*/
    *rx_lenth_p = buffer[1];
    if(*rx_lenth_p > 0) {
        /*read data of rx_lenth_p lenth*/
        buffer[0] = 0;
        _i2c_main_read(buffer,  2 + *rx_lenth_p);
        N_SPRINTF("[UICO]: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);

    }
    // Write Stop Acknowledge
    _write_stop_acknowledge();
    *rx_lenth_p += 2;
    N_SPRINTF("[UICO] lenth %d", *rx_lenth_p );
    return true;
    /*some thing eroor happened return false*/
    *rx_lenth_p = 0;
    return false;
#endif
}
/******************************************************************************
 * FunctionName : read_data_from_uico_touch
 * Description	: read specific  lenth data from uico ic for debug purpose
 * Parameters	: buffer_p : tx data buffer
 *                 rx_lenth :  data lenth need ro read
 * Returns		: true: sucess false£º failed
*******************************************************************************/
int read_data_from_uico_touch(uint8_t *buffer_p, uint16_t rx_lenth)
{
#if 0

#else
    uint8_t *buffer = buffer_p; //[100];
	
    if(buffer == NULL) {
        return  false;
    }

    /*--------------------------- Acknowledge Any Response ---------------------------*/
    // Write Start Acknowledge (0x20)
    buffer[0] = 0x20;
    _i2c_main_write(buffer, 1);


    _i2c_main_read(buffer,  rx_lenth);
    N_SPRINTF("[UICO]: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
    // Write Stop Acknowledge
    _write_stop_acknowledge();

    return true;
#endif
}
/******************************************************************************
 * FunctionName : read_unkown_lenth_from_uico_touch
 * Description	: when a isr happens  we do not  even khow the lenth we need
 * Parameters	: buffer_p :  data buffer
 *                 rx_lenth :  data lenth has been read
 * Returns		: true: sucess false£º failed
*******************************************************************************/
/*
        write to Touch IC: 0x20 // required; changes to the response buffer
        read 2 bytes from Touch IC and store in touch_data // determine the number of bytes in the response (byte 1 in the response)
        len = touch_data[1]

        if(touch_data[1] > 0) {
            read(len + 2) bytes from Touch IC and store in touch_data
        }

        write to Touch IC: 0x20 0x01 // required; clears the touch IC interrupt line;
*/

int read_unkown_lenth_from_uico_touch(uint8_t *buffer_p, uint16_t *rx_lenth)
{
#if 0

#else
    uint8_t *buffer = buffer_p; //[100];

    if(buffer == NULL) {
        return  false;
    }

    /*--------------------------- Acknowledge Any Response ---------------------------*/
    // Write Start Acknowledge (0x20)
    buffer[0] = 0x20;
    _i2c_main_write(buffer, 1);

    /*--------------------------- Read Response  ---------------------------*/
    // Read First 2 bytes

    _i2c_main_read (buffer, 2);
    N_SPRINTF("[UICO]: 0x%02x 0x%02x", buffer[0], buffer[1]);
    *rx_lenth =  buffer[1];
    *rx_lenth += 2;
    _i2c_main_read(buffer,  *rx_lenth);
    N_SPRINTF("[UICO]: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
    // Write Stop Acknowledge
    _write_stop_acknowledge();

    return true;
#endif
}

#endif

