/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H
#include "nrf_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PIN_VOLTAGE_CHANGE_SUPPORTED    (1)
#define _PIN_NAME_PORT_OFFSET_          8
#define GENERATE_PIN_NAME(PORT, PINNUM)  (uint16_t)((PORT<<8) | (PINNUM))
#define GET_PORT_NUM(PINNAME)            (uint8_t)(((uint16_t)(PINNAME>>8))&0X00FF)
#define GET_PIN_NUM(PINNAME)             (uint8_t)(((uint16_t)(PINNAME))&0X00FF)

typedef enum {
        PIN_INPUT = 0, /**< GPIO as an input */
        PIN_INPUT_PULL_UP ,/**< GPIO as an input with pull-up */
        PIN_INPUT_PULL_DOWN ,/**< GPIO as an input with pull-down */
        PIN_OUTPUT,/**< GPIO as an (implicitly push-pull) output */
        PIN_OUTPUT_PUSH_PULL,/**< GPIO as an (explicitly push-pull) output */
        PIN_OUTPUT_OPEN_DRAIN/**< GPIO as an open-drain out*/ //pin state can be set and read back
} PinDirection;

#if PIN_VOLTAGE_CHANGE_SUPPORTED
typedef enum {
        PIN_POWER_V33  = 0, /**< V33 (3.3 V) power rail */
        PIN_POWER_POWER_VDD1V8P , /**< VDD1V8P (1.8 V) power rail */
} PinVoltageLevel;
#endif

typedef enum {
        PIN_FUNC_NC = 0XFFFF, /*if function below not exised, set it to 0xffff*/
        PIN_FUNC_GPIO = NRF_GPIO_PIN_NOSENSE, /**< GPIO */
				PIN_FUNC_SENSE_LOW = NRF_GPIO_PIN_SENSE_LOW,
				PIN_FUNC_SENSE_HIGH = NRF_GPIO_PIN_SENSE_HIGH,
} PIN_FUNC;

typedef enum {
        // Not connected
        NC = (int)0xFFFFFFFF,
        P00 = GENERATE_PIN_NAME(0, 0),
        P01,
        P02,
        P03,
        P04,
        P05,
        P06,
        P07,
        P08,
        P09,
        P10,
        P11,
        P12,
        P13,
        P14,
        P15,

        P16,
        P17,
        P18,
        P19,
        P20,
        P21,
        P22,

        P23,
        P24,
        P25,
        P26,
        P27,
        P28,
        P29,
        P30,

        P31,
        P32,
        P33,
        P34,
        P35,
        P36,
        P37,
        P38,
				P_NOT_USED = 0XFF,
				AIN_DIASBLE = 0,
				AIN0,
				AIN1,
				AIN2,
        /*GENERAL PURPOSE pin*/
				GPIO_GPS_RESET = P28,
				GPIO_PPG_ON = P29,
				GPIO_OLED_VPP_ON = P30,
				
				GPIO_GPS_SAFEBOOT = P27,
				GPIO_OLED_RST = P31,
				GPIO_TP_RST = P21,
				GPIO_OLED_A0 = P02,  // Touch key pin index
				GPIO_CHG_IN = AIN2,  // BATERY VOLTAGE SAMPLE PIN
 
        /*retarget uart pin group*/
        GPIO_UART_DBG_TX = P22,
        GPIO_UART_DBG_RX = P20,


				GPIO_VBR_EN = P18,
				GPIO_CHG_IRQ = P16,


        /*spi device cs group*/
        GPIO_SPI_CS_ACC =  P12,
        GPIO_FLASH_CS =  P10,
				GPIO_OLED_CS = P03,  // Touch key pin index
        /*spi bus pin assignment group*/
        GPIO_SPI_SCK = P14,
        GPIO_SPI_MISO = P07,
        GPIO_SPI_MOSI = P17,


        ACC_SENSOR_INT = P09,  // Accelerometer pin index
        GPIO_TOUCH_INT = P22,  // Touch key pin index
				GPIO_RTC_INT = P08,  // Touch key pin index
				GPIO_PPG_INT = P25,
				

        GPIO_TWI_CLK = P05,
        GPIO_TWI_DATA = P06,

        GPIO_VIBRATOR_EN = P18,

        GPIO_TOUCH_PWR_EN = P21,
				
				GPIO_7816_IO = GPIO_UART_DBG_RX,
				GPIO_7816_CLK = GPIO_UART_DBG_TX,
				GPIO_7816_RST = P04,
				GPIO_7816_VCC = P_NOT_USED,//NRF_DRV_SPI_PIN_NOT_USED

} PinName;

typedef enum {
        PullNone = 0,
        PullUp = 1,
        PullDown = 2,
        PullDefault = PullUp
} PinMode;

#ifdef __cplusplus
}
#endif

#endif
