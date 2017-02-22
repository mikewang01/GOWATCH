/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
/*
 *      PROJECT:   AS7000 firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file communication_constants.h
 *
 *  \author M. Arpa
 *
 *  \brief contains all the constants that are common to all communication
 * protocol versions.
 *
 */

#ifndef COMMUNICATION_CONSTANTS_H
#define COMMUNICATION_CONSTANTS_H

/*
 *****************************************************************************
 * DEFINES
 *****************************************************************************
 */


/* Application IDs that are already in use or reserved */
#define RESERVED_APPLICATION_ID                     0x00        /* do not use */
#define LOADER_APPLICATION_ID                       0x01        /* unique ID for the loader */
#define HEARTRATE_APPLICATION_ID                    0x02        /* unique ID for heartrate */
#define PULSRATE_VARIABILITY_APPLICATION_ID         0x04        /* unique ID for PRV/HRV */
#define GALVANIC_SKIN_RESISTIVITY_APPLICATION_ID    0x08
#define TEMPERATURE_SENSOR_APPLICATION_ID           0x10


/* these addresses are the same in every as7000 application */
#define REG_ADDR_PROTOCOL_VERSION_MAJOR     0
#define REG_ADDR_PROTOCOL_VERSION_MINOR     1
#define REG_ADDR_SW_VERSION_MAJOR           2
#define REG_ADDR_SW_VERSION_MINOR           3
#define REG_ADDR_APPLICATION_ID             4
#define REG_ADDR_HW_VERSION                 5
#define REG_ADDR_RESERVED0                  6
#define REG_ADDR_RESERVED1                  7

#define REG_INFO_BLOCK_SIZE                 ( REG_ADDR_RESERVED1 )


#define COMMUNICATION_16BIT_MSB( x )        ( (uint8_t)( ( ( (uint16_t)(x) ) >> 8 ) & 0xFF ) )
#define COMMUNICATION_16BIT_LSB( x )        ( (uint8_t)(   ( (uint16_t)(x) )        & 0xFF ) )

#define AS7000_APP_HRM_ID                   2
#define AS7000_APP_HRV_ID                   4
#define REG_ADDR_LOADER_STATUS              0
#define LOADER_STATUS_READY                 0
#define LOADER_STATUS_BUSY                  1

#define MAX_INTEL_HEX_RECORD_SIZE           256 /* could also be more - up to approximately 256 */
/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */



#endif /* COMMUNICATION_CONSTANTS_H */
