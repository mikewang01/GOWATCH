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

/*! \file communication.h
 *
 *  \author M. Arpa
 *
 *  \brief application specific header file - defines the communication 
 * protocol to be used between the as7000 and another controller. E.g.
 * a sensor hub, a display board within a watch.
 *
 *
 */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */

#include "stdint.h"
#include "ppg_comm_constants.h"
/*
 *****************************************************************************
 * DEFINES
 *****************************************************************************
 */

/* define a new error if protocol versions do not match */
#define ERR_PROTOCOL_VERSION_MISMATCH       -8
#define ERR_COM_LOST_SUDDENLY               -9
#define WARNING_LOADER_STILL_ACTIVE         -10
#define ERR_INTERNAL_ERROR                  -11

/* whenever we talk to the as7000 - we use this slave address */
#define HEARTRATE_AS7000_I2C_SLAVE_ADDRESS 0x30
#define COMMUNICATION_PROTOCOL_VERSION_MAJOR    0x01      /* current protocol version */
#define COMMUNICATION_PROTOCOL_VERSION_MINOR    0x03      /* current protocol version */

/* Heartrate application specific register addresses */
/*******************************************************************************
 * The following are the register locations for the Pulse Rate Measurement
 * information */

#define REG_ADDR_PRM_TID                    8
#define REG_ADDR_PRM_STATUS                 9
#define REG_ADDR_PRM_HEARTRATE              10
#define REG_ADDR_PRM_SIGNAL_QUALITY         11
#define REG_ADDR_PRM_CURRENT_MSB            12
#define REG_ADDR_PRM_CURRENT_LSB            13
#define REG_ADDR_PRM_SYNC                   14

/* diagnostic registers */
#define REG_ADDR_PRMALT_TID                 18
#define REG_ADDR_PRMALT_STATUS              19
#define REG_ADDR_PRMALT_HEARTRATE           20
#define REG_ADDR_PRMALT_SIGNAL_QUALITY      21
#define REG_ADDR_PRMALT_CURRENT_MSB         22
#define REG_ADDR_PRMALT_CURRENT_LSB         23
#define REG_ADDR_PRMALT_RAW_VALUE_OFE_MSB   24
#define REG_ADDR_PRMALT_RAW_VALUE_OFE_LSB   25
#define REG_ADDR_PRMALT_RAW_VALUE_TIA_MSB   26
#define REG_ADDR_PRMALT_RAW_VALUE_TIA_LSB   27
#define REG_ADDR_PRMALT_X_AXIS_MSB          28
#define REG_ADDR_PRMALT_X_AXIS_LSB          29
#define REG_ADDR_PRMALT_Y_AXIS_MSB          30
#define REG_ADDR_PRMALT_Y_AXIS_LSB          31
#define REG_ADDR_PRMALT_Z_AXIS_MSB          32
#define REG_ADDR_PRMALT_Z_AXIS_LSB          33
#define REG_ADDR_PRMALT_GSR_MSB             34
#define REG_ADDR_PRMALT_GSR_LSB             35
#define REG_ADDR_PRMALT_NTC_MSB             36
#define REG_ADDR_PRMALT_NTC_LSB             37
#define REG_ADDR_PRMALT_SYNC                38

/* number of bytes in the register block - the sync register itself is not written */
#define REG_PRM_BLOCK_SIZE      (REG_ADDR_PRMALT_SYNC - REG_ADDR_PRMALT_TID + 1)

/*******************************************************************************
 * The following are the register locations for the Pulse Rate Measurement
 * information */
#define REG_ADDR_PRV_TID                    40
#define REG_ADDR_PRV_STATUS                 41
#define REG_ADDR_PRV_PPI_MSB                42
#define REG_ADDR_PRV_PPI_LSB                43
#define REG_ADDR_PRV_PEAK_VALUE_MSB         44
#define REG_ADDR_PRV_PEAK_VALUE_LSB         45
#define REG_ADDR_PRV_SYNC                   46

/* number of bytes in the register block - the sync register itself is not written */
#define REG_PRV_BLOCK_SIZE      (REG_ADDR_PRV_SYNC - REG_ADDR_PRV_TID + 1)

/* defines for errors that the as7000 will report in the status field */

#define HEARTRATE_OK                        (0)         /* status is ok */
#define HEARTRATE_ERR_PARAM                 (1)         /* illegal input or output parameter for algorithm */
#define HEARTRATE_ERR_DATA_LOST             (2)
#define HEARTRATE_ERR_ACCELEROMETER         (3)         /* either no accelerometer or not working */


/* external accelerometer interrupt line is RA4 */
#define AS7000_EXT_ACC_MASK     ( 1 << 4 )
#define AS7000_EXT_ACC_REQUEST  0
#define AS7000_EXT_ACC_IDLE     (AS7000_EXT_ACC_MASK)


/*
 *****************************************************************************
 * TYPES
 *****************************************************************************
 */

typedef struct _as7000Info
{
    uint8_t protocolRevMajor;
    uint8_t protocolRevMinor;
    uint8_t swRevMsb;
    uint8_t swRevLsb;
    uint8_t applicationId;
    uint8_t hwRevision;
    uint8_t reserved0;
    uint8_t reserved1;
} as7000Info_t;

#define SDK_REV_MAJOR(info)      ((info).swRevMsb >> 6)
#define SDK_REV_MINOR(info)      ((info).swRevMsb & 0x3F)
#define SDK_REV_BUILD(info)      ((info).swRevLsb)

typedef struct __attribute__ ((packed)) _pulseRateData
{
    uint8_t transactionId;
    uint8_t status;
    uint8_t heartRate;
    uint8_t signalQuality;
    uint16_t ledCurrent;
    uint16_t ofe;
    uint16_t tia;
    int16_t xAxis;
    int16_t yAxis;
    int16_t zAxis;
    uint16_t gsr;
    uint16_t ntc;
    uint8_t sync;
} pulseRateData_t;

typedef struct __attribute__ ((packed)) _pulseVariabilityData
{
    uint8_t transactionId;
    uint8_t status;
    uint16_t ppi;
    uint16_t peakValue;
    uint8_t sync;
} pulseVariabilityData_t;

typedef struct _accelerometerData
{
    int16_t xAxis;
    int16_t yAxis;
    int16_t zAxis;
} accelerometerData;


/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */


/*!
 *****************************************************************************
 * \brief Initialises all internal variables of the communication protocol
 * and initialises the device driver used for communication.
 * \returns ERR_NONE when initialisation of i2c was successfull
 * \returns ERR_IO when i2c fails
 *****************************************************************************
 */
int8_t comInitialise( void );

/*!
 *****************************************************************************
 * \brief Opens the communication channel and tries to read out the first
 * 8 bytes of the i2c register map of the as7000.
 * \returns ERR_NONE when the 8 bytes of the as7000 were read successfully
 * and the protocol versions do match
 * \returns ERR_IO when the i2c communication fails
 * \returns ERR_PROTOCOL_VERSION_MISMATCH when i2c fails
 * \returns WARNING_LOADER_STILL_ACTIVE when the loader is still active - try
 * later again
 *****************************************************************************
 */
int8_t comOpen( as7000Info_t *infoRecord );

/*!
 *****************************************************************************
 * \brief Resets and closes
 *****************************************************************************
 */
void comClose( void );

/*!
 *****************************************************************************
 * \brief Reads in the pulse rate measurement data from the as7000
 * \returns 0 when no new data is availabe
 * \returns 1 if new data is available
 * \returns -1 when communication is broken == AS7000 does not ack on i2c line
 *****************************************************************************
 */
int8_t comGetPrmData( pulseRateData_t * data );

/*!
 *****************************************************************************
 * \brief Reads in the pulse rate variability data from the as7000
 * \returns 0 when no new data is availabe
 * \returns 1 if new data is available
 * \returns -1 when communication is broken == AS7000 does not ack on i2c line
 *****************************************************************************
 */
int8_t comGetPrvData( pulseVariabilityData_t * data );

/*!
 *****************************************************************************
 * \brief Write the accelerometer data to the AS7000.
 * \returns ERR_NONE when data has been successfully transmitted
 * \returns ERR_IO when communication is broken == AS7000 does not ack on i2c line
 *****************************************************************************
 */
int8_t comSetData ( int16_t xAxis, int16_t yAxis, int16_t zAxis );

/*!
 *****************************************************************************
 * \brief function to start the as7000 intel hex file loader
 * \returns 0 when successfully requested start of the loader - call
 * function comIsLoaderAppRunning to find out if it really started
 * \returns <>0 when start failed i.e. as7000 naked the request
 *****************************************************************************
 */
int8_t comEnterLoader( void );

/*!
 *****************************************************************************
 * \brief function to select application on the AS7000
 * \returns 0 when communication was successful
 * \returns <>0 when communication failed
 *****************************************************************************
 */
int8_t comSelectApplication (uint8_t appId);

/*!
 *****************************************************************************
 * \brief function to find out if the loader started
 * Note: do not call this function after you have sent one (or more) intel hex
   records - in this case the register map field app-id is re-used for
 * different purposes
 * \param loaderVersionNumber - inserts the loaders major and minor 8-bit
 * version number into this parameter - is only valid if function returns >0.
 * \returns >0 when successfully started the loader
 * \returns 0 when start failed == there is no loader
 * \returns <0 when loader is currently busy - try again
 *****************************************************************************
 */
int8_t comIsLoaderAppRunning( uint16_t * loaderVersionNumber );

/*!
 *****************************************************************************
 * \brief function checks if the loader at the as7000 is ready to receive new data
 * \returns 0 when loader is ready
 * \returns <>0 when loader is busy, or nor responding (is also considered busy)
 *****************************************************************************
 */
int8_t comIsLoaderReady( void );

/*!
 *****************************************************************************
 * \brief function takes one intel hex record to be transmitted to the loader
 * \param toTx - number of bytes to transmit via i2c
 * \param txData - data to be transmitted via i2c
 * \returns 0 when record could be sent (next time use new record)
 * \returns <>0 when record could not be sent due to the as7000 i2c slave
 * not responding anymore - could be due to a power down on as7000
 *****************************************************************************
 */
int8_t comSendIntelHex( uint16_t toTx, const uint8_t * txData );


#endif /* COMMUNICATION_H */
