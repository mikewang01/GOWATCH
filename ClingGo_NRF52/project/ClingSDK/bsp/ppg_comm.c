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
 *      PROJECT:   AS7000 heartrate application
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file communication.c
 *
 *  \author M. Arpa
 *
 *  \brief implementation of the communication protocol
 *
 */

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */

#include "ppg_comm.h"
#include "errno.h"
#include  "main.h"
#include "nrf_error.h"
#include "i2cdev_hal.h"
#include "clingtask.h"
/*
 *****************************************************************************
 * DEFINES
 *****************************************************************************
 */

/* Allow incorrect structure sizes to be captured at compile time */
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))
#define  ERR_NONE   NRF_SUCCESS
#define  ERR_PARAM  NRF_ERROR_INVALID_PARAM
#define	 ERR_IO			NRF_ERROR_INTERNAL
#define  LOG // Y_SPRINTF
/* The pic uses data little endian, where the registers on the AS7000
 * are big endian, provide a mechanism to swap the bytes */
#define byteSwap(var)                                               \
{                                                                   \
    uint8_t *ptr = (uint8_t *) &var;                                          \
    uint8_t tmp;                                                         \
    tmp = *ptr;                                                     \
    *ptr = *(ptr + 1);                                              \
    *(ptr + 1) = tmp;                                               \
}
 
#define COM_GET_16_BIT_UNSIGNED( data, offset )                    \
    ( ( ( (int16_t)( (data)[ (offset)     ] ) ) << 8 )                 \
    |   ( (int16_t)( (data)[ (offset) + 1 ] ) )                        \
    )
/*
 *****************************************************************************
 * VARIABLES
 *****************************************************************************
 */
 
/* configuration to talk to the as7000 through i2c */
//const i2cConfig_t as7000i2cConfig =
//{ I2C1_MODULE
//, I2C_ADDRESS_MODE_7_BIT
//, I2C_SCK_400KHZ
//, 0 /* no custom speed */
//};

/* ---- loader request records ---- */

/* request to enter loader application - i2c tx only */
const uint8_t as7000ReqLoader[ ] =
{ ( HEARTRATE_AS7000_I2C_SLAVE_ADDRESS << 1 )
, REG_ADDR_APPLICATION_ID /* write new app id */
, LOADER_APPLICATION_ID
};

/* data to be sent to read back the loader status - i2c tx data to rx status */
const uint8_t as7000ReqLoaderStatus[ ] =
{ ( HEARTRATE_AS7000_I2C_SLAVE_ADDRESS << 1 )
, REG_ADDR_LOADER_STATUS /* reg 0 contains status */
, ( HEARTRATE_AS7000_I2C_SLAVE_ADDRESS << 1 ) | 1
};

/* array to hold the status response receive with i2c rx */
uint8_t as7000ResLoaderStatus [ 1 ]; /* just 1 byte is the status */

/* request to send loader an intel hex record, add 2 for slave address and
 * register address. This record is filled at run-time. */
uint8_t as7000IntelHexRecord[ 2 + MAX_INTEL_HEX_RECORD_SIZE + 1 ];
/* +1 for device address +1 for register address, +1 for zero-terminator for logging   */

/* ---- standard communication with as7000 board == no loader communication */

/* ---- info record ---- */

/* tx to read out all ids */
const uint8_t as7000ReqIDs[ ] =
{ ( HEARTRATE_AS7000_I2C_SLAVE_ADDRESS << 1 )
, REG_ADDR_PROTOCOL_VERSION_MAJOR /* fist reg to read */
, ( HEARTRATE_AS7000_I2C_SLAVE_ADDRESS << 1 ) | 1
};

/*******************************************************************************
 * I2C command sequence to read either the pulse rate measurement or variability
 * data from the AS7000
 */
const uint8_t as7000ReqPrmData[ ] =
{ ( HEARTRATE_AS7000_I2C_SLAVE_ADDRESS << 1 )
, REG_ADDR_PRMALT_TID /* first reg to read */
, ( HEARTRATE_AS7000_I2C_SLAVE_ADDRESS << 1 ) | 1   /* Slave address for the register reads */
};

const uint8_t as7000ReqPrvData[ ] =
{ ( HEARTRATE_AS7000_I2C_SLAVE_ADDRESS << 1 )
, REG_ADDR_PRV_TID /* first reg to read */
, ( HEARTRATE_AS7000_I2C_SLAVE_ADDRESS << 1 ) | 1   /* Slave address for the register reads */
};

/* only used when using an external accelerometer
 * ---- acc data ---- */

/* tx to write all acc data */
uint8_t as7000ReqAcc[ 2 + 6 ];               /* 2 bytes for i2c device write address and register address */
                                        /* 6 bytes for the 3 accelerometer - 10bit values */
/* tx to write all acc data */
/* ------------------- local variables -------------------------- */

/* we save the last read TID - so that we know when we get new data */
int16_t comLastReadPrmTid = 0x100;;
int16_t comLastReadPrvTid =  0x100;;

/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */

 /* function gets the accelerometer data from the accelerometer IC */
extern void getAccelerometerData( accelerometerData * acc );

/* change notification interrupt - is called when the corresponding pin changes
 * its state (edle sensitive) - so we get an interrupt when the as7000 request
 * the accelerometer data
 */
void  _CNInterrupt ( void )
{
    int16_t value = 0;// mPORTAReadBit( AS7000_EXT_ACC_MASK );   /* read out value of request pin (high/low) */
    accelerometerData accData;
    if ( value ==  AS7000_EXT_ACC_REQUEST )
    {
        LOG( "AS7000 Ext Acc Request ");
        //getAccelerometerData( &accData );       /* get data from the accelerometer IC */
        comSetData( accData.xAxis, accData.yAxis, accData.zAxis );  /* transmit data to as7000 */
    }
    else
    {
        LOG( "AS7000 Ext Acc Idle\n");      /* received data confirmed by as7000 - nothing to do */
    }
}


static int8_t  i2cRxTx(size_t tx_size, const uint8_t *tx_buf, size_t rx_size, uint8_t *rx_buf, uint8_t start_cond, uint8_t stop_cond)
{
		int err_code = NRF_SUCCESS;
		uint8_t send_buffer[32];
		memcpy(send_buffer, tx_buf, tx_size);
	  CLASS(I2CDevHal) *p_instance = I2CDevHal_get_instance();
	  i2c_dev_handle_t t = p_instance->ppg_open(p_instance);
		//int err_code = p_instance->write(p_instance, t, send_buf, txbytes);
		//err_code = p_instance->read(p_instance, t, pRegVal, rxbytes);
		/*sicnce  the first bytes of buffer is stored as saleve adress which we don't need in lower driver, bufer[1] is fine to be passed to lower layer*/
	  err_code = p_instance->write_read(p_instance, t, (uint8_t *)&send_buffer[1], tx_size - 1, rx_buf, rx_size);
		if(err_code == 0){
				err_code = ERR_NONE;
		}else{
				err_code = ERR_IO;
		}
		p_instance->close(p_instance);
		return err_code;
}

								
								
int8_t comInitialise ( void )
{
    comLastReadPrmTid = 0x100;              /* Ensure the value is not possible on the AS7000 */
    comLastReadPrvTid = 0x100;              /* Ensure the value is not possible on the AS7000 */

    /* configure i2c to be able to talk to the as7000 through i2c */
    return 1;// i2cInitialize( systemClockGetFcy(), &as7000i2cConfig, 0);
}

int8_t comOpen ( as7000Info_t *infoRecord )
{
    int8_t result = ERR_PARAM;

    if ( infoRecord )
    {
        memset(infoRecord, 0, sizeof(as7000Info_t));

        result = i2cRxTx
                    ( sizeof( as7000ReqIDs )    /* number of bytes to tx == i2c write */
                    , as7000ReqIDs              /* pointer to buffer to be txed */
                    , sizeof( as7000Info_t )    /* number of bytes to rx == i2c read */
                    , (uint8_t *) infoRecord         /* pointer to buffer to store the rxed data */
                    , 1                         /* generate i2c start condition */
                    , 1                         /* generate i2c stop condition */
                    );

        if ( result == ERR_NONE )
        {
            if (  ( infoRecord->protocolRevMajor == COMMUNICATION_PROTOCOL_VERSION_MAJOR )
               && ( infoRecord->protocolRevMinor == COMMUNICATION_PROTOCOL_VERSION_MINOR )
               )
            { /* protocol versions match -> continue */
                result = ERR_NONE;
            }
            else
            {
                if ( infoRecord->applicationId == LOADER_APPLICATION_ID )
                {
                    result = WARNING_LOADER_STILL_ACTIVE;   // TODO Make sure this case is handled correctly
                    LOG( "Loader still active - wait longer for heartrate application\n" );
                }
                else
                {
                    result = ERR_PROTOCOL_VERSION_MISMATCH;
                    LOG( "Version mismatch %d.%d != %d %d\n"
                        , (int)(infoRecord->protocolRevMajor)
                        , (int)(infoRecord->protocolRevMinor)
                        , (int)(COMMUNICATION_PROTOCOL_VERSION_MAJOR)
                        , (int)(COMMUNICATION_PROTOCOL_VERSION_MINOR)
                        );
                }
            }
        }
        else
        {
            LOG( "Com Open failed with %d\n ", result );
        }
    }

    return result;
}

void comClose ( void )
{
    comInitialise( );
}

/*******************************************************************************
 * The following return will get the Pulse Rate Measurement information from the
 * AS7000 */
int8_t comGetPrmData ( pulseRateData_t * data )
{
    BUILD_BUG_ON(REG_PRM_BLOCK_SIZE != sizeof(pulseRateData_t));

    int8_t result = i2cRxTx
                ( sizeof( as7000ReqPrmData )/* number of bytes to tx == i2c write */
                , as7000ReqPrmData          /* pointer to buffer to be txed */
                , sizeof( pulseRateData_t ) /* number of bytes to rx == i2c read */
                , (uint8_t *)data                /* pointer to buffer to store the rxed data */
                , 1                         /* generate i2c start condition */
                , 1                         /* generate i2c stop condition */
                );

    if ( result == ERR_NONE )
    {
        if ( data->transactionId != comLastReadPrmTid )
        { /* have new data */
            comLastReadPrmTid = data->transactionId;
            byteSwap(data->ledCurrent);
            byteSwap(data->ofe);
            byteSwap(data->tia);
            byteSwap(data->xAxis);
            byteSwap(data->yAxis);
            byteSwap(data->zAxis);
            byteSwap(data->gsr);
            byteSwap(data->ntc);

            LOG( "Data %d %d %d %d %d %d\n", data->transactionId, data->heartRate, data->ofe, data->tia, data->gsr, data->ntc );
            return 1; /* indicate new data */
        }

        LOG( "No new\n" ); /* did not get new data */
        return 0;/* no new data */
    }

    LOG( "Com lost\n" );
    return -1;
}

/*******************************************************************************
 * The following return will get the Pulse Rate Variability information from
 * the * AS7000 */
int8_t comGetPrvData ( pulseVariabilityData_t * data )
{
    BUILD_BUG_ON(REG_PRV_BLOCK_SIZE != sizeof(pulseVariabilityData_t));

    int8_t result = i2cRxTx
                ( sizeof( as7000ReqPrvData )/* number of bytes to tx == i2c write */
                , as7000ReqPrvData          /* pointer to buffer to be txed */
                , sizeof( pulseVariabilityData_t )/* number of bytes to rx == i2c read */
                , (uint8_t *) data               /* pointer to buffer to store the rxed data */
                , 1                         /* generate i2c start condition */
                , 1                         /* generate i2c stop condition */
                );

    if ( result == ERR_NONE )
    {
        if ( data->transactionId != comLastReadPrvTid )
        { /* have new data */
            comLastReadPrvTid = data->transactionId;
            byteSwap(data->peakValue);
            byteSwap(data->ppi);

            LOG( "Data %d %d %d \n", data->transactionId, data->ppi, data->peakValue );
            return 1; /* indicate new data */
        }

        LOG( "No new\n" ); /* did not get new data */
        return 0;/* no new data */
    }

    LOG( "Com lost\n" );
    return -1;
}

/*******************************************************************************
 *  Test routine to send accelerometer data to the AS7000 */
int8_t comSetData ( int16_t xAxis, int16_t yAxis, int16_t zAxis )
{
    int8_t result;

    as7000ReqAcc[ 0 ] = ( HEARTRATE_AS7000_I2C_SLAVE_ADDRESS << 1 );    /* i2c device address write */
    as7000ReqAcc[ 1 ] =  REG_ADDR_PRMALT_X_AXIS_MSB;                       /* first reg address to write */
    as7000ReqAcc[ 2 + REG_ADDR_PRMALT_X_AXIS_MSB - REG_ADDR_PRMALT_X_AXIS_MSB ] = COMMUNICATION_16BIT_MSB( xAxis );   /* data to write to regs */
    as7000ReqAcc[ 2 + REG_ADDR_PRMALT_X_AXIS_LSB - REG_ADDR_PRMALT_X_AXIS_MSB ] = COMMUNICATION_16BIT_LSB( xAxis );
    as7000ReqAcc[ 2 + REG_ADDR_PRMALT_Y_AXIS_MSB - REG_ADDR_PRMALT_X_AXIS_MSB ] = COMMUNICATION_16BIT_MSB( yAxis );
    as7000ReqAcc[ 2 + REG_ADDR_PRMALT_Y_AXIS_LSB - REG_ADDR_PRMALT_X_AXIS_MSB ] = COMMUNICATION_16BIT_LSB( yAxis );
    as7000ReqAcc[ 2 + REG_ADDR_PRMALT_Z_AXIS_MSB - REG_ADDR_PRMALT_X_AXIS_MSB ] = COMMUNICATION_16BIT_MSB( zAxis );
    as7000ReqAcc[ 2 + REG_ADDR_PRMALT_Z_AXIS_LSB - REG_ADDR_PRMALT_X_AXIS_MSB ] = COMMUNICATION_16BIT_LSB( zAxis );
    result = i2cRxTx
                ( sizeof( as7000ReqAcc )    /* number of bytes to tx == i2c write */
                , as7000ReqAcc              /* pointer to buffer to be txed */
                , 0                         /* nothing to rx */
                , 0                         /* no rx buffer as nothing to rx */
                , 1                         /* generate i2c start condition */
                , 1                         /* generate i2c stop condition */
                );
    if ( result == ERR_NONE )
    {
        LOG( "txed %d %d %d\n", xAxis, yAxis, zAxis ); /* txed new data */
    }
    else
    {
        LOG( "failed\n" );
    }
    return result;
}

int8_t comEnterLoader ( void )
{
    int8_t result = i2cRxTx
                    ( sizeof( as7000ReqLoader ) /* number of bytes to tx == i2c write */
                    , as7000ReqLoader           /* pointer to buffer to be txed */
                    , 0                         /* nothing to rx */
                    , 0                         /* no rx buffer as nothing to rx */
                    , 1                         /* generate i2c start condition */
                    , 1                         /* generate i2c stop condition */
                    );
    if ( result == ERR_NONE )
    {
        LOG( "EL-Req done\n" );
    }
    else
    {
        LOG( "EL-Req failed\n" );
    }
    return result;
}

int8_t comSelectApplication (uint8_t appId)
{
    uint8_t msg[3] =
        { ( HEARTRATE_AS7000_I2C_SLAVE_ADDRESS << 1 )
        , REG_ADDR_APPLICATION_ID /* write new app id */
        , LOADER_APPLICATION_ID
        };

    msg[2] = appId;

    int8_t result = i2cRxTx
                    ( sizeof(msg)               /* number of bytes to tx == i2c write */
                    , msg                       /* pointer to buffer to be txed */
                    , 0                         /* nothing to rx */
                    , 0                         /* no rx buffer as nothing to rx */
                    , 1                         /* generate i2c start condition */
                    , 1                         /* generate i2c stop condition */
                    );
    if ( result == ERR_NONE )
    {
        LOG( "App selection done new app=%d\n", appId );
    }
    else
    {
        LOG( "App selection failed\n" );
    }
    return result;
}

/* do not call this function after you have sent one (or more) intel hex
   records - in this case the register map field app-id is re-used for
 * different purposes */
int8_t comIsLoaderAppRunning ( uint16_t * loaderVersionNumber )
{
    /* array to rx all ids */
    uint8_t as7000ResIDs [ REG_INFO_BLOCK_SIZE + 1 ]; /* +1 to convert from index of last element to size */

    /* ensure the memory is initialized in case i2cRxTx fails */
    memset(as7000ResIDs, 0, sizeof(as7000ResIDs));

    int8_t result = i2cRxTx
                ( sizeof( as7000ReqIDs )    /* number of bytes to tx == i2c write */
                , as7000ReqIDs              /* pointer to buffer to be txed */
                , sizeof( as7000ResIDs )    /* number of bytes to rx == i2c read */
                , as7000ResIDs              /* pointer to buffer to store the rxed data */
                , 1                         /* generate i2c start condition */
                , 1                         /* generate i2c stop condition */
                );
    if ( result == ERR_NONE )
    {
        if ( as7000ResIDs[ REG_ADDR_APPLICATION_ID ] > LOADER_APPLICATION_ID )
        { /* application switched back to its own id */
            LOG( "Loader not available\n" );
            return 0;
        }
        else
        {
            LOG( "Loader version = %d.%d\n ", as7000ResIDs[ REG_ADDR_SW_VERSION_MAJOR ], as7000ResIDs[ REG_ADDR_SW_VERSION_MINOR ] );
            if ( loaderVersionNumber )
            {
                int16_t temp = as7000ResIDs[ REG_ADDR_SW_VERSION_MAJOR ];
                temp <<= 8;
                temp |= as7000ResIDs[ REG_ADDR_SW_VERSION_MINOR ];/* return loader version number */
                 *loaderVersionNumber = temp;
            }
            return 1;
        }
    }
    /* if either there is no i2c talk possible - loader currently does reboot,
       or the loader has rebooted and cleared the register map, you should re-call this function */
    return -1; /* try later again */
}


int8_t comIsLoaderReady ( void )
{
    int8_t result = i2cRxTx
                    ( sizeof( as7000ReqLoaderStatus )   /* number of bytes to tx == i2c write */
                    , as7000ReqLoaderStatus             /* pointer to buffer to be txed */
                    , sizeof( as7000ResLoaderStatus )   /* number of bytes to rx == i2c read */
                    , as7000ResLoaderStatus             /* pointer to buffer to store the rxed data */
                    , 1                                 /* generate i2c start condition */
                    , 1                                 /* generate i2c stop condition */
                    );
    if ( result == ERR_NONE )
    {
        LOG( "LR(%d)\n", (int)(as7000ResLoaderStatus[ 0 ]) );
        return as7000ResLoaderStatus[ 0 ];  /* if i2c slave acked, than we send back the status == 0 == loader ready or <>0 == loader busy */
    }
    LOG( "LR(FF)\n" );
    return result; /* if the i2c slave naked, than the loader is not ready yet <>0 value is sent back */
}


int8_t comSendIntelHex ( uint16_t toTx, const uint8_t * txData )
{
    as7000IntelHexRecord[ 0 ] = HEARTRATE_AS7000_I2C_SLAVE_ADDRESS << 1; /* i2c write */
    as7000IntelHexRecord[ 1 ] = 0;                                       /* write to register address 0 */
    if ( toTx < MAX_INTEL_HEX_RECORD_SIZE )
    {
        int16_t i;
        for ( i = 0; i < toTx; i++ )
        {
            as7000IntelHexRecord[ 2 + i ] = txData[ i ];
        }
        as7000IntelHexRecord[ 2 + toTx ] = '\0'; /* make it zero-terminated  - for logging only */
        LOG( (const char*)&(as7000IntelHexRecord[ 2 ]) );
        
        return i2cRxTx
                    ( toTx + 2                  /* number of bytes to tx == i2c write */
                    , as7000IntelHexRecord      /* pointer to buffer to be txed */
                    , 0                         /* nothing to rx */
                    , 0                         /* no rx buffer as nothing to rx */
                    , 1                         /* generate i2c start condition */
                    , 1                         /* generate i2c stop condition */
                    );

    }
    return ERR_IO;  /* record too big*/
}
