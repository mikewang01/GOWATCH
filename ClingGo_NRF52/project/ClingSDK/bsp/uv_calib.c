
#ifdef _ENABLE_UV_CALIBRATION_

#include "main.h"

///////////////////////////////////////       msb i2c addr  |  lsb i2c addr  |  alignment
#define SIRPD_ADCHI_IRLED    (collect(buffer,      0x23,             0x22,           0      ))
#define SIRPD_ADCLO_IRLED    (collect(buffer,      0x22,             0x25,           1      ))
#define SIRPD_ADCLO_WHLED    (collect(buffer,      0x24,             0x26,           0      ))
#define VISPD_ADCHI_WHLED    (collect(buffer,      0x26,             0x27,           1      ))
#define VISPD_ADCLO_WHLED    (collect(buffer,      0x28,             0x29,           0      ))
#define LIRPD_ADCHI_IRLED    (collect(buffer,      0x29,             0x2a,           1      ))
#define LED_DRV65            (collect(buffer,      0x2b,             0x2c,           0      ))

static cal_ref_t calref[2] =
{
  {
    FLT_TO_FX20( 4.021290),    // sirpd_adchi_irled
    FLT_TO_FX20(57.528500),    // sirpd_adclo_irled
    FLT_TO_FX20( 2.690010),    // sirpd_adclo_whled
    FLT_TO_FX20( 0.042903),    // vispd_adchi_whled
    FLT_TO_FX20( 0.633435),    // vispd_adclo_whled
    FLT_TO_FX20(23.902900),    // lirpd_adchi_irled
    FLT_TO_FX20(56.889300),    // ledi_65ma
    {0x7B, 0x6B, 0x01, 0x00}   // default ucoef
  },
  {
    FLT_TO_FX20( 2.325484),    // sirpd_adchi_irled
    FLT_TO_FX20(33.541500),    // sirpd_adclo_irled
    FLT_TO_FX20( 1.693750),    // sirpd_adclo_whled
    FLT_TO_FX20( 0.026775),    // vispd_adchi_whled
    FLT_TO_FX20( 0.398443),    // vispd_adclo_whled
    FLT_TO_FX20(12.190900),    // lirpd_adchi_irled
    FLT_TO_FX20(56.558200),    // ledi_65ma
    {0xdb, 0x8f, 0x01, 0x00}   // default ucoef
  }
};

static I32U decode(I32U input)
{
  I32S  exponent, exponent_bias9;
  I32U  mantissa;

  if(input==0) return input;

  exponent_bias9 = (input & 0x0f00) >> 8;
  exponent       = exponent_bias9 - 9;

  mantissa       = input & 0x00ff;         // fraction
  mantissa      |=         0x0100;         // add in integer

  // representation in 12 bit integer, 20 bit fraction 
  mantissa       = mantissa << (12+exponent); 
  return mantissa;
}

static I32U collect(I8U* buffer, I8U msb_addr, I8U lsb_addr, I8U alignment)
{
  I16U value;
  I8U  msb_ind = msb_addr - 0x22;
  I8U  lsb_ind = lsb_addr - 0x22;

  if(alignment == 0) {
    value =  buffer[msb_ind]<<4;
    value += buffer[lsb_ind]>>4;       
  } else {
    value =  buffer[msb_ind]<<8;
    value += buffer[lsb_ind];
    value &= 0x0fff;
  }

  if( (value==0x0fff) || (value==0x0000) ) {
		return FX20_BAD_VALUE;
	} else{
		return decode(value);
	}
}

static I16S find_cal_index(I8U* buffer)
{
  I16S index;
  I8U  size;

  // buffer[6] is the LSB, buffer[7] is the MSB
  index = ( I16S )( buffer[6] + ( (I16U)( buffer[7] ) << 8 ) );

  switch( index ) {
    case -1:  index = 0;             break;
    case -2:  index = 0;             break;
    case -3:  index = 1;             break;
    default:  index = -(4 + index);  break;
  }

  size = sizeof(calref)/sizeof(calref[0]);

  if( index < size ) {
    return  index;
  } else {
    return -1;
  }
}

/***************************************************************************//**
 * @brief
 *   This performs a shift_left function. For convenience, a negative
 *   shift value will shift the value right. Value pointed will be
 *   overwritten.
 ******************************************************************************/
static void shift_left(I32U* value_p, I8S shift)
{
  if(shift > 0) 
    *value_p = *value_p<<shift ;
  else 
    *value_p = *value_p>>(-shift) ;
}

/// @cond DOXYGEN_SHOULD_SKIP_THIS
#define ALIGN_LEFT   1
#define ALIGN_RIGHT -1
/// @endcond
/***************************************************************************//**
 * @brief
 *   Aligns the value pointed by value_p to either the LEFT or RIGHT
 *   the number of shifted bits is returned. The value in value_p is 
 *   overwritten.
 ******************************************************************************/
static I8S align( I32U* value_p, I8S direction )
{
  I8S   local_shift, shift ;
  I32U  mask;
  
  // Check invalid value_p and *value_p, return without shifting if bad.
  if( value_p  == NULL )  return 0;
  if( *value_p == 0 )     return 0;

  // Make sure direction is valid
  switch( direction )
  {
    case ALIGN_LEFT:
      local_shift =  1 ;
      mask  = 0x80000000L;
      break;

    case ALIGN_RIGHT:
      local_shift = -1 ;
      mask  = 0x00000001L;
      break;

    default:
      // Invalid direction, return without shifting
      return 0;
  }

  shift = 0;
  while(1)
  {
    if(*value_p & mask ) break;
    shift++;
    shift_left( value_p, local_shift );
  }
  return shift;
}


/***************************************************************************//**
 * @brief
 *
 *   The fx20_round was designed to perform rounding to however many significant
 *   digits. However, for the factory calibration code, rounding to 16 always is
 *   sufficient. So, the FORCE_ROUND_16 define is provided just in case it would
 *   be necessary to dynamically choose how many bits to round to.
 *   Rounds the uint32_t value pointed by ptr, by the number of bits 
 *   specified by round.
 ******************************************************************************/
static void fx20_round(
  I32U *value_p 
#ifndef FORCE_ROUND_16
  , I8U round 
#endif
)
{
  I8U  shift;
  
#ifdef FORCE_ROUND_16
    // Use the following to force round = 16
    I32U mask1  = 0xffff8000;
    I32U mask2  = 0xffff0000;
    I32U lsb    = 0x00008000;
#else
    // Use the following if you want to routine to be
    // capable of rounding to something other than 16.
    I32U mask1  = ((2<<(round))-1)<<(31-(round));
    I32U mask2  = ((2<<(round-1))-1)<<(31-(round-1));
    I32U lsb    = mask1-mask2;
#endif
  
  shift = align( value_p, ALIGN_LEFT );
  if( ( (*value_p)&mask1 ) == mask1 )
  {
    *value_p = 0x80000000;
    shift -= 1;
  }
  else
  {
    *value_p += lsb;
    *value_p &= mask2;
  }

  shift_left( value_p, -shift );
}

static I32U fx20_divide( operand_t* operand_p ) 
{
  I8S   numerator_sh=0, denominator_sh=0;
  I32U  result;
  I32U* numerator_p;
  I32U* denominator_p;

  if( operand_p == NULL ) return FX20_BAD_VALUE;

  numerator_p   = &operand_p->op1;
  denominator_p = &operand_p->op2;

  if(    (*numerator_p   == FX20_BAD_VALUE) 
      || (*denominator_p == FX20_BAD_VALUE) 
      || (*denominator_p == 0             ) ) return FX20_BAD_VALUE;
  
  fx20_round  ( numerator_p   );
  fx20_round  ( denominator_p );
  numerator_sh   = align ( numerator_p,   ALIGN_LEFT  );
  denominator_sh = align ( denominator_p, ALIGN_RIGHT );

  result = *numerator_p / ( (uint16_t)(*denominator_p) );
  shift_left( &result , 20-numerator_sh-denominator_sh );

  return result;
}

/***************************************************************************//**
 * @brief
 *   Returns a fixed-point (20-bit fraction) after multiplying op1*op2
 ******************************************************************************/
static I32U fx20_multiply(  operand_t* operand_p )
{
  I32U  result;
  I8S   val1_sh, val2_sh;
  I32U* val1_p;
  I32U* val2_p;

  if( operand_p == NULL ) return FX20_BAD_VALUE;

  val1_p = &(operand_p->op1);
  val2_p = &(operand_p->op2);

  fx20_round( val1_p );
  fx20_round( val2_p );

  val1_sh = align( val1_p, ALIGN_RIGHT );
  val2_sh = align( val2_p, ALIGN_RIGHT );


  result = (uint32_t)( ( (uint32_t)(*val1_p) ) * ( (uint32_t)(*val2_p) ) );
  shift_left( &result, -20+val1_sh+val2_sh );

  return result;
}

/***************************************************************************//**
 * @brief
 *   Returns the calibration ratio to be applied to VIS measurements
 ******************************************************************************/
static I32U vispd_correction(I8U* buffer)
{
  operand_t op;
  I32U      result;
  I16S      index = find_cal_index( buffer );

  if( index < 0 ) result = FX20_ONE;

  op.op1 = calref[ index ].vispd_adclo_whled; 
  op.op2 = VISPD_ADCLO_WHLED;
  result = fx20_divide( &op );

  if( result == FX20_BAD_VALUE ) result = FX20_ONE;

  return result;
}


/***************************************************************************//**
 * @brief
 *   Returns the calibration ratio to be applied to IR measurements
 ******************************************************************************/
static I32U irpd_correction(I8U* buffer)
{
  operand_t op;
  I32U      result;
  I16S      index = find_cal_index( buffer );

  if( index < 0 ) result = FX20_ONE;

  // op.op1 = SIRPD_ADCLO_IRLED_REF; op.op2 = SIRPD_ADCLO_IRLED;
  op.op1 = calref[ index ].sirpd_adclo_irled;
  op.op2 = SIRPD_ADCLO_IRLED;
  result = fx20_divide( &op );

  if( result == FX20_BAD_VALUE ) result = FX20_ONE;

  return result;
}

/***************************************************************************//**
 * @brief
 *   Returns the ratio to correlate between x_RANGE=0 and x_RANGE=1
 *   It is typically 14.5, but may have some slight component-to-component 
 *   differences.
 ******************************************************************************/
static I32U adcrange_ratio(I8U* buffer)
{
  operand_t op;
  I32U      result;
  
  op.op1 = SIRPD_ADCLO_IRLED;
	op.op2 = SIRPD_ADCHI_IRLED;
  result = fx20_divide( &op );

  if( result == FX20_BAD_VALUE ) result = FLT_TO_FX20( 14.5 ); 

  return result;
}

/***************************************************************************//**
 * @brief
 *   Returns the ratio to correlate between measurements made from large PD
 *   to measurements made from small PD.
 ******************************************************************************/
static I32U irsize_ratio(I8U* buffer)
{
  operand_t op;
  I32U      result;

  op.op1 = LIRPD_ADCHI_IRLED;
	op.op2 = SIRPD_ADCHI_IRLED;

  result = fx20_divide( &op );
  if( result == FX20_BAD_VALUE ) result = FLT_TO_FX20(  6.0 ); 

  return  result;
}

static I32U ledi_ratio(I8U* buffer)
{
  operand_t op;
  I32U      result;
  I16S      index;

  index = find_cal_index( buffer );
  
  if( index < 0 ) result = FX20_ONE;

  // op.op1 = LED_DRV65_REF; op.op2 = LED_DRV65;
  op.op1 = calref[ index ].ledi_65ma; 
  op.op2 = LED_DRV65;
  result = fx20_divide( &op );

  if( result == FX20_BAD_VALUE ) result = FX20_ONE;

  return result;
}

I16S uv_Get_Cal_Index( I8U* buffer )
{
//I16S retval;
  I8U  response;
	
	do
	{
		uv_Send_Command_NOP();
		uv_read_reg(uv_REGS_RESPONSE, 1, &response);
		if (response!=0) {
			BASE_delay_msec(1);
		}
	} while( response!=0);

  // Retrieve the calibration index
	uv_Send_Command(0x11);
	BASE_delay_msec(1);
	
	uv_read_reg(0x26,    1, (buffer+6));
	uv_read_reg(0x27,    1, (buffer+7));
  return 0;
}

static void uv_Set_UCoef(uv_CAL_S *si1132_cal)
{
	operand_t op;
  I8U       temp;
  I8U       partid;
  I32U      long_temp;
  I32U      vc=FX20_ONE;
  I32U      ic=FX20_ONE;
	
  I8U*      ref_ucoef = si1132_cal->ucoef_p;
  I8U       out_ucoef[4];  

  // retrieve part identification
	uv_read_reg(uv_REGS_PART_ID, 1, &partid);
	
  switch( partid ) {
    case 0x32:
		case 0x45:
		case 0x46:
		case 0x47:
			temp = 1;
		  break;

    default:
			temp = 0;
		  break;
  }
  if( !temp ) return;

  if(si1132_cal != 0)
  {
    if(si1132_cal->vispd_correction > 0) vc = si1132_cal->vispd_correction; 
    if(si1132_cal->irpd_correction  > 0) ic = si1132_cal->irpd_correction;
  }

  op.op1 = ref_ucoef[0] + ((ref_ucoef[1])<<8);
  op.op2 = vc;

  long_temp   = fx20_multiply( &op );
  out_ucoef[0] = (long_temp & 0x00ff);
  out_ucoef[1] = (long_temp & 0xff00)>>8;

  op.op1 = ref_ucoef[2] + (ref_ucoef[3]<<8);
  op.op2 = ic;
  long_temp   = fx20_multiply( &op );
  out_ucoef[2] = (long_temp & 0x00ff);
  out_ucoef[3] = (long_temp & 0xff00)>>8;  

	uv_write_reg(uv_REGS_MEAS_UCOEF0, out_ucoef[0]);
	uv_write_reg(uv_REGS_MEAS_UCOEF1, out_ucoef[1]);
	uv_write_reg(uv_REGS_MEAS_UCOEF2, out_ucoef[2]);
	uv_write_reg(uv_REGS_MEAS_UCOEF3, out_ucoef[3]);
	
	N_SPRINTF("%02x %02x %02x %02x", out_ucoef[0], out_ucoef[1], out_ucoef[2], out_ucoef[3]);
}

#if 0
static uv_CAL_S si1132_cal;
#endif
I16S uv_Get_Calibration(I8U security)
{
	I8S  i;
  I8U  buffer[8];
  I8U  response;
//I16S retval = 0;

  // if requested, check to make sure the interface registers are zero
  // as an indication of a device that has not started any autonomous
  // operation
  if( security == 1 )
  {
		uv_read_reg(uv_REGS_ALS_VIS_DATA0,    1, (buffer+0));
		uv_read_reg(uv_REGS_ALS_VIS_DATA1,    1, (buffer+1));
		uv_read_reg(uv_REGS_ALS_IR_DATA0,     1, (buffer+2));
		uv_read_reg(uv_REGS_ALS_IR_DATA1,     1, (buffer+3));
		uv_read_reg(uv_REGS_AUX_DATA0_UVIDX0, 1, (buffer+4));
		uv_read_reg(uv_REGS_AUX_DATA1_UVIDX1, 1, (buffer+5));
		
    for (i=0; i<6; i++) {
			if ( buffer[i]!=0 )
				return -2;
		}
  }

  // Check to make sure that the device is ready to receive commands
  do 
  {
		uv_Send_Command_NOP();
		uv_read_reg(uv_REGS_RESPONSE, 1, &response);
    if (response != 0) {
      BASE_delay_msec(1);
    }
  } while( response != 0 );

  // Request for the calibration data
	uv_Get_Cal();

  // Wait for the response register to increment
  do
  {
		uv_read_reg(uv_REGS_RESPONSE, 1, &response);
		if (response==0x80) {
			uv_Send_Command_NOP();
			return -3;
		} else if (response&0xFFF0) {
			return -2;
		}
		
		if (response!=1) {
			BASE_delay_msec(1);
		}
	} while (response!=1);

  // Retrieve the 6 bytes from the interface registers
	uv_read_reg(uv_REGS_ALS_VIS_DATA0,    1, (buffer+0));
	uv_read_reg(uv_REGS_ALS_VIS_DATA1,    1, (buffer+1));
	uv_read_reg(uv_REGS_ALS_IR_DATA0,     1, (buffer+2));
	uv_read_reg(uv_REGS_ALS_IR_DATA1,     1, (buffer+3));
	uv_read_reg(uv_REGS_AUX_DATA0_UVIDX0, 1, (buffer+4));
	uv_read_reg(uv_REGS_AUX_DATA1_UVIDX1, 1, (buffer+5));
	
	uv_Get_Cal_Index(buffer);
#if 0
  si1132_cal.ledi_ratio       = ledi_ratio(buffer);
  si1132_cal.vispd_correction = vispd_correction(buffer);
  si1132_cal.irpd_correction  = irpd_correction(buffer);
  si1132_cal.adcrange_ratio   = adcrange_ratio(buffer);
  si1132_cal.ucoef_p          = calref[find_cal_index(buffer)].ucoef;
  si1132_cal.irsize_ratio     = irsize_ratio(buffer);
	
	N_SPRINTF("0x%02x 0x%02x 0x%02x 0x%02x", si1132_cal.ucoef_p[0], si1132_cal.ucoef_p[1], si1132_cal.ucoef_p[2], si1132_cal.ucoef_p[3]);
#endif
	return 0;
}

#endif    // #ifdef _ENABLE_UV_CALIBRATION_

