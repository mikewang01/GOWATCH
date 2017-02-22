//
//  pedometer_api.h
//
//


#ifndef __PEDO_API_HEADER__
#define __PEDO_API_HEADER__
//
// Data format used in API header
//
#ifndef I8U
typedef unsigned char I8U;    /**<  Boolean value */
#endif

#ifndef BOOLEAN
typedef I8U BOOLEAN;    /**<  Boolean value */
#endif

#ifndef I32S
typedef int I32S;
#endif
#ifndef I16U
typedef unsigned short int I16U;
#endif

// 
// Define all the motion types 
//
#ifndef _MOTION_TYPE_H_
#define _MOTION_TYPE_H_
typedef enum  {
	// 0~7 is 3-bit definition of valid acitivity in SoW
  MOTION_UNKNOWN=0,
	MOTION_STATIONARY,
	MOTION_WALKING,
	MOTION_RUNNING,
	MOTION_CHEATING, 
	MOTION_CAR, 
  MOTION_STATIONARY_LOW_POWER,
  //8 and above is the extended definition in meta-layer
	MOTION_UNAVAILABLE,
} MOTION_TYPE ;
#endif // _MOTION_TYPE_H_

//
// Pedometer main output
//
#define PDM_STAT_NONE 0x0
#define PDM_STEP_DETECTED 0x01
#define PDM_MOTION_DETECTED 0x02

//
// Pedometer input
//
typedef struct tagACCELEROMETER_3D {
	I32S x;
	I32S y;
	I32S z;
} ACCELEROMETER_3D;

//
// Create a pedometer and reset the structure
//
void PEDO_create(void);
//
// Release all resources and terminate the pedometer
//
void PEDO_release(void);
//
// Pedometer main processing routine
//
I16U PEDO_main(ACCELEROMETER_3D in);
//
// Check if pedometer is in a "transport" mode
//
BOOLEAN PEDO_is_transport_mode(void);
//
// Get motion type if pedometer detected step or motion
//
MOTION_TYPE PEDO_get_motion_type(void);
//
// Set motion type. It is only usable if user doesn't want pedometer to enter sleep mode.
//
//
void PEDO_set_motion_type(MOTION_TYPE mode);
//
// Return the global buffer used by pedometer. User has option to re-use it for other purpose, e.g., OLED display
//
I8U* PEDO_get_global_buffer(void);
//
// Pedometer get pace for distance and calorie calculation
//
I16U PEDO_get_pace(void);
//
// Set step detection sensitivity
//
void PEDO_set_step_detection_sensitivity(BOOLEAN b_sensitive);

#endif
