//
//  pedometer.h
//  StepsPlusActivity
//
//


#ifndef __PEDO_HEADER__
#define __PEDO_HEADER__

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#include "standard_types.h"
#include "pedo_api.h"

enum {
    MAIN_LPF_3P6HZ,
    MAIN_LPF_6HZ,
    MAIN_LPF_8HZ,
};

enum {
	PEDO_SENSITIVITY_HIGH,
	PEDO_SENSITIVITY_MEDIUM,
	PEDO_SENSITIVITY_LOW
};

#define g1ValueDef 2048
#define NOMINAL_G g1ValueDef
#define STATIONARY_ENG_DC 4194304 /* header.g1Value*header.g1Value */

#define STATIONARY_ENG_TH  52428 /* STATIONARY_ENG_DC>>6 * 0.8, The energy threshold for detecting stationary period */

#define NOMINAL_G_MAG (I32S)4194304 /*(g1ValueDef*g1ValueDef) */
#define NOMINAL_G_MAG_SHIFT 22
#define _GET_NORM_SH	11

#define PEDO_SPS 50 // Samples per second
#define PEDO_TICK_LENGTH_SHORT 2 // SHORT TERM WINDOW: 4 seonds (2^2)
#define PEDO_TICK_LENGTH_LONG  7 // LONG TERM WINDOW: 28 seconds (2^7)

#define STEP_COUNT_ENTRANCE_TH (I32S) 2097152 /* (0.5*NOMINAL_G_MAG) */

#define CLASSIFIER_RUNNING_MIN_TH (I32S)2097152 /*(0.20*NOMINAL_G_MAG) */

//#define STEP_COUNT_PEAK_DIFF_TH (I32S)754975 /*(0.18*NOMINAL_G_MAG) */
//#define STEP_COUNT_PEAK_DIFF_TH (I32S)641729 /*(0.15*NOMINAL_G_MAG) */
#define STEP_COUNT_PEAK_DIFF_TH (I32S)556165 /*(0.13*NOMINAL_G_MAG) */

//
//  APU:  walking / running / cheating classification thresholds
//
#define CLASSIFIER_WRC_1_TH  (I32S)11324621 /*(2.70*NOMINAL_G_MAG) */
#define CLASSIFIER_WRC_2_TH  (I32S)8598323 /*(2.05*NOMINAL_G_MAG) */

#define CLASSIFIER_WRC_PURE_WALK_TH  (I32S)6291456 /*(1.5*NOMINAL_G_MAG) */

#define CLASSIFIER_CAR_AUP_EXIT_TH (I32U)2222981 /* (0.53*NOMINAL_G_MAG). */
#define AUP_HIST_Q_STEP (41943<<1) /* 0.02*NOMINAL_G_MAG */

#define AUP_HIST_STEPS 50 /* Overall bin number. */
#define STEP_CONSISTENCY_THRESHOLD_MIN 7
#define STEP_CONSISTENCY_THRESHOLD_MAX 30
#define STEP_CONSISTENCY_THRESHOLD_INC 8 /*6*/

#define ADJ_G_STEP_WIN 4


#define CLASSIFICATION_TIME_SPAN 8 /* A 8 seconds span for classification . */
#define CLASSIFIER_STEP_WIN_SZ 32  /* a window of 32 steps for classification */
//#define CLASSIFIER_STEP_WIN_SZ 16  /* a window of 32 steps for classification */
#define CHEATING_POINTS_NORM        (I8U)64
#define CHEATING_POINTS_NORM_SHIFT	6

// Cheating thresholds
#define CHEATING_NORM_SHIFT 2    /* Normalize cheating threshold to 4 of 7. */
#define TH_R_CHEATING_G_MAG (3891-g1ValueDef)   /* (+1.9*2048) *///(3687-g1ValueDef)   /* (+1.8*2048) */
#define TH_W_CHEATING_G_MAG (3482-g1ValueDef)   /* (+1.7*2048) */
#define TL_R_CHEATING_G_MAG (1843)    /* (-0.9*2048) */
#define TL_W_CHEATING_G_MAG (1229)    /* (-0.6*2048) */

#define PACE_CHEATING_TH          4 /* faster than 4 steps per second, declare it as cheating. */
#define KC_CHEATING_W_TH          (35)
#define KC_CHEATING_R_TH          (32)
#define KC_APU_P2P_TH             (I32S) 2097152 /* (0.5*NOMINAL_G_MAG) */

#define AXIS_RATIO_CHEATING_TH_LO    8  // P2P ratio between two axis
#define AXIS_RATIO_CHEATING_TH_HI    10  // P2P ratio between two axis

// Unintentional cheating constrains
enum {
    UC_CONSTRAINS_LOW,
    UC_CONSTRAINS_MED,
    UC_CONSTRAINS_HI
};
#define UC_CONSTRAINS_LEVEL UC_CONSTRAINS_LOW

#define STATIONARY_TIME_LOW_POWER_TH 6000 /* 2 Minutes. */
#define STATIONARY_TIME_TH  150 /* 3 seconds for a stationary period */

#define STATIONARY_TIME_EARLY_TH    65

#define NOISE_STEP_TIME_TH  100 /* 2 second noise threshold to eliminate inconsistent small motion .*/

#define GEST_PROTECTION_PERIOD  500
#define WIN_ADJ_G_MIN   13      /* 13/50 = 0.26 second -> 0.26 seconds for half step. */
#define WIN_ADJ_G_MAX   100     /* 100/50 = 2 seconds -> 2 seconds for half step. */
#if 1
#define DC_A_PRIME_UP_TIME  6
#define DC_A_PRIME_UP_SAMPLES   64 /* 128 samples -> 2.5 seconds DC period. */
#else
#define DC_A_PRIME_UP_TIME  7
#define DC_A_PRIME_UP_SAMPLES   128 /* 128 samples -> 2.5 seconds DC period. */
#endif
#define STEP_WIN_MIN    4
enum {
    STEP_NOT_AVAILABLE = 0,
    STEP_TO_BE_CLASSIFIED,
    STEP_ALREADY_CLASSIFIED,
};
enum {
	NO_CROSSING = 0,
	NEG_TO_POS,
	POS_TO_NEG
};

enum {
    SAMPLERATE_100 =0,
    SAMPLERATE_50
};
enum {
	STATIONARY_NONE = 0,
	STATIONARY_REAL_TRUE,
	STATIONARY_EARLY_TRUE,
	STATIONARY_LOW_POWER_TRUE,
	STATIONARY_END
};

typedef enum {
    CAR_CTX_LOW,
    CAR_CTX_MEDIAN,
    CAR_CTX_HIGH,
} CAR_CERTAINTY;

// Butterworth 4 taps low pass filter
#define LPF_TAPS 4

typedef struct tagLP_FILTER {
    I16S B_NORM_BITS;// B normalization.
	I16S b[LPF_TAPS+1];    // coefficients - feedback filter order
    I16S A_NORM_BITS; // A normalization
	I16S a[LPF_TAPS];		// coefficients - feedforward filter order
} LP_FILTER, *PLP_FILTER;

// Accelerometer Accumulator
typedef struct tagACCELEROMETER_ACC {
	I32S x;
	I32S y;
	I32S z;
	I16U samples;
} ACCELEROMETER_ACC, *PACCELEROMETER_ACC;

// Stationary state
typedef struct tagSTATIONARY_STAT {
	I32S mag_prev;
	I32S mag_curr;
	I32S mag_win[4];
	ACCELEROMETER_ACC acc;
	ACCELEROMETER_ACC init;
	ACCELEROMETER_3D est;
	I32S norm;
	BOOLEAN norm_init;
} STATIONARY_STAT, *PSTATIONARY_STAT;

// Adaptive gravity estimation
typedef struct tagADAPTIVE_G_ESTIMATE {
    BOOLEAN b_update;
    BOOLEAN b_init;

	ACCELEROMETER_ACC acc_one_step;                      // The running accumulator
	ACCELEROMETER_3D est;                                   // G Estimate
	ACCELEROMETER_3D hist_g[ADJ_G_STEP_WIN];                                   // G Estimate
	I32U min_t;                                             // Time stampe on G estimate
	
	ACCELEROMETER_ACC acc_10s;                           // The running accumulator over 10 seconds
	ACCELEROMETER_3D lt_est;                                   // G Estimate
	I32U norm_ts;                                           // Normalization time stamp

} ADAPTIVE_G_ESTIMATE, *PADAPTIVE_G_ESTIMATE;

// A prime up DC state
typedef struct tagDC_A_PRIME_UP {
	I32S acc; // the dc component
	I32S v;   // the latest DC estimate
	I32S cnt;
	I32S prev;
  I32U mag;
} DC_A_PRIME_UP, *PDC_A_PRIME_UP;

// Step count state
typedef struct tagSTEP_COUNT_STAT {
	I32S prev2;     // previous A_prime_up
	I32S prev;     // previous A_prime_up
	I32S curr;     // current A_prime_up
	I32U count;    // overall counts
	I32S p2p_peak;
	I32S p2p_bottom_0;
	I32S p2p_bottom_1;
	I32U prev_g_time;
	I8U p2p_bump_set;
} STEP_COUNT_STAT, *PSTEP_COUNT_STAT;


enum {
    GENERAL_DIR_UP=0,
    GENERAL_DIR_DOWN,
};

#define CHEATING_INTEGRATION_LENGTH 8

typedef struct tagANTI_CHEATING_CTX {

    // Non intentional cheating detection feature
    ACCELEROMETER_ACC a_acc;             // Raw A integration (in processing) within a step
    ACCELEROMETER_ACC a_acc_s;           // Raw A integration (finished) for current step
    ACCELEROMETER_ACC a_acc_s1;          // Raw A integration (finished) for previous step
    ACCELEROMETER_ACC a_acc_s2;          // Raw A integration (finished) for the one before previous step

} ANTI_CHEATING_CTX, *PANTI_CHEATING_CTX;

typedef struct tagUC_CONSTRAINS_TAB {
    I32U a_diff_th;
    I8U step_th;
} UC_CONSTRAINS_TAB, *PUC_CONSTRAINS_TAB;

// Classifier state
typedef struct tagCLASSIFIER_STAT {
    I8U step_stat[CLASSIFIER_STEP_WIN_SZ];   // step classification Status
    MOTION_TYPE step_motion[CLASSIFIER_STEP_WIN_SZ];   // step classification Status
    I32U step_ts[CLASSIFIER_STEP_WIN_SZ];  // Time stamp on each step
    I32U step_a_diff[CLASSIFIER_STEP_WIN_SZ];  // Integrated A difference per step
    I8U step_consistency_th;
    I8U step_idx;
    BOOLEAN step_is_noise;
    
		MOTION_TYPE motion;     // Motion: walking/running and stationary
    I32U step_count;        // Step: step count

    // Walking/running classification
    I32S apu_p2p;           // P2P examination for Walking/Running classification
    I32S apu_min;           // Set of parameters to examine the APU peak consistency
    I32S apu_max;
    I32S w_r_votes[CLASSIFIER_STEP_WIN_SZ];

    I16U apu_pace;
    // step pace check over a span of last 8 steps
    I8U b_pace_cheating;

    I32U stationary_ts; // Exit time of stationary mode, this parameter is used to exclude CAR classification for short walking/running activities

    I8U start_normal_activity; // The flag controlling the normal activity classification, and used to eliminate random steps
    
    I16S car_steps_compensation;      /* CAR step compensation. */
    I16S unknown_steps_compensation; /* unknown step compensation . */

    // Anti-cheating scheme
    ANTI_CHEATING_CTX anti_cheating;

} CLASSIFIER_STAT, *PCLASSIFIER_STAT;

typedef struct tagMAGNITUDE_STAT {
	I32S v;
	I32S prev;
} MAGNITUDE_STAT, *PMAGNITUDE_STAT;

typedef struct tagAPU_HISTOGRAM {
    I32U cdf_20_percentile;  // 20 percentile in a cumulative distribution function
    I32U cdf_80_percentile;  // 80 percentile in a cumulative distribution function
    I32U ts;
    I32U dist_update;
    I16U hist[AUP_HIST_STEPS];// Histogram bins
} APU_HISTORGRAM, *PAPU_HISTOGRAM;

typedef struct tagRAW_ACCE_STAT {
    I32S prev;              // The previous acceleration
    I32S curr;              // The current acceleration
    I32S min_local;         // Local minima
    I8U dir;                // Direction
    I8U num_vibrate;        // Number of vibration
    I8U num_strike;         // Number of strike within a second
    I32S acc_step;          // Accumulation within a step
    I32S min_global;        // Minima within a period of time
    I32S max_global;        // Maxima within a period of time
} RAW_ACCE_STAT, *PRAW_ACCE_STAT;

typedef struct tagACCELEROMETER_INPUT_STAT {

    I8U pair_ratio_lo;              // the maximum of each axis
    I8U pair_ratio_hi;              // the maximum of each axis
    I32U ts;                        // time stamp on second
    I32U min_mag;                   // Minimum magnitude
    I32U max_mag;                   // Minimum magnitude
    I32U acc;
    I8U peak_vibrate_single;
    I32U peak_vibrate_window;
    I32U peak_vibrate_smooth;
    I8U peak_strike_single;
    I8U peak_strike_window;         // the maximum of each axis
    I8U acc_symmetric_window;
    I8U max_acce_idx;

    RAW_ACCE_STAT stat[3];          // Raw acceleration statistics

} ACCELEROMETER_INPUT_STAT, *PACCELEROMETER_INPUT_STAT;

typedef struct tagMOTION_TICK_CTX {
    APU_HISTORGRAM st; // Short-term statistics ( over a 8 seconds window )
    APU_HISTORGRAM lt; // Long-term statistics ( over a 128 seconds window)
    CAR_CERTAINTY car_context_certainty;

    BOOLEAN b_vibrate_mode;
    I32U vibrate_ts;

    ACCELEROMETER_INPUT_STAT raw;

} MOTION_TICK_CTX, *PMOTION_TICK_CTX;

typedef struct tagLPF_HISTORY {
	ACCELEROMETER_3D in[LPF_TAPS];// filter history -- feedback
	ACCELEROMETER_3D out[LPF_TAPS];// filter history -- feed forward
} LPF_HISTORY, *PLPF_HISTORY;

typedef struct tagPEDO_STAT {

	I32U global_time; // global timer

	LPF_HISTORY main_lpf;  // Low pass filtering history

	ACCELEROMETER_3D A;// 3D accelerometer data

	MAGNITUDE_STAT mag; // magnitude of A

	I32S A_prime_up;            // A prime up
	ACCELEROMETER_3D A_prime;   // A prime, A with gravity removed

	STATIONARY_STAT stationary; // stationary state

	ACCELEROMETER_3D g_est; // gravity estimates

	ADAPTIVE_G_ESTIMATE g_adj; // gravity estimates

	STEP_COUNT_STAT step; // step count states

	CLASSIFIER_STAT classifier; // Motion classifier

	MOTION_TICK_CTX tick;  // Statistics per second
	
} PEDO_STAT, *PPEDO_STAT;

#endif
