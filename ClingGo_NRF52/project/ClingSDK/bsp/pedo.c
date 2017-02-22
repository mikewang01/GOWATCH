/*
 *  pedometer.c
 *
 */

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "pedo.h"
#include "base.h"
#include "standards.h"

#include "dbg.h"

PEDO_STAT    gPDM;

// low pass filter coefficient table
const LP_FILTER lpf_coeff[3] = {
    // Sampling rate: 50 Hz, 3.6Hz
	{21, {3238, 12951, 19426, 12951, 3238}, 13, {-23121, 25499, -12849, 2482}},
    // Sampling rate: 50 Hz, 6 Hz
    {19, {4674, 18695, 28042, 18695, 4674}, 13, {-16780, 15088, -6410, 1079}},
    // Sampling rate: 50 Hz, 8 Hz
    {17, {2998, 11991, 17986, 11991, 2998}, 14, {-23134, 18395, -6686, 1036}},
};

#define CONSTRAINS_STEP_HI_TH 6
#define CONSTRAINS_DIFF_HI_TH 55000
#define NOISE_STEP_CLEANUP_HI_TH 75
#define CONSISTENT_STEP_HI_TH 2

#define CONSTRAINS_STEP_ME_TH 8
#define CONSTRAINS_DIFF_ME_TH 42000
#define NOISE_STEP_CLEANUP_ME_TH 50
#define CONSISTENT_STEP_ME_TH 5

#define CONSTRAINS_STEP_LO_TH 9
#define CONSTRAINS_DIFF_LO_TH 33000
#define NOISE_STEP_CLEANUP_LO_TH 50
#define CONSISTENT_STEP_LO_TH 7


#define CONSTRAINS_SLEEP_STEP_TH 10
#define CONSTRAINS_SLEEP_DIFF_TH 34000

static I32U pedo_constrain_diff_th;
static I8U  pedo_constrain_step_th;
static I8U  pedo_noise_cleanup_th;
static I8U pedo_constrain_consistent_step_th;
// 
// Argument -
//
//   Input - 
//         acce_sps: accelerometer sampling rate
//         classification latency:  classification latency in accelerometer samples
//
//                                  e.g., 50: 1 second latency for 50 Hz sampling rate
//                                        200: 4 seconds lantecy for 50 Hz sampling rate
//
//   Output -
void PEDO_create()
{
	// Clear whatever exists in gPDM.
	// reset the timer
	memset(&gPDM, 0, sizeof(PEDO_STAT));

	// Initialize a large pace for anti-cheating
	gPDM.classifier.apu_pace = PEDO_SPS << 1;

	// Initialize the tick parameters for CAR classification
	gPDM.tick.car_context_certainty = CAR_CTX_LOW; // CAR context certainty - low

	// No motin available
	gPDM.classifier.motion = MOTION_UNAVAILABLE;

	// Consistency check step threshold
	gPDM.classifier.step_consistency_th = STEP_CONSISTENCY_THRESHOLD_MIN;
}

void PEDO_release()
{
	// Clear whatever exists in gPDM.
	memset(&gPDM, 0, sizeof(PEDO_STAT));
    
}

static ACCELEROMETER_3D _lpf(ACCELEROMETER_3D in, LP_FILTER *flt, LPF_HISTORY *hist)
{
	I16S i;
    ACCELEROMETER_3D *hist_i = hist->in;
	ACCELEROMETER_3D *hist_o = hist->out;
	I32S o_b_x, o_b_y, o_b_z;
	I32S o_a_x, o_a_y, o_a_z;
	I32S o_x, o_y, o_z;
	ACCELEROMETER_3D A;// 3D accelerometer data
	
	//
 	// feedforward filter
	//
	// Coefficients normalized with 2^28
	//
	o_b_x = (I32S)(flt->b[0] * in.x);
	o_b_y = (I32S)(flt->b[0] * in.y);
	o_b_z = (I32S)(flt->b[0] * in.z);
	for (i = 0; i < LPF_TAPS; i++) {
		o_b_x += flt->b[i+1] * hist_i[i].x;
		o_b_y += flt->b[i+1] * hist_i[i].y;
		o_b_z += flt->b[i+1] * hist_i[i].z;
	}
	
	// feedback filter
	//
	// Coefficients normalized with 2^9
	//
	o_a_x = o_a_y = o_a_z = 0;
	for (i = 0; i < LPF_TAPS; i++) {
		o_a_x += flt->a[i] * hist_o[i].x;
		o_a_y += flt->a[i] * hist_o[i].y;
		o_a_z += flt->a[i] * hist_o[i].z;
	}
	
	// denormalization
	o_x = (o_b_x >> (flt->B_NORM_BITS - flt->A_NORM_BITS)) - o_a_x;
	o_y = (o_b_y >> (flt->B_NORM_BITS - flt->A_NORM_BITS)) - o_a_y;
	o_z = (o_b_z >> (flt->B_NORM_BITS - flt->A_NORM_BITS)) - o_a_z;
	
	// get filtered A
	A.x = o_x >> flt->A_NORM_BITS;
	A.y = o_y >> flt->A_NORM_BITS;
	A.z = o_z >> flt->A_NORM_BITS;
	
	// Update history data
	for (i = LPF_TAPS-1; i > 0; i--) {
		hist_o[i] = hist_o[i-1];
		hist_i[i] = hist_i[i-1];
	}
	hist_o[i] = A;
	hist_i[i] = in;

    return A;
}

// 
// 3D magnitude calculation with shifting
//
static I32S _comp_magnitude(ACCELEROMETER_3D in, I8U sh)
{
    I32S x = in.x>>sh;
    I32S y = in.y>>sh;
    I32S z = in.z>>sh;

	return (x*x+y*y+z*z);
}

static void _calc_mag()
{
	MAGNITUDE_STAT *m = &gPDM.mag;
	I32S mag;
	
	// Calculate the magnitude and we might not need filter out the magnitude
	//
	mag = _comp_magnitude(gPDM.A, 0);

	// filtering needed, here, we use a simple RC filter to smooth out the magnitude
	// 
	if (!m->prev) {
		m->prev = mag;
		m->v = mag;
	} else {
		m->prev = m->v;
		m->v = m->prev + ((mag-m->prev)>>3);
	}
}

static void _reset_pedo_stationary()
{
	STATIONARY_STAT *s = &gPDM.stationary;

    // Reset stationary state
	s->acc.samples = 0;
	s->acc.x = 0;
	s->acc.y = 0;
	s->acc.z = 0;
}

static BOOLEAN _det_stationary()
{
	STATIONARY_STAT *s = &gPDM.stationary;
	I32S diff, diff_2, diff_3;
	I16U det_stat;
	
    // a second filter to smooth out magnitude for stationary detection
	if (!s->mag_prev) {
		// ZERO after initialization, the first value assignment
		s->mag_prev = gPDM.mag.v;
		s->mag_curr = gPDM.mag.v;
	} else {
		// RC Low pass filtering
		s->mag_prev = s->mag_curr;
		s->mag_curr = s->mag_prev + ((gPDM.mag.v-s->mag_prev)>>3);
	}
	
	s->mag_win[0] = s->mag_win[1];
	s->mag_win[1] = s->mag_win[2];
	s->mag_win[2] = s->mag_win[3];
	s->mag_win[3] = s->mag_curr;

	// Delta - 0 < - > 1
	diff = BASE_abs((I32S)(s->mag_win[0]-s->mag_win[1]));
	
	// Delta: 0 < - > 2
	diff_2 = BASE_abs((I32S)(s->mag_win[0]-s->mag_win[2]));
	if (diff_2 > diff)
		diff = diff_2;
	
	// Delta: 0 < - > 3  
  diff_3 = BASE_abs((I32S)(s->mag_win[0]-s->mag_win[3]));
	if (diff_3 > diff)
		diff = diff_3;

	// compare to a threshold to determine whether it is a stationary state
	if (diff > STATIONARY_ENG_TH) {
		if (s->acc.samples > STATIONARY_TIME_EARLY_TH) {
			// Update stationary Gest
			s->est.x = -(s->acc.x/s->acc.samples);
			s->est.y = -(s->acc.y/s->acc.samples);
			s->est.z = -(s->acc.z/s->acc.samples);
			
			s->norm = (I32S)BASE_sqrt(_comp_magnitude(s->est, 0));
			
			s->norm_init = TRUE;
			// At end of stationary period, we reset some of important components
			// Classifier CAR timer - 2 minutes
      gPDM.classifier.stationary_ts = gPDM.global_time;
			
			det_stat = STATIONARY_END;
		} else {
			det_stat = STATIONARY_NONE;
		}

		// Reset stationary state, including "samples" and accumulated X/Y/Z accelerations.
		_reset_pedo_stationary();

	} else {
		s->acc.samples ++;
		s->acc.x += gPDM.A.x;
		s->acc.y += gPDM.A.y;
		s->acc.z += gPDM.A.z;
		
		// depand on the duration of stationay, we declare it as 1) regular stationary, 2) low power stationary
		if (s->acc.samples > STATIONARY_TIME_LOW_POWER_TH) {
			det_stat = STATIONARY_LOW_POWER_TRUE;
		} else if (s->acc.samples > STATIONARY_TIME_TH) {
			det_stat = STATIONARY_REAL_TRUE;
    } else if (s->acc.samples > STATIONARY_TIME_EARLY_TH) {
			det_stat = STATIONARY_EARLY_TRUE;
		} else {
			det_stat = STATIONARY_NONE;
		}
	}
	
	// if a stationary period is yet to be detected, we take a running average
	// for Gest
	if (!s->norm_init) {
		
		if (s->init.samples < (PEDO_SPS<<1)) {
			s->init.x += gPDM.A.x;
			s->init.y += gPDM.A.y;
			s->init.z += gPDM.A.z;
			s->init.samples ++;
			
			s->est.x = -(s->init.x/s->init.samples);
			s->est.y = -(s->init.y/s->init.samples);
			s->est.z = -(s->init.z/s->init.samples);
			
			s->norm = NOMINAL_G;
		} else {
			s->norm = (I32S)BASE_sqrt(_comp_magnitude(s->est, 0));
			s->norm_init = TRUE;
		}
	}
	
	if (det_stat == STATIONARY_LOW_POWER_TRUE) {
			if (gPDM.classifier.motion != MOTION_STATIONARY_LOW_POWER) {
					gPDM.classifier.motion = MOTION_STATIONARY_LOW_POWER;
					return TRUE;
			}
	} else if (det_stat == STATIONARY_REAL_TRUE) {
			// If pedometer is already in a state of either stationary or stationary_low_power,
			// no update on the motion type.
			if ((gPDM.classifier.motion != MOTION_STATIONARY) && 
							(gPDM.classifier.motion != MOTION_STATIONARY_LOW_POWER)) 
			{
				gPDM.classifier.motion = MOTION_STATIONARY;
				return TRUE;
			}
	}

	return FALSE;
}

static ACCELEROMETER_3D _3D_norm(ACCELEROMETER_3D in, I32S norm)
{
	I32S tmp;
    ACCELEROMETER_3D out;

    if (!norm) {
        return in;
    }

	// Normalizing A
	tmp = in.x << _GET_NORM_SH;
	out.x = tmp / norm;
	
	tmp = in.y << _GET_NORM_SH;
	out.y = tmp / norm;
	
	tmp = in.z << _GET_NORM_SH;
	out.z = tmp / norm;
	
    return out;
}

static void _norm_A()
{
	if (!gPDM.stationary.norm) {return;}
    
  gPDM.A = _3D_norm(gPDM.A, gPDM.stationary.norm);
}

static void _g_est_win(ACCELEROMETER_3D new_g)
{
	ADAPTIVE_G_ESTIMATE *g = &gPDM.g_adj;
	I32S x, y,z, i;

	// take mean over a window of steps
	for (i = ADJ_G_STEP_WIN-1; i > 0; i --) {
        g->hist_g[i] = g->hist_g[i-1];
	}
	g->hist_g[0] = new_g;
			
	// get a estimate
	x = g->hist_g[0].x;
	y = g->hist_g[0].y;
	z = g->hist_g[0].z;
			
	// take mean over 2 steps to estimate G, mitigating the step loss of attitude change
	for (i = 1; i < ADJ_G_STEP_WIN; i++) {
		x += g->hist_g[i].x;
		y += g->hist_g[i].y;
		z += g->hist_g[i].z;
	}
			
	g->est.x = x / ADJ_G_STEP_WIN;
	g->est.y = y / ADJ_G_STEP_WIN;
	g->est.z = z / ADJ_G_STEP_WIN;

	// Initializaton for next step
	g->acc_one_step.x = gPDM.A.x;
	g->acc_one_step.y = gPDM.A.y;
	g->acc_one_step.z = gPDM.A.z;

	g->acc_one_step.samples = 1;
	g->min_t = gPDM.global_time;
}

// 
// Take raw Accelerometer data in, and calculate a longer term average as the G reference
static void _g_norm_update(ACCELEROMETER_3D in)
{
	ADAPTIVE_G_ESTIMATE *g = &gPDM.g_adj;
    I32S x, y, z;

    // Update background reference G accumulator.
    g->acc_10s.x += in.x;
    g->acc_10s.y += in.y;
    g->acc_10s.z += in.z;
			
    g->acc_10s.samples ++;

    // Set a up limit of 5 seconds
    if (g->acc_10s.samples >= 250) {
        // Reduce the counter by half
        g->acc_10s.x >>= 1;
        g->acc_10s.y >>= 1;
        g->acc_10s.z >>= 1;
			
        g->acc_10s.samples >>= 1;
    }

	// 20 ms, 5 samples -> 100 ms,
	// so acc_10s gets updated every 100 ms, which pass through low pass filter
	// to get lt_est (long-term G estimate)
    if (gPDM.global_time > g->norm_ts + 5) {
        x = -g->acc_10s.x / g->acc_10s.samples;
		y = -g->acc_10s.y / g->acc_10s.samples;
		z = -g->acc_10s.z / g->acc_10s.samples;

        g->lt_est.x = g->lt_est.x + ((x - g->lt_est.x) >> 3);
        g->lt_est.y = g->lt_est.y + ((y - g->lt_est.y) >> 3);
        g->lt_est.z = g->lt_est.z + ((z - g->lt_est.z) >> 3);
        g->norm_ts += 5;
    }
}

// Adaptive gravity estimation using min/max of magnitude of A.
static void _adj_g_est(ACCELEROMETER_3D in)
{
	ADAPTIVE_G_ESTIMATE *g = &gPDM.g_adj;
	I32S diff;
	I32S i;
  ACCELEROMETER_3D m_acc;
	
	// detect the min point based on three conditions
	//
	//  1. local minima
	//  2. must be smaller than 1g
	//  3. a window within [0.26 s, 2s], which corresponding to 
	//     a 3 steps per second to 1/2 step per second, i.e., the neighboring
    //     minima has to be 0.26 seconds apart
	//
	
	// check if it is initialized
    if (!g->b_init) {
        // 
        g->b_init = TRUE;

		// if not, initialize the gravity estimator
		g->est.x = -in.x;
		g->est.y = -in.y;
		g->est.z = -in.z;
	    // take mean over a window of steps
	    for (i = 0; i < ADJ_G_STEP_WIN; i ++) {
            g->hist_g[i] = g->est;
	    }
        g->lt_est = g->est;
    
		return;
	}
	
    // Obtain a time difference
	diff = gPDM.global_time - g->min_t;

	// b_update: get set when a new step is detected
	// filter out noise and non minima samples
	if (!g->b_update)  
	{
        if (diff > WIN_ADJ_G_MAX) {
            // Using long-term average for a good estimate
            _g_est_win(g->lt_est);
        } else {
		    g->acc_one_step.x += gPDM.A.x;
		    g->acc_one_step.y += gPDM.A.y;
		    g->acc_one_step.z += gPDM.A.z;
            g->acc_one_step.samples ++;
        }
	} else {

		// Turn off the update flag
        g->b_update = FALSE;

        if (diff < WIN_ADJ_G_MIN) {
		    g->acc_one_step.x += gPDM.A.x;
		    g->acc_one_step.y += gPDM.A.y;
		    g->acc_one_step.z += gPDM.A.z;
            g->acc_one_step.samples ++;
        } else {
            // if this is the very first point
            m_acc.x = -(g->acc_one_step.x / g->acc_one_step.samples);
            m_acc.y = -(g->acc_one_step.y / g->acc_one_step.samples);
            m_acc.z = -(g->acc_one_step.z / g->acc_one_step.samples);

            _g_est_win(m_acc);
        }
    }

    
    // Update a long-term G estimate through low-pass filtering
    _g_norm_update(in);
	
}

static ACCELEROMETER_3D _add_3d(ACCELEROMETER_3D in1, ACCELEROMETER_3D in2)
{
	ACCELEROMETER_3D out;
	
	out.x = in1.x + in2.x;
	out.y = in1.y + in2.y;
	out.z = in1.z + in2.z;
	
	return out;
}

static I32S _dot_3d(ACCELEROMETER_3D in1, ACCELEROMETER_3D in2)
{
	return (in1.x*in2.x + in1.y*in2.y + in1.z*in2.z);
}

static void _calc_apu_p2p(CLASSIFIER_STAT *c, I32S apu)
{
    // Two steps a time for P2P check
    //if (gPDM.step.count & 0x01) 
	{
        c->apu_p2p = BASE_abs(c->apu_max - c->apu_min);
        c->apu_max = apu;
        c->apu_min = apu;
    }
}

static void _update_apu_step_ts(CLASSIFIER_STAT *c)
{
    I16S i;

	// Update time stamp and reset its stat.
	for (i = CLASSIFIER_STEP_WIN_SZ-1; i > 0; i--) {
        c->step_ts[i] = c->step_ts[i-1];
        c->step_stat[i] = c->step_stat[i-1];
        c->step_motion[i] = c->step_motion[i-1];
	}
	
	// Time stamp for the particular step
	c->step_ts[0] = gPDM.global_time;
	// Flag it a step to be classified
	c->step_stat[0] = STEP_TO_BE_CLASSIFIED;  
	// Motion to be classified
	c->step_motion[0] = MOTION_UNKNOWN;
}

// Update walking and running statistics
static void _update_w_r_activity(CLASSIFIER_STAT *c)
{
    I32S i;

    // Sliding window shift by 1 step
    for (i = CLASSIFIER_STEP_WIN_SZ-1; i > 0; i--) {
        c->w_r_votes[i] = c->w_r_votes[i-1];
    }

    // Update walking and running vote
    c->w_r_votes[0] = c->apu_p2p-(I32S)CLASSIFIER_WRC_1_TH;
}

static void _update_car_exiting(CLASSIFIER_STAT *c)
{
    BOOLEAN b_exit = FALSE;
    I8U i, cnt = 0;
    MOTION_TICK_CTX *pt = (MOTION_TICK_CTX *)&gPDM.tick;

    // When we observe dramatic change on APU_P2P, we should exit CAR mode
    if (gPDM.classifier.apu_p2p > (CLASSIFIER_WRC_2_TH)) {
        b_exit = TRUE;
    } else {
        for (i = 0; i < CLASSIFIER_STEP_WIN_SZ; i++) {
            if ((c->w_r_votes[i] + CLASSIFIER_WRC_1_TH) < CLASSIFIER_CAR_AUP_EXIT_TH) {
                cnt = 0;
            } else {
                cnt ++;
            }
            if (cnt > 5) {
                b_exit = TRUE;
                break;
            }
            if (!c->w_r_votes[i])
                break;
            else if (c->w_r_votes[i] > 0) {
                b_exit = TRUE;
                break;
            }
        }
    }

    if (b_exit) {
        c->step_consistency_th = STEP_CONSISTENCY_THRESHOLD_MIN;
        pt->st.ts = gPDM.global_time; // Reset short-term distribution process
        for (i = 0; i < AUP_HIST_STEPS; i++)
            pt->st.hist[i] = 0;
        pt->car_context_certainty = CAR_CTX_LOW;
    }
}

static void _update_orientation(ACCELEROMETER_3D in, CLASSIFIER_STAT *c)
{
    ANTI_CHEATING_CTX *ac = &c->anti_cheating;
    I32S a_3d_diff, i;
    
    // Sliding window shift by 1 step
    for (i = CLASSIFIER_STEP_WIN_SZ-1; i > 0; i--) {
        c->step_a_diff[i] = c->step_a_diff[i-1];
    }

    ac->a_acc_s2 = ac->a_acc_s1;
    ac->a_acc_s1 = ac->a_acc_s;
    ac->a_acc_s = ac->a_acc;
    
    ac->a_acc.x = in.x;
    ac->a_acc.y = in.y;
    ac->a_acc.z = in.z;

    // 
    a_3d_diff = BASE_abs(ac->a_acc_s.x - ac->a_acc_s2.x);
    a_3d_diff += BASE_abs(ac->a_acc_s.y - ac->a_acc_s2.y);
    a_3d_diff += BASE_abs(ac->a_acc_s.z - ac->a_acc_s2.z);

    c->step_a_diff[0] = (I32U) a_3d_diff;
    
}

static void _update_classifier_stat(ACCELEROMETER_3D in, I32S apu, BOOLEAN b_step)
{
    CLASSIFIER_STAT *c = &gPDM.classifier;
    ANTI_CHEATING_CTX *ac = &c->anti_cheating;

    if (b_step) {
	
        // A prime up P2P : for walking and running classification
        _calc_apu_p2p(c, apu);

        // Update the time stamp of detected steps
        _update_apu_step_ts(c);

        // Walking and Running update
        _update_w_r_activity(c);

        // CAR exiting update
        _update_car_exiting(c);

        // Non-intentional cheating stat update
        _update_orientation(in, c);
    } else {
        // Every new step is detected, we update the peak-to-peak of A prime up
        // and take current A prime up as the max/min initialization
        if (apu > c->apu_max) {
            c->apu_max = apu;
        }
        if (apu < c->apu_min) {
            c->apu_min = apu;
        }

        ac->a_acc.x += in.x;
        ac->a_acc.y += in.y;
        ac->a_acc.z += in.z;
    }
}


static void _classifier_stat_update(ACCELEROMETER_3D in, BOOLEAN b_step)
{
    CLASSIFIER_STAT *c = &gPDM.classifier;
    I32U ts_s, ts_e;
	
    //Ap_mag_main = _comp_magnitude(gPDM.A_prime, 0);

    if (b_step) {
        // Looking back past 8 steps, and calculate the pace
        ts_s = c->step_ts[0];
        ts_e = c->step_ts[8];
        if (ts_s && ts_e) {
            c->apu_pace = (ts_s-ts_e) >> 3;
        } else {
            c->apu_pace = PEDO_SPS << 1; // a large cadence - 0.25 steps per second
        }
    }
		
    // Update the statistics of afs and a'up
    // The main purpose for this is to check whether side and vertical acceleration
    // holds certain ratio for normal walking/running cases
    _update_classifier_stat(in, gPDM.A_prime_up, b_step);

}

static BOOLEAN _step_count()
{
	ACCELEROMETER_3D A_prime;
	I32S A_prime_up, peak, bottom, diff0, diff1;
	STEP_COUNT_STAT *sc=&gPDM.step;
	BOOLEAN b_p2p = FALSE;

    // Calculate A prime, A minus earth gravity
	A_prime = _add_3d(gPDM.A, gPDM.g_est);
	gPDM.A_prime = A_prime;
	
	// dot product to obtain vertical projection of A_prime
	A_prime_up = -_dot_3d(A_prime, gPDM.g_est);
	gPDM.A_prime_up = A_prime_up;


	// counting steps
	sc->prev2 = sc->prev;
	sc->prev = sc->curr;
	sc->curr = A_prime_up;
	
	if (gPDM.global_time < (sc->prev_g_time + 4))
		return FALSE;

	if ((sc->prev > sc->prev2) && (sc->prev > sc->curr)) {
		sc->prev_g_time = gPDM.global_time;
		peak = sc->prev;
		if (peak > sc->p2p_peak) {
			sc->p2p_peak = peak;
			sc->p2p_bump_set |= 2;
			b_p2p = TRUE;
		}
		// Find a new Peak, we should update bottom if the nearest bottom is much lower
		if (sc->p2p_bottom_0 > sc->p2p_bottom_1) {
			sc->p2p_bottom_0 = sc->p2p_bottom_1;
			sc->p2p_bump_set |= 1;
			sc->p2p_bump_set &= 0x03;
			sc->p2p_bottom_1 = 0x7fffffff; // set a maximum value

			// This seems to be a valid bottom point, let's update peak as well
			sc->p2p_peak = peak;
			sc->p2p_bump_set |= 2;
			b_p2p = TRUE;
		}

		if (b_p2p) {
			sc->p2p_bump_set &= 0x03;
			sc->p2p_bottom_1 = 0x7fffffff; // set a maximum value
		}
	}
	else if ((sc->prev < sc->prev2) && (sc->prev < sc->curr)) {
		sc->prev_g_time = gPDM.global_time;
		bottom = sc->prev;
		if (bottom < sc->p2p_bottom_1) {
			sc->p2p_bottom_1 = bottom;
			sc->p2p_bump_set |= 0x04;

			// This is the point to check whether a valid step is detected
			if (sc->p2p_bump_set == 0x07) {
				diff0 = sc->p2p_peak - sc->p2p_bottom_0;
				diff1 = sc->p2p_peak - sc->p2p_bottom_1;
				if (diff0 > diff1) {
					if (diff1 > STEP_COUNT_PEAK_DIFF_TH) {
						diff0 *= 153;
						diff0 >>= 10; // 183/1024 ~ 0.15
						if (diff1 > diff0) {
							// a step is detected
							sc->count ++;
							sc->p2p_bump_set = 0x01;
							sc->p2p_peak = 0x80000000;
							sc->p2p_bottom_0 = bottom;
							sc->p2p_bottom_1 = 0x7fffffff;
							return TRUE;
						}
					}
				}
				else {
					if (diff0 > STEP_COUNT_PEAK_DIFF_TH) {
						diff1 *= 153;
						diff1 >>= 10; // 183/1024 ~ 0.15
						if (diff0 > diff1) {
							// a step is detected
							sc->count ++;
							sc->p2p_bump_set = 0x01;
							sc->p2p_peak = 0x80000000;
							sc->p2p_bottom_0 = bottom;
							sc->p2p_bottom_1 = 0x7fffffff;
							return TRUE;
						}
					}
				}
			}
		}
	}

	return FALSE;
}

static MOTION_TYPE _WALKING_RUNNING_classify(CLASSIFIER_STAT *c, I32S win_siz)
{
    MOTION_TYPE motion;
    I32S votes;
    I32S i;

    // Deal with individual steps, though the steps are mostly taken care of by anti-cheating algorithm
    if (!win_siz) {
        return MOTION_WALKING;
    }

    votes = 0;
    for (i = 0; i < win_siz; i++) {
        votes += c->w_r_votes[i];
    }

    votes /= win_siz;

    // Simple classification on running and walking 
    // 1. High the A'UP P2P differential
    // 2. Faster than 2 steps per second
    //
    if (c->apu_pace < 20) {
        // 1. High pace
        //    Running: Faster than 2.5 steps per second (and P2P is less the pure walking threshold
		votes += (CLASSIFIER_WRC_1_TH - CLASSIFIER_WRC_PURE_WALK_TH);
		if (votes < 0)
			motion = MOTION_WALKING;
		else
			motion = MOTION_RUNNING;
    } else if (votes > 0) {
        // 2. High APU magnitude
        //    Running: > 2 steps per second
        //    Walking: otherwise
        if (c->apu_pace < 25) {
            motion = MOTION_RUNNING;
        } else {
            motion  = MOTION_WALKING;
        }
    } else {
        // 3. Median APU magnitude
        //    Running: > 2.2 steps per second
        //    Walking: otherwise
        votes += (CLASSIFIER_WRC_1_TH-CLASSIFIER_WRC_2_TH);
        if ((votes > 0) && (c->apu_pace < 22)) {
            motion = MOTION_RUNNING;
        } else {
            motion = MOTION_WALKING;
        }
    }

    return motion;
}

static I32S _get_classify_win_siz(CLASSIFIER_STAT *c, I32U ts_th)
{
    I32S win_siz;
    I32U ts_diff;
    I32S i;

    win_siz = 0;
    for (i = 0; i < CLASSIFIER_STEP_WIN_SZ; i++) {
        ts_diff = c->step_ts[0] - c->step_ts[i];
        if ((ts_diff <= ts_th) && (c->step_stat[i] != STEP_NOT_AVAILABLE)) {
            win_siz ++;
        }
    }

    return win_siz;
}

static BOOLEAN _vehicle_step_check(CLASSIFIER_STAT *c, I32U idx)
{
	I32S apu_avg = 0;
	I32U tick_diff;
	I8U i;

	i = idx;
	// Check 4 STEP TIME WINDOW,
	tick_diff = c->step_ts[i] - c->step_ts[i + 3];
	if (tick_diff <= 50) {
		apu_avg += c->w_r_votes[i];
		apu_avg += c->w_r_votes[i + 1];
		apu_avg += c->w_r_votes[i + 2];
		apu_avg += c->w_r_votes[i + 3];
		apu_avg >>= 2;
		apu_avg += CLASSIFIER_WRC_1_TH;
		if (apu_avg < (NOMINAL_G_MAG >> 1)) {
			return TRUE;
		}
	}
	return FALSE;
}

// If a step coming through all classification, we finally validate it against pace
// make sure it is consistent
static MOTION_TYPE _noise_validate(CLASSIFIER_STAT *c, MOTION_TYPE motion, I32U idx)
{
	I32U walk_end_ts, run_end_ts, p2p;
	I8U valid_step_cnt = 0;
	I8U i;

	// The pace
	if ((idx == CLASSIFIER_STEP_WIN_SZ - 1) || !c->step_ts[idx + 1]) {
		return motion;
	}
	// Make sure we can go back for 1 second
	if (c->step_ts[idx] < 47)
		return motion;
	run_end_ts = c->step_ts[idx] - 47;
	walk_end_ts = c->step_ts[idx] - 37;

	c->step_is_noise = FALSE;

	// Need a double check on whether it is a real step, as we know two steps cannot be really close
	// - overcount - if we do not validate the step timing
	// 50/12 ~ 4.1  -> Assumption: two steps should be at least 12 samples apart.
	if (motion == MOTION_WALKING) {
		for (i = idx + 1; i < CLASSIFIER_STEP_WIN_SZ; i++) {
			if ((c->step_motion[i] == MOTION_RUNNING) || (c->step_motion[i] == MOTION_WALKING)) {
				if (c->step_ts[i] < walk_end_ts)
					break;
				else
					valid_step_cnt++;
			}
		}
		if (valid_step_cnt >= 2) {
			motion = MOTION_UNKNOWN;
			c->step_is_noise = TRUE;
		}

		// Check whether it is likely to be in a vechicle
		if (_vehicle_step_check(c, idx)) {
			motion = MOTION_UNKNOWN;
			c->step_is_noise = TRUE;
			c->start_normal_activity = FALSE;
		}
	}
	else if (motion == MOTION_RUNNING) {
		for (i = idx + 1; i < CLASSIFIER_STEP_WIN_SZ; i++) {
			if ((c->step_motion[i] == MOTION_RUNNING) || (c->step_motion[i] == MOTION_WALKING))
			{
				if (c->step_ts[i] < run_end_ts)
					break;
				else
					valid_step_cnt++;
			}
		}
		if (valid_step_cnt >= 3) {
			motion = MOTION_UNKNOWN;
			c->step_is_noise = TRUE;
		}
		else {
			p2p = c->w_r_votes[idx] + CLASSIFIER_WRC_1_TH;
			if (p2p < CLASSIFIER_RUNNING_MIN_TH) {
				motion = MOTION_UNKNOWN;
				c->step_is_noise = TRUE;
			}
		}
	}
	return motion;
}

static MOTION_TYPE _CAR_classify(CLASSIFIER_STAT *c, MOTION_TYPE motion)
{
    MOTION_TICK_CTX *pt = (MOTION_TICK_CTX *)&gPDM.tick;
    I16U idx;
    I8U cnt;

    // Check whether we are likely in a vibration mode (like device in moving vechicle)
    cnt = BASE_calculate_occurance(gPDM.tick.raw.peak_vibrate_window, 32);

    // If there is no vibration detected, return original motion
    if (cnt <= 2) {
        return motion;
    }

    // If device seems to wake up from a non-motion mode, let's pass it through 
    if (gPDM.global_time < (c->stationary_ts + (PEDO_SPS<<4))) {
        if (c->step_consistency_th == STEP_CONSISTENCY_THRESHOLD_MIN)
            return motion;
    }
    
    // filter out those sporadic CAR steps
    if (c->motion == MOTION_RUNNING) {
        c->step_consistency_th = STEP_CONSISTENCY_THRESHOLD_MIN;
        pt->car_context_certainty = CAR_CTX_LOW;
        pt->st.ts = gPDM.global_time; // Reset short-term distribution process
        for (idx = 0; idx < AUP_HIST_STEPS; idx++)
            pt->st.hist[idx] = 0;
        return motion;
    }
    
    if (pt->car_context_certainty == CAR_CTX_HIGH) {
        motion = MOTION_CAR;
    } else if (pt->car_context_certainty == CAR_CTX_MEDIAN && c->step_consistency_th >= 10) {
        motion = MOTION_CAR;
    }
    return motion;
}

static void _update_classification_status(CLASSIFIER_STAT *c, MOTION_TYPE motion, I8U step_idx)
{
    c->step_stat[step_idx] = STEP_ALREADY_CLASSIFIED;
    c->step_motion[step_idx] = motion;
    c->step_idx = step_idx;
    c->motion = motion;
}

static void _core_classify(CLASSIFIER_STAT *c, STEP_COUNT_STAT *sc, I8U step_idx)
{
    MOTION_TYPE motion = MOTION_UNKNOWN;
    I32S win_siz;
    
    //
    // The classification follows a sequence as (Walking/Running) -> (StairsUp) -> (Car) -> (Cheating)
    //

    // 1/ get the window size (4 seconds of statistics) for classification
    win_siz = _get_classify_win_siz(c, PEDO_SPS<<2);

    // 2/ Binary classification on WALKING/RUNNING
    //    Votes are from a span of CLASSIFICATION_TIME_SPAN
    motion = _WALKING_RUNNING_classify(c, win_siz);

		if (!cling.activity.b_workout_active) {
			// 3/ Car (moving vechicle) Classication
			motion = _CAR_classify(c, motion);

			// 4/ Validate the step against the noise
			motion = _noise_validate(c, motion, step_idx);
		}
		
    // 7/ Update classification status
    _update_classification_status(c, motion, step_idx);

}


static I8S _if_a_step_to_classify(CLASSIFIER_STAT *c)
{
	I8S i;

	// Find the 1st to be classifed step
	for (i = (CLASSIFIER_STEP_WIN_SZ - 1); i >= 0; i --) {
		if (c->step_stat[i] == STEP_TO_BE_CLASSIFIED) {
			break;
		}
	}

	if (i<0) { // No step to classify
		return (-1);
	}

    // If the step is approaching the time latency, start classification right away
	if ((c->step_ts[i]) < gPDM.global_time) {
		return (i);
	}

	// If we have filled up more than half of step window, start classification right away
	if ((i+1) >= (CLASSIFIER_STEP_WIN_SZ>>1)) {
		return (i);
	}

	// Any other cases, we idle the classifier
	return (-1);
}

static void _small_step_check(CLASSIFIER_STAT *c)
{
	I32S apu_avg = 0;
	I32U tick_diff;
	I8U i, j;

	for (i = 0; i < (CLASSIFIER_STEP_WIN_SZ - 4); i++) {
		if (c->step_stat[i] == STEP_ALREADY_CLASSIFIED)
			break;
		// Check 4 STEP TIME WINDOW,
			tick_diff = c->step_ts[i] - c->step_ts[i + 3];
			if (tick_diff <= 50) {
				apu_avg += c->w_r_votes[i];
				apu_avg += c->w_r_votes[i + 1];
				apu_avg += c->w_r_votes[i + 2];
				apu_avg += c->w_r_votes[i + 3];
				apu_avg >>= 2;
				apu_avg += CLASSIFIER_WRC_1_TH;
				if (apu_avg < STEP_COUNT_ENTRANCE_TH) {
					j = i;
					for (; j < CLASSIFIER_STEP_WIN_SZ; j++) {
						if (c->step_stat[j] == STEP_TO_BE_CLASSIFIED) {
							c->step_stat[j] = STEP_ALREADY_CLASSIFIED;
							c->unknown_steps_compensation++;
							c->step_motion[j] = MOTION_UNKNOWN;
							N_SPRINTF("--- unknown --> UNKNOWN (2) ---");
						}
					}
				}

			}
	}
}


static BOOLEAN _is_incidental_steps(CLASSIFIER_STAT *c, I8U last_step_ts)
{
    I8U cnt, i;

    // Get rid of any previous incidental steps
    cnt = 0;
    for (i = 0; i < CLASSIFIER_STEP_WIN_SZ; i ++) {
        if (c->step_a_diff[i] < pedo_constrain_diff_th) {
            cnt = 0;
        } else {
            cnt ++; // Counts how many inconsistent steps in the window
        }

        if (cnt >= 12) {
            for (; i < CLASSIFIER_STEP_WIN_SZ; i ++) {
                if (c->step_stat[i] == STEP_TO_BE_CLASSIFIED) {
					c->step_stat[i] = STEP_ALREADY_CLASSIFIED;
					// If these steps are noise, we should clean the step compensation counter as well
                    // For CAR steps, we should show them since we do care about CAR steps as a indicator
                    if (c->step_consistency_th > STEP_CONSISTENCY_THRESHOLD_MIN) {
                        c->car_steps_compensation ++;
                        c->step_motion[i] = MOTION_CAR;
                    } else {
                        c->unknown_steps_compensation ++;
                        c->step_motion[i] = MOTION_UNKNOWN;
                    }
                }
            }
        }
    }

    // Make sure we have balanced A prime up
	_small_step_check(c);

    // Make sure not some hand-shaking like movement causing device to count steps
    cnt = BASE_calculate_occurance(gPDM.tick.raw.pair_ratio_lo, 6);
    if (cnt > 3) {
        return TRUE;
    }

    // Apply non-intentional cheating criterion.
    cnt = 0;

    for (i = 0; i < (pedo_constrain_step_th<<1); i++) {
        if ((c->step_stat[i] == STEP_TO_BE_CLASSIFIED) && c->step_ts[i])  {
            if (c->step_a_diff[i] < pedo_constrain_diff_th) 
            {
                cnt ++;
            }
        }
    }

//    if (cnt < (pedo_constrain_step_th>>1)) {
	if (cnt < 6) {
        return TRUE;
    } else { // If we have at least 6 steps looks very similar, start up counting procedure
        return FALSE;
    }
}

static BOOLEAN _do_steps_keep_consistent(CLASSIFIER_STAT *c)
{
	I8U i, cnt;
	
	cnt = 0;
	
    for (i = 0; i < 10; i++) {
				if (c->step_a_diff[i] < pedo_constrain_diff_th) 
				{
						cnt ++;
				}
    }
		
		// Make sure the latest 6 steps (out of 10) are consistent
		if (cnt < pedo_constrain_consistent_step_th)
			return FALSE;
		else
			return TRUE;
}

static BOOLEAN _is_noise_non_step(CLASSIFIER_STAT *c)
{
    I8U i;
    I32U ts_strt, diff;

    if (c->step_stat[0] != STEP_TO_BE_CLASSIFIED) {
	    return TRUE;
	}

	// Clean up random steps, then perform a consistency exam
    ts_strt = gPDM.global_time;

    for (i = 0; i < CLASSIFIER_STEP_WIN_SZ; i ++) {
        diff = ts_strt - c->step_ts[i];

		if (i > 30) {
			if ((c->step_stat[i] == STEP_TO_BE_CLASSIFIED) && c->step_ts[i])
				// Exit un-intentional cheating mode
				return FALSE;
			else
				break;
		} else if (diff > pedo_noise_cleanup_th) {
            for (; i < CLASSIFIER_STEP_WIN_SZ; i ++) {
                if (c->step_stat[i] == STEP_TO_BE_CLASSIFIED) {
										c->step_stat[i] = STEP_ALREADY_CLASSIFIED;
										// If these steps are noise, we should clean the step compensation counter as well
                    // For CAR steps, we should show them since we do care about CAR steps as a indicator
                    if (c->step_consistency_th > STEP_CONSISTENCY_THRESHOLD_MIN) {
                        c->car_steps_compensation ++;
                        c->step_motion[i] = MOTION_CAR;
                    } else {
                        c->unknown_steps_compensation ++;
                        c->step_motion[i] = MOTION_UNKNOWN;
                    }
                }
            }
        }
        ts_strt = c->step_ts[i];
    }
    
    for (i = 0; i < CLASSIFIER_STEP_WIN_SZ; i ++) {
        if (c->step_stat[i] != STEP_TO_BE_CLASSIFIED) {
            break;
        }
    }
		
		// Check if device is in a transportation mode
		if (PEDO_is_transport_mode()) {
					for (i = 0; i < CLASSIFIER_STEP_WIN_SZ; i ++) {
							if (c->step_stat[i] == STEP_TO_BE_CLASSIFIED) {
								c->step_stat[i] = STEP_ALREADY_CLASSIFIED;
                c->step_motion[i] = MOTION_UNKNOWN;
							}
					}
					return TRUE;
		}

    // If consistent step event presents, we further confirm they are not device incidental movement
    if (i >= pedo_constrain_step_th) {
        if (_is_incidental_steps(c, i-1))
            return TRUE;
        else
            return FALSE;
    } else {
        return TRUE;
    }
}


static BOOLEAN _motion_classification()
{
	I8S step_idx;
	CLASSIFIER_STAT *c = &gPDM.classifier;

    // Reset confidence level
	c->step_is_noise = FALSE;

	if (c->car_steps_compensation > 0) {
#if 1
			c->car_steps_compensation = 0;
#else
			// Show the steps as a indication of driving.
			c->car_steps_compensation --;
			c->motion = MOTION_CAR;

		// Do NOT set up motion flag as it won't go into overall step count
			return TRUE;
#endif
	}
	
	if (c->unknown_steps_compensation > 0) {
#if 1
		c->unknown_steps_compensation = 0;
#else
			// Show the steps as a indication of driving.
			c->unknown_steps_compensation --;
    c->motion = MOTION_UNKNOWN;

		// Do NOT set up motion flag as it won't go into overall step count
			return FALSE;
#endif
	}

	// Pre-classifcation, filtering out random steps
	if (!c->start_normal_activity) {
			// Make sure we are not in working out mode
			if (!cling.activity.b_workout_active) {
				if (_is_noise_non_step(c)) {
						 return FALSE;
				} 
			}

			// Start normal classification
			c->start_normal_activity = TRUE;

			// Some unknown steps from noise suppression
			if (c->unknown_steps_compensation) {
					return FALSE;
			}
	}

	// The classification starts after a pre-defined latency, 
	step_idx = _if_a_step_to_classify(c);

	if (step_idx < 0) {

			return FALSE;
	}

    // core classification
	_core_classify(&gPDM.classifier, &gPDM.step, (I8U) step_idx);
	
	if (!cling.activity.b_workout_active) {

		// Finally, make sure we are getting consistent steps, otherwise, restart incidental movement detection
		if (!_do_steps_keep_consistent(c)) {
				c->start_normal_activity = FALSE;
		}

		// If device enters transportation mode, get out the state
		if (PEDO_is_transport_mode()) {
			c->start_normal_activity = FALSE;
		}

	}
	
	return TRUE;
}

static BOOLEAN _update_apu_distribution(APU_HISTORGRAM *hist, I8U idx, I8U length_sh)
{
    I8U i;
    I32U perc_20, perc_80, perc=0;

    hist->dist_update = FALSE;

    // Determine threshold
    if (gPDM.global_time < (hist->ts+(PEDO_SPS<<length_sh))) {
        hist->hist[idx] ++;
    } else {
        perc_20 = (PEDO_SPS<<(length_sh+1))/10;
        perc_80 = (PEDO_SPS<<(length_sh+3))/10;
        // Update the time stamp.
        hist->ts = gPDM.global_time;

        // Update CDF characteristic
        for (i = 0; i < AUP_HIST_STEPS; i++) {
            perc += hist->hist[i];
            if (perc > perc_20) {
                hist->cdf_20_percentile = i;
                break;
            }
        }
        for (; i < AUP_HIST_STEPS; i++) {
            perc += hist->hist[i];
            if (perc > perc_80) {
                hist->cdf_80_percentile = i;
                break;
            }
        }

        // Clear the histgram.
        for (i = 0; i < AUP_HIST_STEPS; i++) {
            hist->hist[i] = 0;
        }

        // Initialize the first index
        hist->hist[idx] ++;

        // Flag the distribution update
        hist->dist_update = TRUE;
    }

    return hist->dist_update;
}

static void _orig_statistic(RAW_ACCE_STAT *r, I32S in) 
{

    if (in > r->max_global) {
        r->max_global = in;
    } else if (in < r->min_global) {
        r->min_global = in;
    }
    if ((r->curr > in) && (r->curr > r->prev)) {
        if (r->curr > (r->min_local + 500)) {
            r->num_strike ++;
        }
        r->num_vibrate ++;
        r->dir = GENERAL_DIR_DOWN;
    } else if ((r->curr < in) && (r->curr < r->prev)) {
        r->min_local = r->curr;
        r->dir = GENERAL_DIR_UP;
    } 
    if (gPDM.g_adj.b_update) {
        r->acc_step = 0;                    // Start new round of accumulation
    }
    if (r->dir == GENERAL_DIR_UP) {
        r->acc_step += in;
    } else {
        r->acc_step -= in;
    }
    // Update raw data
    r->prev = r->curr;
    r->curr = in;
}

static void _reset_raw_stat(RAW_ACCE_STAT *r)
{
    r->num_strike = 0;
    r->num_vibrate = 0;
}

static I32S _calc_raw_stat(RAW_ACCE_STAT *r, I32S in)
{
    I32S p2p = r->max_global-r->min_global;

    r->max_global = in;
    r->min_global = in;

    return p2p;
}

static void _update_raw_stat(ACCELEROMETER_INPUT_STAT *raw, RAW_ACCE_STAT *stat)
{

    raw->peak_strike_single = stat->num_strike;  // Peak stride within a second
    raw->peak_vibrate_single = stat->num_vibrate;
}

static void _acce_correlation(ACCELEROMETER_3D in, MOTION_TICK_CTX *pt)
{
    ACCELEROMETER_INPUT_STAT *r = &pt->raw;
    ACCELEROMETER_3D p2p;   // the maximum of each axis
    I32U ratio;
    I8U cnt;

    if (gPDM.global_time >= r->ts+PEDO_SPS) {
        r->ts = gPDM.global_time;
        p2p.x = _calc_raw_stat(&r->stat[0], in.x);
        p2p.y = _calc_raw_stat(&r->stat[1], in.y);
        p2p.z = _calc_raw_stat(&r->stat[2], in.z);

        r->pair_ratio_lo <<= 1;
        r->pair_ratio_hi <<= 1;
        r->peak_strike_window <<= 1;
        r->peak_vibrate_window <<= 1;
        r->peak_vibrate_smooth <<= 1;
        if (!p2p.x || !p2p.y || !p2p.z) {
            ratio = 0;
            r->peak_strike_single = 0;
            r->min_mag = 0;
            r->max_mag = 0;
        } else {
            if (p2p.x > p2p.y) {
                if (p2p.y > p2p.z) {  // x > y > z
                    ratio = (p2p.x<<1)/p2p.z;           // Ratio
                    r->min_mag = p2p.z;                 // Minimum component magnitude from accelerometer
                    r->max_mag = p2p.x;
                    r->max_acce_idx = 0;
                } else if (p2p.x > p2p.z) { // x > z > y
                    ratio = (p2p.x<<1)/p2p.y;
                    r->min_mag = p2p.y;
                    r->max_mag = p2p.x;
                    r->max_acce_idx = 0;
                } else { // z > x > y
                    ratio = (p2p.z<<1)/p2p.y;
                    r->min_mag = p2p.x;
                    r->max_mag = p2p.z;
                    r->max_acce_idx = 2;
                }
            } else {
                if (p2p.x > p2p.z) { // y > x > z
                    ratio = (p2p.y<<1)/p2p.z;
                    r->min_mag = p2p.z;
                    r->max_mag = p2p.y;
                    r->max_acce_idx = 1;
                } else if (p2p.y > p2p.z) { // y > z > x
                    ratio = (p2p.y<<1)/p2p.x;
                    r->min_mag = p2p.x;
                    r->max_mag = p2p.y;
                    r->max_acce_idx = 1;
                } else { // z > y > x
                    ratio = (p2p.z<<1)/p2p.x;
                    r->min_mag = p2p.x;
                    r->max_mag = p2p.z;
                    r->max_acce_idx = 2;
                }
            }

            _update_raw_stat(r, &r->stat[r->max_acce_idx]);
        }

        // Keep 8 second of axis peak-to-peak ratio history
        if (ratio > AXIS_RATIO_CHEATING_TH_LO)
            r->pair_ratio_lo |= 1;
        if (ratio > AXIS_RATIO_CHEATING_TH_HI)
            r->pair_ratio_hi |= 1;
        if (r->peak_strike_single >= 7)
            r->peak_strike_window |= 1;

        // Detect whether device is in a vibrating mode
        // The maximum magnitude of one axis (peak) is within 1 G range, while it gets a lot of vibration
        if ((r->peak_vibrate_single >= 10) && (r->max_mag >= 300) && (r->max_mag <= 1500))
            r->peak_vibrate_window |= 1;

        // Detect whether device is possiblly in a cheating mode
        if (r->peak_vibrate_single <= 3)
            r->peak_vibrate_smooth |= 1;

        // Check whether the device is likely in a CAR mode
        cnt = BASE_calculate_occurance(r->peak_vibrate_window, 32);
        if (cnt >= 16) {
            // Declare a CAR mode
            pt->b_vibrate_mode = TRUE;
            pt->vibrate_ts = gPDM.global_time;
        }
				
        if (pt->b_vibrate_mode) {
            // Vibration mode expires after 60 seconds
            if (gPDM.global_time > (pt->vibrate_ts + 2500)) {
                pt->b_vibrate_mode = FALSE;
            }
            // Important - Exit CAR mode if we see real steps coming
            cnt = BASE_calculate_occurance(r->peak_vibrate_window, 6);
            if (cnt < 2) {
                pt->b_vibrate_mode = FALSE;
            }
        } 

        // Clear stride counts
        _reset_raw_stat(&r->stat[0]);
        _reset_raw_stat(&r->stat[1]);
        _reset_raw_stat(&r->stat[2]);
    } 
    
    if (gPDM.g_adj.b_update) {
        r->acc_symmetric_window <<= 1;
        r->acc = BASE_abs(r->stat[r->max_acce_idx].acc_step);
        if (r->acc < 10000) {
            r->acc_symmetric_window |=  1;  // Accumulating the waveform for its symmetric check
        }
    }

    // Calculate the number of strike
    //   - If the neighboring maxima and minima pair has a gap larger than 500, it is a strike
    //
    _orig_statistic(&r->stat[0], in.x);
    _orig_statistic(&r->stat[1], in.y);
    _orig_statistic(&r->stat[2], in.z);
}

static void _tick_processing(ACCELEROMETER_3D in, I32S A_prime_up)
{
    MOTION_TICK_CTX *pt = (MOTION_TICK_CTX *)&gPDM.tick;
    CLASSIFIER_STAT *c = &gPDM.classifier;
    I32U idx;

    // Explore accelerometer 3-axis correlation
    _acce_correlation(in, pt);

    // Calculate the histogram of A'UP
    if (A_prime_up > 0) {
        idx = (A_prime_up+(AUP_HIST_Q_STEP>>1))/AUP_HIST_Q_STEP;
    } else {
        idx = ((AUP_HIST_Q_STEP>>1)-A_prime_up)/AUP_HIST_Q_STEP;
    }
    if (idx > (AUP_HIST_STEPS-1))
        idx = AUP_HIST_STEPS - 1;

    // Calcuate long-term distribution over a window of 128 seconds.
    if (_update_apu_distribution(&pt->lt, idx, PEDO_TICK_LENGTH_LONG)) {
        // Update the consistency step threshold
        if (pt->lt.cdf_80_percentile > 10) {
            c->step_consistency_th = STEP_CONSISTENCY_THRESHOLD_MIN;
        } else if (pt->lt.cdf_20_percentile > 0) {
            c->step_consistency_th += STEP_CONSISTENCY_THRESHOLD_INC;
            if (c->step_consistency_th > STEP_CONSISTENCY_THRESHOLD_MAX) {
                c->step_consistency_th = STEP_CONSISTENCY_THRESHOLD_MAX;
            }
        } else if (pt->lt.cdf_80_percentile > 0) {
            c->step_consistency_th += (STEP_CONSISTENCY_THRESHOLD_INC>>1);
            if (c->step_consistency_th > STEP_CONSISTENCY_THRESHOLD_MAX) {
                c->step_consistency_th = STEP_CONSISTENCY_THRESHOLD_MAX;
            }
        } 
    }
    // Calcuate short-term distribution over a window of 5 seconds.
    if (_update_apu_distribution(&pt->st, idx, PEDO_TICK_LENGTH_SHORT)) {
        // Update the consistency step threshold
        pt->car_context_certainty = CAR_CTX_LOW;
        pt->st.dist_update = FALSE;
        if ((pt->st.cdf_80_percentile > 0) && (pt->st.cdf_80_percentile < 5)) {
            pt->car_context_certainty = CAR_CTX_HIGH;
        } else if ((pt->st.cdf_20_percentile > 0) && (pt->st.cdf_80_percentile > 15)) {
            c->step_consistency_th = STEP_CONSISTENCY_THRESHOLD_MIN;
            pt->st.dist_update = TRUE;
        } else if (pt->st.cdf_80_percentile <= 13) {
            pt->car_context_certainty = CAR_CTX_MEDIAN;
        } else {
            pt->car_context_certainty = CAR_CTX_MEDIAN;
            pt->st.dist_update = FALSE;
        }
    }

}


static void _clear_steps_buffer(I8U start_idx)
{
	I8U i;
	CLASSIFIER_STAT *c = &gPDM.classifier;

	for (i = start_idx; i < CLASSIFIER_STEP_WIN_SZ; i++) {
			if (c->step_stat[i] == STEP_TO_BE_CLASSIFIED) {
					c->step_stat[i] = STEP_ALREADY_CLASSIFIED;
					// If these steps are noise, we should clean the step compensation counter as well
					// For CAR steps, we should show them since we do care about CAR steps as a indicator
					if (c->step_consistency_th > STEP_CONSISTENCY_THRESHOLD_MIN) {
							c->car_steps_compensation ++;
							c->step_motion[i] = MOTION_CAR;
					} else {
							c->unknown_steps_compensation ++;
							c->step_motion[i] = MOTION_UNKNOWN;
					}
			}
	}
}

static void _clean_up_random_steps()
{
	I32U tick_diff, tick_first_step;

	tick_first_step = gPDM.classifier.step_ts[0];
	if (gPDM.global_time > tick_first_step) {

		tick_diff = gPDM.global_time - tick_first_step;

		if (tick_diff > pedo_noise_cleanup_th) {
			gPDM.classifier.start_normal_activity = FALSE;

			// Clear up all un-classified steps
			_clear_steps_buffer(0);
			return;
		}
	}
	else
	{
		gPDM.classifier.start_normal_activity = FALSE;

		// Clear up all un-classified steps
		_clear_steps_buffer(0);
		return;
	}

	// Check pace, pause pedometer if pace is detected lower than 1 step per second
	if (gPDM.classifier.step_ts[2]) {
		tick_diff = tick_first_step - gPDM.classifier.step_ts[2];
		if (tick_diff >= 100) { // 2 seconds for 2 steps
			gPDM.classifier.start_normal_activity = FALSE;

			// Clear up all un-classified steps from the 2nd steps forward
			_clear_steps_buffer(1);
			return;
		}
	}

	if (gPDM.classifier.step_ts[3]) {
		tick_diff = tick_first_step - gPDM.classifier.step_ts[3];
		if (tick_diff >= 150) {// 3 seconds for 3 steps
			gPDM.classifier.start_normal_activity = FALSE;

			// Clear up all un-classified steps from the 2nd steps forward
			_clear_steps_buffer(1);
			return;
		}
	}

}

static int steps_overall = 0;

I16U PEDO_main(ACCELEROMETER_3D in)
{
	I16U stat=PDM_STAT_NONE;// Flag whether we need to report a event

	// Update a global timer
	// Time tick, every 20 ms (50 Hz)
	gPDM.global_time ++;
	
	// Restart unintentional cheating detection if non-step period is longer than 10 seconds
	_clean_up_random_steps();

	// Low pass filtering input 3-D accelerometer A, mainly used for step detection
  gPDM.A = _lpf(in, (LP_FILTER *)&lpf_coeff[MAIN_LPF_8HZ], &gPDM.main_lpf);
	
	// Calculate magnitude of A, and low pass it
	_calc_mag();
	
	// detecting stationary period
	//
	 if (_det_stationary()) {
		 stat |= PDM_MOTION_DETECTED;
		 // If device gets into stationary mode, start unintentioanl cheating detection
		 gPDM.classifier.start_normal_activity = FALSE;
		 
		 N_SPRINTF("-- STATIONARY DETECTED --");
		 // Clear up all un-classified steps
		 _clear_steps_buffer(0);
	 } else {
		 // If the device appears to be stationary, skip all the step count and classification
		 if (gPDM.stationary.acc.samples > PEDO_SPS) {
			 return stat;
		 }
	 }

	// Normalize the accelerometer input
	_norm_A();
	
	// Calculate Adaptive Gravity using min/max of mag_A 
	_adj_g_est(in);
	
	// Determine Gest, we should mainly use adaptive Gest calculated on the fly,
	// however, if we skip processing during stationary, we might see some
	// discontinuities of Gest when we exit stationary period...
	// A potential optimization might need here ...
	//
    // Get adaptive Gest. Though A has been normalized, but after taking the mean
    // the vector might not be a unit, so we should normalize the adaptive one too (TO-DO).
    gPDM.g_est = gPDM.g_adj.est;
	 
	if (stat & PDM_MOTION_DETECTED)
		return stat;
    
    // Count steps, detect one step, and update classification information
	if (_step_count()) {
        // Every new step is detected, we update the peak-to-peak of A prime up
        // and take current A prime up as the max/min initialization
        _classifier_stat_update(in, TRUE);

        // Also time to update G
        gPDM.g_adj.b_update = TRUE;

        // Reset stationary 
        _reset_pedo_stationary();
		
				steps_overall ++;
				N_SPRINTF("STEPS overall -- %d/%d --", steps_motion, steps_overall);
		
    } else{
        _classifier_stat_update(in, FALSE);
    }

	// Classify one step.
	if (_motion_classification()) {
		// Get rid of noisy steps 
		if (!gPDM.classifier.step_is_noise) {
			stat = PDM_STEP_DETECTED | PDM_MOTION_DETECTED;
			
			if (gPDM.classifier.motion != MOTION_UNKNOWN)
				gPDM.classifier.step_count ++;
				N_SPRINTF("STEPS motion ++ %d ++", steps_motion);
			
				// User is in a motion state, tune up sensitivity
				PEDO_set_step_detection_sensitivity(TRUE);
		}
	} 
	

    // pedometer tick processing, statistic per second for 
    // long-term activity classification, e.g., CAR, incidental movement detection
    _tick_processing(in, gPDM.A_prime_up);
	
	return stat;
}



I16U PEDO_get_pace()
{
    CLASSIFIER_STAT *c = &gPDM.classifier;

	return c->apu_pace;
}

BOOLEAN PEDO_is_transport_mode()
{
	if (gPDM.tick.car_context_certainty == CAR_CTX_HIGH) {
		return TRUE;
	} else if (gPDM.tick.car_context_certainty == CAR_CTX_MEDIAN && gPDM.classifier.step_consistency_th >= 10) {
		return TRUE;
	}

	return FALSE;
}

MOTION_TYPE PEDO_get_motion_type()
{
	return gPDM.classifier.motion;
}

void PEDO_set_motion_type(MOTION_TYPE mode)
{
	gPDM.classifier.motion = mode;
}

I8U* PEDO_get_global_buffer()
{
	return (I8U *)&gPDM;
}

void PEDO_set_step_detection_sensitivity(BOOLEAN b_sensitivity)
{	
	if (b_sensitivity) {
		if (cling.user_data.m_pedo_sensitivity == PEDO_SENSITIVITY_HIGH) {
			pedo_constrain_diff_th = CONSTRAINS_DIFF_HI_TH;
			pedo_constrain_step_th = CONSTRAINS_STEP_HI_TH;
			pedo_noise_cleanup_th = NOISE_STEP_CLEANUP_HI_TH;
			pedo_constrain_consistent_step_th = CONSISTENT_STEP_HI_TH;
		} else if (cling.user_data.m_pedo_sensitivity == PEDO_SENSITIVITY_MEDIUM) {
			pedo_constrain_diff_th = CONSTRAINS_DIFF_ME_TH;
			pedo_constrain_step_th = CONSTRAINS_STEP_ME_TH;
			pedo_noise_cleanup_th = NOISE_STEP_CLEANUP_ME_TH;
			pedo_constrain_consistent_step_th = CONSISTENT_STEP_ME_TH;
		} else if (cling.user_data.m_pedo_sensitivity == PEDO_SENSITIVITY_LOW) {
			pedo_constrain_diff_th = CONSTRAINS_DIFF_LO_TH;
			pedo_constrain_step_th = CONSTRAINS_STEP_LO_TH;
			pedo_noise_cleanup_th = NOISE_STEP_CLEANUP_LO_TH;
			pedo_constrain_consistent_step_th = CONSISTENT_STEP_LO_TH;
		} else {
			pedo_constrain_diff_th = CONSTRAINS_DIFF_HI_TH;
			pedo_constrain_step_th = CONSTRAINS_STEP_HI_TH;
			pedo_noise_cleanup_th = NOISE_STEP_CLEANUP_HI_TH;
			pedo_constrain_consistent_step_th = CONSISTENT_STEP_HI_TH;
		}
		return;
	} else {
		// If user enters a sendentary state, set pedometer sensitivty to LOW sensitivity
		pedo_constrain_diff_th = CONSTRAINS_SLEEP_DIFF_TH;
		pedo_constrain_step_th = CONSTRAINS_SLEEP_STEP_TH;
		
		return;
	}
	
	N_SPRINTF("[PEDO] set sensitivity: %d, %d, %d", 
		cling.user_data.m_pedo_sensitivity, pedo_constrain_diff_th, pedo_constrain_step_th);
}
