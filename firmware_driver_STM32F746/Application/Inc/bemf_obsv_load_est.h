#ifndef _BEMF_OBSV_LOAD_EST_
#define _BEMF_OBSV_LOAD_EST_

#include <math.h>
#include <math_f.h>

typedef struct bemf_obsv_load_est_s {
	float			ts;		    					/* sampling time */
	float			psi_s_alpha_hat;		    	/*  */
	float			psi_r_alpha_hat;		    	/*  */
	float			psi_s_beta_hat;		    		/*  */
	float			psi_r_beta_hat;		    		/*  */
	float			theta_hat;		    			/*  */
	float			omega_hat;		    			/*  */
	float			torque_load_hat;		    	/*  */
	float			bemf_obsv_fb_p;		    		/*  */
	float			bemf_obsv_p;		    		/*  */
	float			motorc_omega_bez;		    	/*  */
	float			motorc_omega_bez_i;		    	/*  */
	float			motorc_m_scale;		    		/*  */
	float			motorc_rs_norm;		    		/*  */
	float			motorc_ls_norm;		    		/*  */
	float			motorc_psi_m_norm;		    	/*  */
	float			motorc_load_inertia_norm;		/*  */
	float			bemf_obsv_luenberger_1;			/*  */
	float			bemf_obsv_luenberger_2;			/*  */
	float			bemf_obsv_luenberger_3;			/*  */
} bemf_obsv_load_est_t;

#define BEMF_OBSV_LOAD_EST bemf_obsv_load_est_t

void bemf_obsv_load_est_init(volatile BEMF_OBSV_LOAD_EST *f, volatile float bemf_obsv_fb_p, volatile float bemf_obsv_p,
		volatile float motorc_omega_bez, volatile float motorc_m_scale, volatile float motorc_rs_norm, 
		volatile float motorc_ls_norm, volatile float motorc_psi_m_norm, volatile float motorc_load_inertia_norm, volatile float bemf_obsv_luenberger_1,
		volatile float bemf_obsv_luenberger_2, volatile float bemf_obsv_luenberger_3);

void bemf_obsv_load_est_ts(volatile BEMF_OBSV_LOAD_EST *f, volatile float ts);

void bemf_obsv_load_est_reset(volatile BEMF_OBSV_LOAD_EST *f);

void bemf_obsv_load_est_update(volatile BEMF_OBSV_LOAD_EST *f,volatile float bemf_obsv_fb_p, volatile float bemf_obsv_p);

float bemf_obsv_load_est_process(volatile BEMF_OBSV_LOAD_EST *f, volatile float u_alpha, volatile float u_beta, volatile float i_alpha,
	volatile float i_beta, volatile float u_dc, volatile float motor_torque_pu);

#endif
