#include "fb_state_machine.h"
#include "math.h"
#include "math_f.h"
#include "sv_pwm.h"

volatile param_t param;
volatile STATE_MACHINE global_sm_ctrl;
volatile INVERTER_CTRL inv_ctrl;

volatile cmd_word_generation_t global_cmd_word_generation;
volatile double_pulse_test_pwm_t double_pulse_test;

volatile uint16_t adc1_buf[ADC1_BUF_LEN] = {0};
volatile uint16_t adc2_buf[ADC2_BUF_LEN] = {0};
volatile uint16_t adc3_buf[ADC3_BUF_LEN] = {0};

volatile uint32_t adc_current_u_channel_offset = 0;
volatile uint32_t adc_current_v_channel_offset = 0;
volatile uint32_t adc_current_w_channel_offset = 0;
volatile uint16_t adc_current_calibration_counter = 0;


volatile SIGNAL_GENERATOR phase_signal;


void peripherals_init()
{


	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);

	/* enable TIM-DMA for top switches */
	HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)&inv_ctrl.inv_uab_sv_out.du,1);
	HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_2,(uint32_t *)&inv_ctrl.inv_uab_sv_out.dv,1);
	HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_3,(uint32_t *)&inv_ctrl.inv_uab_sv_out.dw,1);

	/* enable TIM-DMA for bottom switches */
	HAL_TIMEx_PWMN_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)&inv_ctrl.inv_uab_sv_out.du,1);
	HAL_TIMEx_PWMN_Start_DMA(&htim1,TIM_CHANNEL_2,(uint32_t *)&inv_ctrl.inv_uab_sv_out.dv,1);
	HAL_TIMEx_PWMN_Start_DMA(&htim1,TIM_CHANNEL_3,(uint32_t *)&inv_ctrl.inv_uab_sv_out.dw,1);

	/* force all gate mosfet command to low-state in low-impedance */
	inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
	inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
	inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
	inv_ctrl.inv_uab_sv_out.ua = 0;
	inv_ctrl.inv_uab_sv_out.ub = 0;

    sv_pwm_process(&inv_ctrl.inv_uab_sv_out);

	/* init DMA for ADCs */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buf, ADC1_BUF_LEN);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buf, ADC2_BUF_LEN);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc3_buf, ADC3_BUF_LEN);

	/* init DAC */
	HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

	/* init CAN BUS */
	fb_canbus_init();

	/* in standard operation, during breakpoint debug the IWD is freeze that means is not operating - the following macro keep the IWD running also during breaking point from debug
	 * this condition will force a reset after 4*125us */
	__HAL_DBGMCU_UNFREEZE_IWDG();
}

void adc_current_calibration()
{
	if (adc_current_calibration_counter < ADC_CALIBRATION_SAMPLES)
	{
		adc_current_u_channel_offset = adc_current_u_channel_offset + (uint32_t)((adc1_buf[0] + adc2_buf[0] + adc3_buf[0])/3);
		adc_current_v_channel_offset = adc_current_v_channel_offset + (uint32_t)((adc1_buf[2] + adc2_buf[2])/2);
		adc_current_w_channel_offset = adc_current_w_channel_offset + (uint32_t)(adc3_buf[2]);
		adc_current_calibration_counter++;
	}
	if (adc_current_calibration_counter == ADC_CALIBRATION_SAMPLES)
	{
		adc_current_u_channel_offset = (uint32_t)(adc_current_u_channel_offset/ADC_CALIBRATION_SAMPLES);
		adc_current_v_channel_offset = (uint32_t)(adc_current_v_channel_offset/ADC_CALIBRATION_SAMPLES);
		adc_current_w_channel_offset = (uint32_t)(adc_current_w_channel_offset/ADC_CALIBRATION_SAMPLES);

		param.inverter_current_u_offset = - param.inverter_current_scale * adc_current_u_channel_offset;
		param.inverter_current_v_offset = - param.inverter_current_scale * adc_current_v_channel_offset;
		param.inverter_current_w_offset = - param.inverter_current_scale * adc_current_w_channel_offset;

		adc_current_calibration_counter++;
	}
}

void global_state_machine_init()
{
	/* force gate top and bottom to zero */
	inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
	inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
	inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
	inv_ctrl.inv_uab_sv_out.ua = 0;
	inv_ctrl.inv_uab_sv_out.ub = 0;
    sv_pwm_process(&inv_ctrl.inv_uab_sv_out);

	/* initialization of the global state machine structure */
	global_sm_ctrl.global_cmd = GLOBAL_CMD_RESET;
    global_sm_ctrl.global_state = GLOBAL_STATE_STOP;
    global_sm_ctrl.global_fault_state_description = GLOBAL_STATE_NONE_ERROR;

	/* initialization of the inverter state machine structure */
    global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
    global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_STOP;
    global_sm_ctrl.inverter_fault_state_description &= INVERTER_NONE_ERROR;

	/* initialization of the pwm enable/disbale commands */
    global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;

	/* initialization of the brake enable/disbale commands */
    global_sm_ctrl.brake_enable = BRAKE_DISABLE;

	/* set nominal data and scaling factors of the motor variables */
	param.motor_current_max = MOTOR_MAXIMUM_CURRENT;
	param.motor_current_nom = MOTOR_NOMINAL_CURRENT;
	param.motor_current_scale = ((param.motor_current_max / param.motor_current_nom) * 2.0) / ADC_RESOLUTION;
	param.motor_current_offset = - param.motor_current_scale * ADC_RESOLUTION / 2.0;
	param.motor_voltage_max = MOTOR_MAXIMUM_VOLTAGE;
	param.motor_voltage_nom = MOTOR_NOMINAL_VOLTAGE;
	param.motor_voltage_scale = ((param.motor_voltage_max / param.motor_voltage_nom) * 2.0) / ADC_RESOLUTION;
	param.motor_voltage_offset = - param.motor_voltage_scale * ADC_RESOLUTION / 2.0;

	/* set nominal data and scaling factors for data */
	param.inverter_current_max = INVERTER_MAXIMUM_CURRENT;
	param.inverter_current_nom = INVERTER_NOMINAL_CURRENT;
	param.inverter_current_scale = ((param.inverter_current_max / param.inverter_current_nom) * 2.0) / ADC_RESOLUTION;
	param.inverter_current_offset = - param.inverter_current_scale * ADC_RESOLUTION / 2.0;

	if (adc_current_calibration_counter == 0)
	{
		param.inverter_current_u_offset = - param.inverter_current_scale * ADC_RESOLUTION / 2.0;
		param.inverter_current_v_offset = - param.inverter_current_scale * ADC_RESOLUTION / 2.0;
		param.inverter_current_w_offset = - param.inverter_current_scale * ADC_RESOLUTION / 2.0;
	}

	param.inverter_voltage_max = INVERTER_MAXIMUM_VOLTAGE;
	param.inverter_voltage_nom = INVERTER_NOMINAL_VOLTAGE;
	param.inverter_voltage_scale = ((param.inverter_voltage_max / param.inverter_voltage_nom) * 2.0) / ADC_RESOLUTION;
	param.inverter_voltage_offset = - param.inverter_voltage_scale * ADC_RESOLUTION / 2.0;

	/* set nominal data and scaling factors for temperature measures */
	param.temperature_igbt_inverter_max = INVERTER_MAXIMUM_TEMPERATURE;
	param.temperature_igbt_inverter_nom = INVERTER_NOMINAL_TEMPERATURE;
	param.temperature_igbt_inverter_scale = ((param.temperature_igbt_inverter_nom) * 1.0) / ADC_RESOLUTION;
	param.temperature_igbt_inverter_offset = 0;

	/* set nominal data and scaling factors for DClink measure */
	param.dclink_voltage_max = DCLINK_MAXIMUM_VOLTAGE;
	param.dclink_voltage_nom = DCLINK_NOMINAL_VOLTAGE;
	param.dclink_voltage_scale = param.dclink_voltage_max / param.dclink_voltage_nom * 1.0 / ADC_RESOLUTION;
	param.dclink_voltage_offset = 0;

	/* normalization of the motor parameters */
	param.motorc_omega_bez = MOTOR_NOMINAL_SPEED/60 * MATH_2PI * MOTOR_POLE_PAIRS;
	param.motorc_m_scale = MATH_2_3 * DCLINK_NOMINAL_VOLTAGE/MOTOR_NOMINAL_VOLTAGE;
	param.motorc_xbez = MOTOR_NOMINAL_VOLTAGE/MOTOR_NOMINAL_CURRENT;
	param.motorc_lbez = param.motorc_xbez/param.motorc_omega_bez;
	param.motorc_rs_norm = MOTOR_NOMINAL_RS/param.motorc_xbez;
	param.motorc_ls_norm = MOTOR_NOMINAL_LS/param.motorc_lbez;
	param.motorc_ld_norm = MOTOR_NOMINAL_LD/param.motorc_lbez;
	param.motorc_lq_norm = MOTOR_NOMINAL_LQ/param.motorc_lbez;
	param.motorc_psi_bez = MOTOR_NOMINAL_VOLTAGE/param.motorc_omega_bez;
	param.motorc_psi_m_norm = MOTOR_NOMINAL_PHI_M/param.motorc_psi_bez;
	param.motorc_load_inertia_norm = MATH_HALF*MOTOR_NOMINAL_JM*param.motorc_omega_bez/MOTOR_POLE_PAIRS/MOTOR_NOMINAL_TORQUE;

	/* initialization of the system variables */
	param.inverter_theta_hat_sin = 0;
	param.inverter_theta_hat_cos = 0;
	param.dclink_voltage = 0;
	param.inverter_current_u = 0;
	param.inverter_current_v = 0;
	param.inverter_current_w = 0;
	param.temperature_igbt_inverter_u = 0;
	param.temperature_igbt_inverter_v = 0;
	param.temperature_igbt_inverter_w = 0;
	param.temperature_heatsink = 0;
	param.temperature_motor = 0;
	param.temperature_ambient = 0;

	/* initialization of the fault conditions  */
	param.dclink_overvoltage_fault_th = DCLINK_OVERVOLTAGE_FAULT_TH;
	param.dclink_undervoltage_fault_th = DCLINK_UNDERVOLTAGE_FAULT_TH;
	param.inverter_overcurrent_fault_th = INVERTER_OVERCURRENT_FAULT_TH;
	param.omega_max_th = MOTOR_OVERSPEED_FAULT_TH;

	/* initialization of frequency cut filters  */
	param.reference_flt_fcut = 1.0;

	/* initialization of the global state machine and state variables */
	param.global_state = GLOBAL_STATE_RESUME;
	param.inverter_ctrl_state = INVERTER_STATE_RESUME;
	param.global_fault_state_description &= GLOBAL_STATE_NONE_ERROR;
	param.inverter_fault_state_description &= INVERTER_NONE_ERROR;

	/* initialization of the global state machine command variables */
	param.global_cmd = GLOBAL_CMD_RESET;
	param.inverter_cmd = INVERTER_CMD_RESET;

	/* set up of the control variables */
	param.inverter_current_d = 0;
	param.inverter_current_q = 0;
	param.inverter_current_alpha = 0;
	param.inverter_current_beta = 0;
	param.inverter_current_reference_q = 0;
	param.inverter_current_reference_d = 0;

	param.omega_ref_pu = 0;
	param.omega_hat_pu = 0;

	param.iq_ref_fb = 0;
	param.id_ref_fb = 0;

	param.mu = (uint32_t)0;
	param.mv = (uint32_t)0;
	param.mw = (uint32_t)0;

	/* Inverter control mode pre-selection (default value is set to INVERTER_CTRL_TORQUE)
	 * INVERTER_CTRL_SPEED          	4
	 * INVERTER_CTRL_TORQUE         	1
     * INVERTER_CTRL_U_F            	2
	 * INVERTER_CTRL_I_F            	3
	 */
	param.inverter_ctrl_mode = INVERTER_CTRL_TORQUE;

	/* run init functions for inverter control */
	inverter_ctrl_init();
	/* run init functions for generator of the global command word state machine*/
	global_cmd_word_generator_init();



	/* init debug mode */
//	param.debug_mode = DEBUG_MODE_CTRL_U_F;
//	param.debug_mode = DEBUG_MODE_CTRL_I_F;
//	param.debug_mode = DEBUG_MODE_CTRL_TORQUE;
//	param.debug_mode = DEBUG_MODE_CTRL_SPEED;
//	param.debug_mode = DEBUG_MODE_DOUBLE_PULSE_TEST;
	param.debug_mode = 0;


	/* double pulse test parameters - combined three phase pwm using channel 5 in AND with the selected leg */
	param.double_pulse_m_duty = DPT_M_REFERENCE_DUTY;
	param.double_pulse_n_duty = DPT_N_REFERENCE_DUTY;
	param.double_pulse_leg = 3;
	param.double_pulse_test_counter = 1;

}
void inverter_ctrl_reset(void)
{
	/* state observer initialization */
	/*
	bemf_obsv_load_est_init(&inv_ctrl.inv_obsv_load_est, param.bemf_obsv_fb_p, param.bemf_obsv_p, param.motorc_omega_bez,
			param.motorc_m_scale, param.motorc_rs_norm, param.motorc_ls_norm, param.motorc_psi_m_norm, param.motorc_load_inertia_norm, param.bemf_obsv_luenberger_1,
			param.bemf_obsv_luenberger_2, param.bemf_obsv_luenberger_3);
	bemf_obsv_load_est_ts(&inv_ctrl.inv_obsv_load_est, param.inverter_t_pwm);
	*/

	bemf_obsv_init(&inv_ctrl.inv_obsv, param.bemf_obsv_fb_p, param.bemf_obsv_p, param.motorc_omega_bez,
			param.motorc_m_scale, param.motorc_rs_norm, param.motorc_ls_norm, param.motorc_psi_m_norm,
			param.bemf_obsv_kalman_omega, param.bemf_obsv_kalman_theta);
	bemf_obsv_ts(&inv_ctrl.inv_obsv, param.inverter_t_pwm);


	/* dq-vector pi control initialization */
	dqvector_pi_init(&inv_ctrl.inv_cv_ctrl, param.kp_inv_id, param.ki_inv_id, param.kp_inv_iq, param.ki_inv_iq, param.u_out_lim);
	dqvector_pi_ts(&inv_ctrl.inv_cv_ctrl, param.inverter_t_pwm);

	/* speed control initialization */
	regpi_init(&inv_ctrl.inv_speed_ctrl, &param.inverter_t_pwm, &param.kp_omega, &param.ki_omega, &param.torque_lim_top, &param.torque_lim_bottom);

	/* signal generator initialization */
	signal_generator_init(&phase_signal, param.inverter_t_pwm, param.signal_generator_freq_bez);
}

void inverter_ctrl_init(void)
{

	/* sampling time - this value must be coordinated with CLOCK and TIM2 COUNTER BASE
	 * TIM2 as well as TIM1 are served with full clock frequency of 216MHz by APB2
	 * TIM1 and TIM2 are implemented using Center Aligned mode 1 that means the calculus of of the
	 * counting period is determined as: 216MHz/25kHz/2-1 where 25kHz is the frequency of the PWM as well
	 * as of the main task.
	 */
	param.inverter_t_pwm = (float)(PWM_TB*2.0/CLOCK_APB2);

	/* setting for signal generator - this value must be coordinated with CLOCK and TIM2 COUNTER BASE */
	param.signal_generator_freq_bez = 100.0;
	param.signal_generator_u_ref_out = 0.0;

#ifdef MAD_MOTOR
	/* in PMSM the torque current component is located in Q-axis while D-axis is used for voltage de-fluxing or MTPA */

	/* gain parameters for state observers */
	param.bemf_obsv_fb_p = 0.4;
	param.bemf_obsv_p = 0.16;

	/* kalman mechanical observer gains for MAD Motor - the gains shown here below are used for per rad/sec observer*/
	param.bemf_obsv_kalman_theta = 0.0491607270985086;
	param.bemf_obsv_kalman_omega = 9.75378998056553;

	param.bemf_obsv_luenberger_1 = 0.09793;
	param.bemf_obsv_luenberger_2 = 69.6937;
	param.bemf_obsv_luenberger_3 = -334.35;

//	param.bemf_obsv_luenberger_1 = 0.0657;
//	param.bemf_obsv_luenberger_2 = 31.4;
//	param.bemf_obsv_luenberger_3 = -101.0;

//	param.bemf_obsv_luenberger_1 = 0.0495;
//	param.bemf_obsv_luenberger_2 = 17.8;
//	param.bemf_obsv_luenberger_3 = -42.8;

	/* gain parameters for vector control */
	param.kp_omega = 0.0;
	param.ki_omega = 0.0;

	/* current gains */
//	param.kp_inv_id = 0.1875;
//	param.ki_inv_id = 9.0;
//	param.kp_inv_iq = 0.1875;
//	param.ki_inv_iq = 9.0;


	/* current gains */
	param.kp_inv_id = 0.4;
	param.ki_inv_id = 9.0;
	param.kp_inv_iq = 0.4;
	param.ki_inv_iq = 9.0;

	/* current gains */
//	param.kp_inv_id = 0.35;
//	param.ki_inv_id = 8.0;
//	param.kp_inv_iq = 0.35;
//	param.ki_inv_iq = 8.0;

	/* external references - with MAD Motor, for helping the starting process, we set a a small id current useful for rotor pre-alignment  */
	param.id_ext_ref = 0.0;
	param.iq_ext_ref = 0.0;
	param.motor_torque_sign = 1;

#endif

	/* limit of the current control outputs */
	param.u_out_lim = SVPWM_OUT_LIM;

	/* limits of the speed control outputs */
	param.torque_lim_top = TORQUE_PU_LIM;
	param.torque_lim_bottom = -TORQUE_PU_LIM;

	/* initialization of the pwm enable/disbale commands */
	param.pwm_enable = INVERTER_PWM_STOP;

	/* initialization of the brake enable/disbale commands */
	param.brake_enable = BRAKE_DISABLE;

	/* state observer initialization */
	/*
	bemf_obsv_load_est_init(&inv_ctrl.inv_obsv_load_est, param.bemf_obsv_fb_p, param.bemf_obsv_p, param.motorc_omega_bez,
			param.motorc_m_scale, param.motorc_rs_norm, param.motorc_ls_norm, param.motorc_psi_m_norm, param.motorc_load_inertia_norm, param.bemf_obsv_luenberger_1,
			param.bemf_obsv_luenberger_2, param.bemf_obsv_luenberger_3);
	bemf_obsv_load_est_ts(&inv_ctrl.inv_obsv_load_est, param.inverter_t_pwm);
	*/

	bemf_obsv_init(&inv_ctrl.inv_obsv, param.bemf_obsv_fb_p, param.bemf_obsv_p, param.motorc_omega_bez,
			param.motorc_m_scale, param.motorc_rs_norm, param.motorc_ls_norm, param.motorc_psi_m_norm,
			param.bemf_obsv_kalman_omega, param.bemf_obsv_kalman_theta);
	bemf_obsv_ts(&inv_ctrl.inv_obsv, param.inverter_t_pwm);


	/* dq-vector pi control initialization */
	dqvector_pi_init(&inv_ctrl.inv_cv_ctrl, param.kp_inv_id, param.ki_inv_id, param.kp_inv_iq, param.ki_inv_iq, param.u_out_lim);
	dqvector_pi_ts(&inv_ctrl.inv_cv_ctrl, param.inverter_t_pwm);

	/* speed control initialization */
	regpi_init(&inv_ctrl.inv_speed_ctrl, &param.inverter_t_pwm, &param.kp_omega, &param.ki_omega, &param.torque_lim_top, &param.torque_lim_bottom);

	/* signal generator initialization */
	signal_generator_init(&phase_signal, param.inverter_t_pwm, param.signal_generator_freq_bez);

	double_pulse_test_init(&double_pulse_test, param.double_pulse_m_duty, param.double_pulse_n_duty,
			param.double_pulse_leg,	param.double_pulse_test_counter);

}

void global_cmd_word_generator_init()
{
	/* init filters for duty and DClink voltage measure */
	fltlpInit(&global_cmd_word_generation.duty_reference_flt, param.inverter_t_pwm, param.reference_flt_fcut);
	fltlpInit(&global_cmd_word_generation.dclink_voltage_flt, param.inverter_t_pwm, param.reference_flt_fcut);

	/* init structure for the function which generates the global command word
	 * from measure of DClink voltage and Duty reference */
	global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_RESET;
	global_cmd_word_generation.count = 0;
	global_sm_ctrl.global_cmd = GLOBAL_CMD_RESET;

	param.duty_reference = 0.0;
	param.duty_reference_abs = 0.0;
	param.duty_reference_flt = 0.0;

}

void global_cmd_word_generator_reset()
{
	/* init filters for duty and DClink voltage measure */
	fltlpInit(&global_cmd_word_generation.duty_reference_flt, param.inverter_t_pwm, param.reference_flt_fcut);

	param.duty_reference = 0.0;
	param.duty_reference_abs = 0.0;
	param.duty_reference_flt = 0.0;

}

/* global_fault_state_description: is a bitwise word which accumulates the list of active faults for the global system
 * inverter_fault_state_description: is a bitwise word which accumulates the list of active faults for the inverter */
void global_state_machine_process()
{
	/* refresh of the IWD */
	HAL_IWDG_Refresh(&hiwdg);

	/* calibration of the ADCs - remove offset into the currents measure */
	adc_current_calibration();

	/* begin - update of the analog measure quantities taken from DMA buffer */

	param.inverter_current_u = -((float)(param.inverter_current_scale * (adc1_buf[0] + adc2_buf[0] + adc3_buf[0])/MATH_3) + param.inverter_current_u_offset);
	param.inverter_current_v = -((float)(param.inverter_current_scale * (adc1_buf[2] + adc2_buf[2])/MATH_2) + param.inverter_current_v_offset);
	param.inverter_current_w = -((float)(param.inverter_current_scale * adc3_buf[2]) + param.inverter_current_w_offset);
	param.dclink_voltage = (float)(param.dclink_voltage_scale * (adc1_buf[1] + adc2_buf[1] + adc3_buf[1])/MATH_3);
//	param.dclink_voltage = (float)(0.95);

	param.temperature_igbt_inverter_u = (float)(param.temperature_igbt_inverter_scale * adc1_buf[3]);
	param.temperature_igbt_inverter_w = (float)(param.temperature_igbt_inverter_scale * adc2_buf[3]);
	param.temperature_igbt_inverter_v = (float)(param.temperature_igbt_inverter_scale * adc3_buf[3]);

//	param.temperature_ambient = (float)(ADC_RESOLUTION_1 * adc3_buf[5]);
//	param.temperature_heatsink = (float)(ADC_RESOLUTION_1 * adc3_buf[4]);

	/* end - update of the analog measure quantities taken from DMA buffer */

	/* enable can bus main process */
	fb_canbus_process();

	/* debug mode  */
    switch (param.debug_mode)
 	{
		case DEBUG_MODE_CTRL_U_F:

		param.inverter_ctrl_mode = INVERTER_CTRL_U_F;

		global_sm_ctrl.global_cmd = param.global_cmd;

	    param.global_state = global_sm_ctrl.global_state ;
	    param.global_fault_state_description = global_sm_ctrl.global_fault_state_description;

	    param.inverter_cmd = global_sm_ctrl.inverter_cmd;
	    param.inverter_ctrl_state = global_sm_ctrl.inverter_ctrl_state;

	    param.pwm_enable = global_sm_ctrl.pwm_enable;
	    param.brake_enable = global_sm_ctrl.brake_enable;

	    break;

		case DEBUG_MODE_CTRL_I_F:

		param.inverter_ctrl_mode = INVERTER_CTRL_I_F;

		global_sm_ctrl.global_cmd = param.global_cmd;

	    param.global_state = global_sm_ctrl.global_state ;
	    param.global_fault_state_description = global_sm_ctrl.global_fault_state_description;

	    param.inverter_cmd = global_sm_ctrl.inverter_cmd;
	    param.inverter_ctrl_state = global_sm_ctrl.inverter_ctrl_state;

	    param.pwm_enable = global_sm_ctrl.pwm_enable;
	    param.brake_enable = global_sm_ctrl.brake_enable;

	    break;

		case DEBUG_MODE_CTRL_TORQUE:

		param.inverter_ctrl_mode = INVERTER_CTRL_TORQUE;

		global_sm_ctrl.global_cmd = param.global_cmd;

	    param.global_state = global_sm_ctrl.global_state ;
	    param.global_fault_state_description = global_sm_ctrl.global_fault_state_description;

	    param.inverter_cmd = global_sm_ctrl.inverter_cmd;
	    param.inverter_ctrl_state = global_sm_ctrl.inverter_ctrl_state;

	    param.pwm_enable = global_sm_ctrl.pwm_enable;
	    param.brake_enable = global_sm_ctrl.brake_enable;

	    break;

		case DEBUG_MODE_CTRL_SPEED:

		param.inverter_ctrl_mode = INVERTER_CTRL_SPEED;

		global_sm_ctrl.global_cmd = param.global_cmd;

	    param.global_state = global_sm_ctrl.global_state ;
	    param.global_fault_state_description = global_sm_ctrl.global_fault_state_description;

	    param.inverter_cmd = global_sm_ctrl.inverter_cmd;
	    param.inverter_ctrl_state = global_sm_ctrl.inverter_ctrl_state;

	    param.pwm_enable = global_sm_ctrl.pwm_enable;
	    param.brake_enable = global_sm_ctrl.brake_enable;

	    break;

		case DEBUG_MODE_DOUBLE_PULSE_TEST:

			param.inverter_ctrl_mode = INVERTER_DOUBLE_PULSE_TEST;

			global_sm_ctrl.global_cmd = param.global_cmd;

			param.global_state = global_sm_ctrl.global_state ;
			param.global_fault_state_description = global_sm_ctrl.global_fault_state_description;

			param.inverter_cmd = global_sm_ctrl.inverter_cmd;
			param.inverter_ctrl_state = global_sm_ctrl.inverter_ctrl_state;

			param.pwm_enable = global_sm_ctrl.pwm_enable;
			param.brake_enable = global_sm_ctrl.brake_enable;

	    break;

		default:
		/* state of the DClink voltage and state of the duty_reference value are continuously monitored
		 * to generate a global command word. Auto reset is included into this function */
		global_cmd_word_generator_process();
	}

	/* global state machine */
    switch (global_sm_ctrl.global_state)
 	{
		case GLOBAL_STATE_RESUME:

			/* in RESUME state the system is forced to goes into STOP state */
			global_sm_ctrl.global_state = GLOBAL_STATE_STOP;
			global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_RESUME;
			global_sm_ctrl.global_fault_state_description &= GLOBAL_STATE_NONE_ERROR;
			global_sm_ctrl.inverter_fault_state_description &= INVERTER_NONE_ERROR;
            global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
            global_sm_ctrl.brake_enable = BRAKE_DISABLE;
  			inverter_ctrl_init();

		break;

  		case GLOBAL_STATE_STOP:

  			/* In STOP state the DClink voltage is enough to boot up the system, but
			 * duty reference is lower than the start threshold.
			 * During this state PWM are disabled and the check of the alarms is still present.
			 * Initialization of the inverter is enabled
			 * */
  			inverter_ctrl_init();
            global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
  			
            if ((global_sm_ctrl.global_cmd == GLOBAL_CMD_STANDBY) || (global_sm_ctrl.global_cmd == GLOBAL_CMD_RUN))
  			{
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STANDBY;
			}
			else
            {
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
            }

			if (((global_sm_ctrl.global_cmd == GLOBAL_CMD_STANDBY) || (global_sm_ctrl.global_cmd == GLOBAL_CMD_RUN))
					&& (global_sm_ctrl.inverter_ctrl_state == INVERTER_STATE_READY))
  			{
                global_sm_ctrl.global_state = GLOBAL_STATE_READY;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STANDBY;
			}

            /* if at least one of the bitwise error state words is different from zero an error state is triggered */
            if ((global_sm_ctrl.global_fault_state_description)||(global_sm_ctrl.inverter_fault_state_description))
            {
                global_sm_ctrl.global_state = GLOBAL_STATE_ERROR;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
                global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
                global_sm_ctrl.brake_enable = BRAKE_DISABLE;
            }

  		break;

  		case GLOBAL_STATE_ERROR:

  			global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
            global_sm_ctrl.global_state = GLOBAL_STATE_ERROR;

            if (global_sm_ctrl.global_cmd == GLOBAL_CMD_RESET)
  			{
  				global_state_machine_init();
  			}

		break;

  		case GLOBAL_STATE_READY:

  			/* In READY state PWM are set in braking mode. The braking mode condition must constrained to low rotor speed condition.
			  * To measure the speed of the rotor the bemf observer must be in running mode. A possible strategy is to drive a set of id for
			  * a short time and enable the bemf observer to check if the rotor speed is down. */

            global_sm_ctrl.inverter_cmd = INVERTER_CMD_STANDBY;
            global_sm_ctrl.global_state = GLOBAL_STATE_READY;
  			
            if (global_sm_ctrl.global_cmd == GLOBAL_CMD_RUN)
  			{
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_RUN;
  			}
            else if ((global_sm_ctrl.global_cmd == GLOBAL_CMD_STANDBY))
  			{
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STANDBY;
			}
			else
            {
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
            }

			if ((global_sm_ctrl.global_cmd == GLOBAL_CMD_RUN) && (global_sm_ctrl.inverter_ctrl_state == INVERTER_STATE_RUN))
  			{
                global_sm_ctrl.global_state = GLOBAL_STATE_RUN;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_RUN;
			}

			if ((global_sm_ctrl.global_cmd == GLOBAL_CMD_STOP) && (global_sm_ctrl.inverter_ctrl_state == INVERTER_STATE_STOP))
  			{
                global_sm_ctrl.global_state = GLOBAL_STATE_STOP;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
			}

            /* if at least one of the bitwise error state words is different from zero an error state is triggered */
            if ((global_sm_ctrl.global_fault_state_description) || (global_sm_ctrl.inverter_fault_state_description))
            {
                global_sm_ctrl.global_state = GLOBAL_STATE_ERROR;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
                global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
                global_sm_ctrl.brake_enable = BRAKE_DISABLE;
            }

		break;

  		case GLOBAL_STATE_CONNECTING:

  		case GLOBAL_STATE_RUN:
            
  			global_sm_ctrl.inverter_cmd = INVERTER_CMD_RUN;
            global_sm_ctrl.global_state = GLOBAL_STATE_RUN;
  			
            if (global_sm_ctrl.global_cmd == GLOBAL_CMD_RUN)
  			{
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_RUN;
  			}
            else if (global_sm_ctrl.global_cmd == GLOBAL_CMD_STANDBY)
  			{
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STANDBY;
                global_sm_ctrl.global_state = GLOBAL_STATE_READY;
			}
            else if (global_sm_ctrl.global_cmd == GLOBAL_CMD_STOP)
  			{
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
                global_sm_ctrl.global_state = GLOBAL_STATE_STOP;
			}
            else if (global_sm_ctrl.global_cmd == GLOBAL_CMD_RESET)
  			{
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_RESET;
                global_sm_ctrl.global_state = GLOBAL_STATE_STOP;
			}
			else
            {
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
                global_sm_ctrl.global_state = GLOBAL_STATE_STOP;
            }

			if ((global_sm_ctrl.global_cmd == GLOBAL_CMD_RUN) && (global_sm_ctrl.inverter_ctrl_state == INVERTER_STATE_RUN))
  			{
                global_sm_ctrl.global_state = GLOBAL_STATE_RUN;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_RUN;
			}

			if ((global_sm_ctrl.global_cmd == GLOBAL_CMD_STOP) && (global_sm_ctrl.inverter_ctrl_state == INVERTER_STATE_STOP))
  			{
                global_sm_ctrl.global_state = GLOBAL_STATE_STOP;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
			}
            
            if ((global_sm_ctrl.global_cmd == GLOBAL_CMD_STANDBY) && (global_sm_ctrl.inverter_ctrl_state == INVERTER_STATE_READY))
  			{
                global_sm_ctrl.global_state = GLOBAL_STATE_READY;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STANDBY;
			}

            /* if at least one of the bitwise error state words is different from zero an error state is triggered */
            if ((global_sm_ctrl.global_fault_state_description)||(global_sm_ctrl.inverter_fault_state_description))
            {
                global_sm_ctrl.global_state = GLOBAL_STATE_ERROR;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
                global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
                global_sm_ctrl.brake_enable = BRAKE_DISABLE;
            }
  		break;

  		case GLOBAL_STATE_DISCONNECTING:
  			/* The disconnecting state is a state where the rotor speed is controlled/monitored in order to bring the system into
  			 * ready state with low residual rotor speed and enable the braking mode */

  		break;

    }

    /* The inverter state machine is also continuously running and receive command from the global state machine */
	inverter_state_machine_process();

    /* The inverter control process is called at every cycle and receive command from the inverter state machine */
	inverter_ctrl_process();

}

void global_cmd_word_generator_process()
{
	fltlpProcess(&global_cmd_word_generation.duty_reference_flt, param.duty_reference, param.reference_flt_fcut);
	param.duty_reference_flt = global_cmd_word_generation.duty_reference_flt.sig_out;

	fltlpProcess(&global_cmd_word_generation.dclink_voltage_flt, param.dclink_voltage, param.reference_flt_fcut);
	param.dclink_voltage_flt = global_cmd_word_generation.dclink_voltage_flt.sig_out;

	param.duty_reference_abs = fabs(param.duty_reference);

	switch (global_cmd_word_generation.state)
	 	{
			case GLOBAL_CMD_WORD_GENERATION_RESET:

				global_sm_ctrl.global_cmd = GLOBAL_CMD_RESET;
				global_cmd_word_generation.count = 0;
				global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_STOP;

			break;

	  		case GLOBAL_CMD_WORD_GENERATION_STOP:

	            global_sm_ctrl.global_cmd = GLOBAL_CMD_STOP;
	            global_cmd_word_generation.count = 0;

	        	if (param.dclink_voltage_flt >= UDC_ENABLE_CMD_STANDBY) {
					global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_STANDBY;
	        	}
	        	else {
	        		global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_STOP;
	        	}

	        	if (global_sm_ctrl.global_state == GLOBAL_STATE_ERROR) {
	            	global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_WAIT;
	            }
	  		break;

	  		case GLOBAL_CMD_WORD_GENERATION_STANDBY:

	            global_sm_ctrl.global_cmd = GLOBAL_CMD_STANDBY;
	            global_cmd_word_generation.count = 0;

	        	if ((param.dclink_voltage_flt >= UDC_ENABLE_CMD_STANDBY) && (param.duty_reference_abs >= DUTY_ENABLE_CMD_RUN)) {
					global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_RUN;
	        	}
	        	if ((param.dclink_voltage_flt >= UDC_ENABLE_CMD_STANDBY) && (param.duty_reference_abs <= DUTY_ENABLE_CMD_RUN)) {
					global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_STANDBY;
	        	}
	        	if (param.dclink_voltage_flt <= UDC_ENABLE_CMD_STANDBY) {
					global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_STOP;
	        	}
	            if (global_sm_ctrl.global_state == GLOBAL_STATE_ERROR) {
	            	global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_WAIT;
	            }
	  		break;

	  		case GLOBAL_CMD_WORD_GENERATION_RUN:

	            global_sm_ctrl.global_cmd = GLOBAL_CMD_RUN;
	            global_cmd_word_generation.count = 0;

//	            param.inverter_ctrl_mode = INVERTER_CTRL_I_F;
	            param.inverter_ctrl_mode = INVERTER_CTRL_TORQUE;
//	            param.inverter_ctrl_mode = INVERTER_CTRL_SPEED;

	        	if ((param.dclink_voltage_flt >= UDC_ENABLE_CMD_STANDBY) && (param.duty_reference_abs >= DUTY_ENABLE_CMD_RUN)) {
					global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_RUN;
	        	}
	        	if ((param.dclink_voltage_flt >= UDC_ENABLE_CMD_STANDBY) && (param.duty_reference_abs <= DUTY_ENABLE_CMD_RUN)) {
					global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_STANDBY;
	        	}
	        	if (param.dclink_voltage_flt <= UDC_ENABLE_CMD_STANDBY) {
					global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_STOP;
	        	}
	            if (global_sm_ctrl.global_state == GLOBAL_STATE_ERROR) {
	            	global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_WAIT;
	            }
			break;

	  		case GLOBAL_CMD_WORD_GENERATION_WAIT:

	        	if (global_cmd_word_generation.count <= TIMEOUT_AUTO_RESET) {
	        		global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_WAIT;
	        		global_cmd_word_generation.count++;
	        	}
	        	else {
	        		global_cmd_word_generation.state = GLOBAL_CMD_WORD_GENERATION_RESET;
	        	}
			break;
	 }
}

/* the state machine for the inverter is used to decouple from global state machine the condition of
 * process(), initialization() and reset() */
void inverter_state_machine_process()
{
	/* mirror of the inverter control mode */
	global_sm_ctrl.inverter_ctrl_mode = param.inverter_ctrl_mode;

	/* mirrors of the global state machine variables */
    param.global_state = global_sm_ctrl.global_state;
    param.global_fault_state_description = global_sm_ctrl.global_fault_state_description;
    param.global_cmd = global_sm_ctrl.global_cmd;

    /* mirrors of the inverter state machine variables */
    param.inverter_ctrl_state = global_sm_ctrl.inverter_ctrl_state;
    param.inverter_fault_state_description = global_sm_ctrl.inverter_fault_state_description;
    param.inverter_cmd = global_sm_ctrl.inverter_cmd;

    /* mirrors of the global command word generation state */
    param.global_cmd_word_generation_state = global_cmd_word_generation.state;

	/* mirror of the pwm control variables */
    param.pwm_enable = global_sm_ctrl.pwm_enable;
    param.brake_enable = global_sm_ctrl.brake_enable;

	switch (global_sm_ctrl.inverter_ctrl_state)
	{
		case INVERTER_STATE_RESUME:

			/* During INVERTER_STATE_RESUME the pwm is disabled and the system goes to INVERTER_STATE_STOP */
            global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
            global_sm_ctrl.brake_enable = BRAKE_DISABLE;
            global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_STOP;
			global_sm_ctrl.inverter_fault_state_description &= INVERTER_NONE_ERROR;

		break;

		case INVERTER_STATE_STOP:

			/* During INVERTER_STATE_STOP the pwm are disabled and the system waits for command */

			/* begin - debug output GPIO */
//			GPIOC->BSRR = GPIO_PIN_9;
//			GPIOC->BSRR = (uint32_t)GPIO_PIN_9 << 16;
			/* end - debug output GPIO */

            global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
            global_sm_ctrl.brake_enable = BRAKE_DISABLE;
            
            if (global_sm_ctrl.inverter_cmd == INVERTER_CMD_RUN)
  			{
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_READY;
  			}
            else if (global_sm_ctrl.inverter_cmd == INVERTER_CMD_STANDBY)
  			{
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_READY;
			}
			else
            {
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_STOP;
            }

            /* if any error is present in the inverter */
            if (global_sm_ctrl.inverter_fault_state_description)
            {
                global_sm_ctrl.global_state = GLOBAL_STATE_ERROR;
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_ERROR;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
                global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
                global_sm_ctrl.brake_enable = BRAKE_DISABLE;
            }

        break;

		case INVERTER_STATE_READY:

			/* During INVERTER_STATE_READY the inverter control inject a id current to evaluate the rotor speed and to enable the braking
			 * procedure. */
            
            global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
            global_sm_ctrl.brake_enable = BRAKE_DISABLE;

            if (global_sm_ctrl.inverter_cmd == INVERTER_CMD_RUN)
  			{
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_CONNECTING;
  			}
            else if (global_sm_ctrl.inverter_cmd == INVERTER_CMD_STANDBY)
  			{
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_READY;
			}
			else
            {
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_STOP;
            }
            /* if any error is present in the inverter */
            if (global_sm_ctrl.inverter_fault_state_description)
            {
                global_sm_ctrl.global_state = GLOBAL_STATE_ERROR;
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_ERROR;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
                global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
                global_sm_ctrl.brake_enable = BRAKE_DISABLE;
            }


		break;

		case INVERTER_STATE_CONNECTING:
			/* During INVERTER_STATE_CONNECTING the filtered duty reference is above the threshold. ID set of current is controlled*/

            global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
            global_sm_ctrl.brake_enable = BRAKE_DISABLE;
            
            if (global_sm_ctrl.inverter_fault_state_description)
            {
                global_sm_ctrl.global_state = GLOBAL_STATE_ERROR;
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_ERROR;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
                global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
                global_sm_ctrl.brake_enable = BRAKE_DISABLE;
            }	
  			
            if (global_sm_ctrl.inverter_cmd == INVERTER_CMD_RUN)
  			{
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_RUN;
  			}
            else
  			{
//                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_CONNECTING;
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_DISCONNECTING;
			}

		break;

		case INVERTER_STATE_RUN:

			/* begin - debug output GPIO */
//			GPIOC->BSRR = GPIO_PIN_9;
//			GPIOC->BSRR = (uint32_t)GPIO_PIN_9 << 16;
			/* end - debug output GPIO */

            global_sm_ctrl.pwm_enable = INVERTER_PWM_START;
            global_sm_ctrl.brake_enable = BRAKE_DISABLE;
            
            if (global_sm_ctrl.inverter_fault_state_description)
            {
                global_sm_ctrl.global_state = GLOBAL_STATE_ERROR;
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_ERROR;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
                global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
                global_sm_ctrl.brake_enable = BRAKE_DISABLE;
            }	
  			
            if (global_sm_ctrl.inverter_cmd != INVERTER_CMD_RUN)
  			{
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_DISCONNECTING;
			}
			
		break;

        case INVERTER_STATE_DISCONNECTING:
        	/* during this state the rotor speed is monitored and is controlled in order to slow down the speed */

            global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
            global_sm_ctrl.brake_enable = BRAKE_DISABLE;

            global_cmd_word_generator_reset();
            inverter_ctrl_reset();

            if (global_sm_ctrl.inverter_fault_state_description)
            {
                global_sm_ctrl.global_state = GLOBAL_STATE_ERROR;
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_ERROR;
                global_sm_ctrl.inverter_cmd = INVERTER_CMD_STOP;
                global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
                global_sm_ctrl.brake_enable = BRAKE_DISABLE;
            }	
  			else
            {
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_STOP;
            }

        break;

        case INVERTER_STATE_ERROR:
            global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
            global_sm_ctrl.brake_enable = BRAKE_DISABLE;
            global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_ERROR;
            global_sm_ctrl.global_state = GLOBAL_STATE_ERROR;

            if (global_sm_ctrl.inverter_cmd == INVERTER_CMD_RESET)
            {
                global_sm_ctrl.inverter_fault_state_description &= INVERTER_NONE_ERROR;
                global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_STOP;
            }

        break;
	}
}

void inverter_ctrl_process(void)
{
	float th_sin;
	float th_cos;
	float i_inverter_alpha;
	float i_inverter_beta;
	float i_inverter_d;
	float i_inverter_q;
	float id_ref;
	float iq_ref;
	float udc;
	float dclink_voltage_compensation;
	float rotor_speed_error;
	float u_out_alpha;
	float u_out_beta;
	float omega_hat;
	float theta_signal;
	uint32_t dac_out;
	VECTORDQ udq_out;

	if (param.dclink_voltage_flt < MATH_HALF)
	{
		param.dclink_voltage_compensation = 1.0f;
	}
	else
	{
		param.dclink_voltage_compensation = (1.0f / param.dclink_voltage_flt);
	}


	if (param.dclink_voltage > param.dclink_overvoltage_fault_th)
	{
		/* system protections - DClink over-voltage*/
		param.global_state = INVERTER_STATE_ERROR;
		param.inverter_ctrl_state = INVERTER_STATE_ERROR;
		param.inverter_fault_state_description |= INVERTER_DCLINK_OVERVOLTAGE_ERROR;
		param.pwm_enable = INVERTER_PWM_STOP;
		param.brake_enable = BRAKE_DISABLE;

		/* force all gate mosfet command to low-state in low-impedance */
		inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
		inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
		inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
		inv_ctrl.inv_uab_sv_out.ua = 0;
		inv_ctrl.inv_uab_sv_out.ub = 0;
        sv_pwm_process(&inv_ctrl.inv_uab_sv_out);

		global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_ERROR;
		global_sm_ctrl.inverter_fault_state_description |= INVERTER_DCLINK_OVERVOLTAGE_ERROR;
		global_sm_ctrl.global_fault_state_description |= GLOBAL_STATE_INVERTER_ERROR;
	}

	if ((param.dclink_voltage < param.dclink_undervoltage_fault_th) && (param.inverter_ctrl_state == INVERTER_STATE_RUN))
	{
		/* system protections - DClink under-voltage*/
		param.global_state = INVERTER_STATE_ERROR;
		param.inverter_ctrl_state = INVERTER_STATE_ERROR;
		param.inverter_fault_state_description |= INVERTER_DCLINK_UNDERVOLTAGE_ERROR;
		param.pwm_enable = INVERTER_PWM_STOP;
		param.brake_enable = BRAKE_DISABLE;

		/* force all gate mosfet command to low-state in low-impedance */
		inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
		inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
		inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
		inv_ctrl.inv_uab_sv_out.ua = 0;
		inv_ctrl.inv_uab_sv_out.ub = 0;
        sv_pwm_process(&inv_ctrl.inv_uab_sv_out);

		global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_ERROR;
		global_sm_ctrl.inverter_fault_state_description |= INVERTER_DCLINK_UNDERVOLTAGE_ERROR;
		global_sm_ctrl.global_fault_state_description |= GLOBAL_STATE_INVERTER_ERROR;
	}

	if ((param.inverter_current_u > param.inverter_overcurrent_fault_th) || (param.inverter_current_u < -param.inverter_overcurrent_fault_th))
	{
		/* system protections - phase u overcurrent */
		param.global_state = INVERTER_STATE_ERROR;
		param.inverter_ctrl_state = INVERTER_STATE_ERROR;
		param.inverter_fault_state_description |= INVERTER_PHU_OVERCURRENT_ERROR;
		param.pwm_enable = INVERTER_PWM_STOP;
		param.brake_enable = BRAKE_DISABLE;

		/* force all gate mosfet command to low-state in low-impedance */
		inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
		inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
		inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
		inv_ctrl.inv_uab_sv_out.ua = 0;
		inv_ctrl.inv_uab_sv_out.ub = 0;
        sv_pwm_process(&inv_ctrl.inv_uab_sv_out);

		global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_ERROR;
		global_sm_ctrl.inverter_fault_state_description |= INVERTER_PHU_OVERCURRENT_ERROR;
		global_sm_ctrl.global_fault_state_description |= GLOBAL_STATE_INVERTER_ERROR;
	}
	if ((param.inverter_current_v > param.inverter_overcurrent_fault_th) || (param.inverter_current_v < -param.inverter_overcurrent_fault_th))
	{
		/* system protections - phase V overcurrent */
		param.global_state = INVERTER_STATE_ERROR;
		param.inverter_ctrl_state = INVERTER_STATE_ERROR;
		param.inverter_fault_state_description |= INVERTER_PHV_OVERCURRENT_ERROR;
		param.pwm_enable = INVERTER_PWM_STOP;
		param.brake_enable = BRAKE_DISABLE;

		/* force all gate mosfet command to low-state in low-impedance */
		inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
		inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
		inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
		inv_ctrl.inv_uab_sv_out.ua = 0;
		inv_ctrl.inv_uab_sv_out.ub = 0;
        sv_pwm_process(&inv_ctrl.inv_uab_sv_out);

		global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_ERROR;
		global_sm_ctrl.inverter_fault_state_description |= INVERTER_PHV_OVERCURRENT_ERROR;
		global_sm_ctrl.global_fault_state_description |= GLOBAL_STATE_INVERTER_ERROR;
	}
	if ((param.inverter_current_w > param.inverter_overcurrent_fault_th) || (param.inverter_current_w < -param.inverter_overcurrent_fault_th))
	{
		/* system protections - phase W over-current */
		param.global_state = INVERTER_STATE_ERROR;
		param.inverter_ctrl_state = INVERTER_STATE_ERROR;
		param.inverter_fault_state_description |= INVERTER_PHW_OVERCURRENT_ERROR;

		/* force all gate mosfet command to low-state in low-impedance */
		inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
		inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
		inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
		inv_ctrl.inv_uab_sv_out.ua = 0;
		inv_ctrl.inv_uab_sv_out.ub = 0;
        sv_pwm_process(&inv_ctrl.inv_uab_sv_out);

		global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
		global_sm_ctrl.brake_enable = BRAKE_DISABLE;

		global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_ERROR;
		global_sm_ctrl.inverter_fault_state_description |= INVERTER_PHW_OVERCURRENT_ERROR;
		global_sm_ctrl.global_fault_state_description |= GLOBAL_STATE_INVERTER_ERROR;
	}

	if ((param.omega_hat_pu > param.omega_max_th) || (param.omega_hat_pu < -param.omega_max_th))
	{
		/* system protections - motor over-speed */
		param.global_state = INVERTER_STATE_ERROR;
		param.inverter_ctrl_state = INVERTER_STATE_ERROR;
		param.inverter_fault_state_description |= MOTOR_OVERSPEED_ERROR;

		/* force all gate mosfet command to low-state in low-impedance */
		inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
		inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
		inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
		inv_ctrl.inv_uab_sv_out.ua = 0;
		inv_ctrl.inv_uab_sv_out.ub = 0;
        sv_pwm_process(&inv_ctrl.inv_uab_sv_out);

		global_sm_ctrl.pwm_enable = INVERTER_PWM_STOP;
		global_sm_ctrl.brake_enable = BRAKE_DISABLE;

		global_sm_ctrl.inverter_ctrl_state = INVERTER_STATE_ERROR;
		global_sm_ctrl.inverter_fault_state_description |= INVERTER_PHW_OVERCURRENT_ERROR;
		global_sm_ctrl.global_fault_state_description |= GLOBAL_STATE_INVERTER_ERROR;
	}

	if (global_sm_ctrl.inverter_ctrl_state == INVERTER_STATE_RUN)
	{

		switch (global_sm_ctrl.inverter_ctrl_mode)
			{
				case INVERTER_CTRL_SPEED:

					th_sin = sinf(param.theta_hat);
					th_cos = cosf(param.theta_hat);

					param.sin_theta = th_sin;
					param.cos_theta = th_cos;

					i_inverter_alpha = param.inverter_current_u * MATH_2_3 -(param.inverter_current_v + param.inverter_current_w) * MATH_1_3;
					i_inverter_beta = (param.inverter_current_v - param.inverter_current_w) * MATH_1_SQRT3;

					i_inverter_d = i_inverter_alpha * th_cos + i_inverter_beta * th_sin;
					i_inverter_q = i_inverter_beta * th_cos - i_inverter_alpha * th_sin;

					param.inverter_current_d = i_inverter_d;
					param.inverter_current_q = i_inverter_q;

					udc = param.dclink_voltage_flt;
					dclink_voltage_compensation = param.dclink_voltage_compensation;

					rotor_speed_error = param.omega_ref_pu - param.omega_hat_pu;
					regpi_process(&inv_ctrl.inv_speed_ctrl, rotor_speed_error, 0, 1);

#ifdef PMSM_LOAD
					iq_ref = param.motor_torque_sign*inv_ctrl.inv_speed_ctrl.sig_out;
					id_ref = param.id_ext_ref;
#endif

					udq_out = dqvector_pi_process(&inv_ctrl.inv_cv_ctrl, id_ref, i_inverter_d, iq_ref, i_inverter_q, dclink_voltage_compensation);

					param.iq_ref_fb = iq_ref;
					param.id_ref_fb = id_ref;

					u_out_alpha = udq_out.d * th_cos - udq_out.q * th_sin;
					u_out_beta = udq_out.q * th_cos + udq_out.d * th_sin;
					param.motor_voltage_pu = inv_ctrl.inv_cv_ctrl.u_out;

//					omega_hat = bemf_obsv_load_est_process(&inv_ctrl.inv_obsv_load_est, u_out_alpha, u_out_beta, i_inverter_alpha, i_inverter_beta, udc, i_inverter_q);
//					param.theta_hat = inv_ctrl.inv_obsv_load_est.theta_hat;

					omega_hat = bemf_obsv_process(&inv_ctrl.inv_obsv, u_out_alpha, u_out_beta, i_inverter_alpha, i_inverter_beta, udc);
					param.theta_hat = inv_ctrl.inv_obsv.theta_hat;

					param.omega_hat = omega_hat*param.motorc_omega_bez;
					param.omega_hat_pu = omega_hat;

					inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
					inv_ctrl.inv_uab_sv_out.ua = u_out_alpha;
					inv_ctrl.inv_uab_sv_out.ub = u_out_beta;
					inv_ctrl.inv_uab_sv_out.pwm_enable = global_sm_ctrl.pwm_enable;
					inv_ctrl.inv_uab_sv_out.brake_enable = global_sm_ctrl.brake_enable;

					/* calculate sv pwm */
					sv_pwm_process(&inv_ctrl.inv_uab_sv_out);

					/* update global variables */
					param.mu = inv_ctrl.inv_uab_sv_out.du;
					param.mv = inv_ctrl.inv_uab_sv_out.dv;
					param.mw = inv_ctrl.inv_uab_sv_out.dw;

				break;

				case INVERTER_CTRL_TORQUE:

					th_sin = sinf(param.theta_hat);
					th_cos = cosf(param.theta_hat);

					param.sin_theta = th_sin;
					param.cos_theta = th_cos;

					i_inverter_alpha = param.inverter_current_u * MATH_2_3 -(param.inverter_current_v + param.inverter_current_w) * MATH_1_3;
					i_inverter_beta = (param.inverter_current_v - param.inverter_current_w) * MATH_1_SQRT3;

					i_inverter_d = i_inverter_alpha * th_cos + i_inverter_beta * th_sin;
					i_inverter_q = i_inverter_beta * th_cos - i_inverter_alpha * th_sin;

					param.inverter_current_d = i_inverter_d;
					param.inverter_current_q = i_inverter_q;

					udc = param.dclink_voltage_flt;
					dclink_voltage_compensation = param.dclink_voltage_compensation;

					param.torque_ref = param.duty_reference_flt;
					param.id_ext_ref = 0.14 * param.duty_reference_flt;

					iq_ref = param.torque_ref;
					param.iq_ref_fb = iq_ref;
					id_ref = param.id_ext_ref;
					udq_out = dqvector_pi_process(&inv_ctrl.inv_cv_ctrl, id_ref, i_inverter_d, iq_ref, i_inverter_q, dclink_voltage_compensation);

					u_out_alpha = udq_out.d * th_cos - udq_out.q * th_sin;
					u_out_beta = udq_out.q * th_cos + udq_out.d * th_sin;
					param.motor_voltage_pu = inv_ctrl.inv_cv_ctrl.u_out;

//					omega_hat = bemf_obsv_load_est_process(&inv_ctrl.inv_obsv_load_est, u_out_alpha, u_out_beta, i_inverter_alpha, i_inverter_beta, udc, i_inverter_q);
//					param.theta_hat = inv_ctrl.inv_obsv_load_est.theta_hat;

					omega_hat = bemf_obsv_process(&inv_ctrl.inv_obsv, u_out_alpha, u_out_beta, i_inverter_alpha, i_inverter_beta, udc);
					param.theta_hat = inv_ctrl.inv_obsv.theta_hat;

					param.omega_hat = omega_hat*param.motorc_omega_bez;
					param.omega_hat_pu = omega_hat;

					inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
					inv_ctrl.inv_uab_sv_out.ua = u_out_alpha;
					inv_ctrl.inv_uab_sv_out.ub = u_out_beta;
					inv_ctrl.inv_uab_sv_out.pwm_enable = global_sm_ctrl.pwm_enable;
					inv_ctrl.inv_uab_sv_out.brake_enable = global_sm_ctrl.brake_enable;

					sv_pwm_process(&inv_ctrl.inv_uab_sv_out);

				break;

				case INVERTER_CTRL_I_F:

					signal_generator_process(&phase_signal, 1.0, param.signal_generator_freq_bez);
					theta_signal = phase_signal.sig_out;
					param.signal_generator_output = theta_signal;

					th_sin = sinf(theta_signal);
					th_cos = cosf(theta_signal);

				    param.sin_theta = th_sin;
				    param.cos_theta = th_cos;

				    i_inverter_alpha = param.inverter_current_u * MATH_2_3 -(param.inverter_current_v + param.inverter_current_w) * MATH_1_3;
				    i_inverter_beta = (param.inverter_current_v - param.inverter_current_w) * MATH_1_SQRT3;

				    i_inverter_d = i_inverter_alpha * th_cos + i_inverter_beta * th_sin;
				    i_inverter_q = i_inverter_beta * th_cos - i_inverter_alpha * th_sin;

				    param.inverter_current_d = i_inverter_d;
				    param.inverter_current_q = i_inverter_q;

					udc = param.dclink_voltage_flt;
					dclink_voltage_compensation = param.dclink_voltage_compensation;

					param.id_ext_ref = param.duty_reference_flt;
					id_ref = param.id_ext_ref;
					iq_ref = param.iq_ext_ref;

					udq_out = dqvector_pi_process(&inv_ctrl.inv_cv_ctrl, id_ref, i_inverter_d, iq_ref, i_inverter_q, dclink_voltage_compensation);

					u_out_alpha = udq_out.d * th_cos - udq_out.q * th_sin;
					u_out_beta = udq_out.q * th_cos + udq_out.d * th_sin;
					param.motor_voltage_pu = inv_ctrl.inv_cv_ctrl.u_out;

//					omega_hat = bemf_obsv_load_est_process(&inv_ctrl.inv_obsv_load_est, u_out_alpha, u_out_beta, i_inverter_alpha, i_inverter_beta, udc, i_inverter_q);
//					param.theta_hat = inv_ctrl.inv_obsv_load_est.theta_hat;

					omega_hat = bemf_obsv_process(&inv_ctrl.inv_obsv, u_out_alpha, u_out_beta, i_inverter_alpha, i_inverter_beta, udc);
					param.theta_hat = inv_ctrl.inv_obsv.theta_hat;

					param.omega_hat = omega_hat*param.motorc_omega_bez;
					param.omega_hat_pu = omega_hat;

					inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
					inv_ctrl.inv_uab_sv_out.ua = u_out_alpha;
					inv_ctrl.inv_uab_sv_out.ub = u_out_beta;
					inv_ctrl.inv_uab_sv_out.pwm_enable = global_sm_ctrl.pwm_enable;
					inv_ctrl.inv_uab_sv_out.brake_enable = global_sm_ctrl.brake_enable;

					sv_pwm_process(&inv_ctrl.inv_uab_sv_out);

					break;

				case INVERTER_CTRL_U_F:

					signal_generator_process(&phase_signal, 1.0, param.signal_generator_freq_bez);
					theta_signal = phase_signal.sig_out;
					param.signal_generator_output = theta_signal;

					th_sin = sinf(theta_signal);
					th_cos = cosf(theta_signal);

					param.sin_theta = th_sin;
					param.cos_theta = th_cos;

					i_inverter_alpha = param.inverter_current_u * MATH_2_3 -(param.inverter_current_v + param.inverter_current_w) * MATH_1_3;
					i_inverter_beta = (param.inverter_current_v - param.inverter_current_w) * MATH_1_SQRT3;

					i_inverter_d = i_inverter_alpha * th_cos + i_inverter_beta * th_sin;
					i_inverter_q = i_inverter_beta * th_cos - i_inverter_alpha * th_sin;

					param.inverter_current_d = i_inverter_d;
					param.inverter_current_q = i_inverter_q;

					/* ud = signal_generator_u_ref_out, and uq = 0, results in alpha beta as follows */
					u_out_alpha = param.signal_generator_u_ref_out * th_cos;
					u_out_beta = param.signal_generator_u_ref_out * th_sin;
					param.motor_voltage_pu = sqrtf(u_out_alpha*u_out_alpha + u_out_beta*u_out_beta);

					udc = param.dclink_voltage_flt;
					dclink_voltage_compensation = param.dclink_voltage_compensation;

//					omega_hat = bemf_obsv_load_est_process(&inv_ctrl.inv_obsv_load_est, u_out_alpha, u_out_beta, i_inverter_alpha, i_inverter_beta, udc, i_inverter_q);
//					param.theta_hat = inv_ctrl.inv_obsv_load_est.theta_hat;

					omega_hat = bemf_obsv_process(&inv_ctrl.inv_obsv, u_out_alpha, u_out_beta, i_inverter_alpha, i_inverter_beta, udc);
					param.theta_hat = inv_ctrl.inv_obsv.theta_hat;

					param.omega_hat = omega_hat*param.motorc_omega_bez;
					param.omega_hat_pu = omega_hat;

					inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
					inv_ctrl.inv_uab_sv_out.ua = u_out_alpha;
					inv_ctrl.inv_uab_sv_out.ub = u_out_beta;
					inv_ctrl.inv_uab_sv_out.pwm_enable = global_sm_ctrl.pwm_enable;
					inv_ctrl.inv_uab_sv_out.brake_enable = global_sm_ctrl.brake_enable;

					sv_pwm_process(&inv_ctrl.inv_uab_sv_out);

					dac_out = (uint32_t)((MATH_PI + param.theta_hat)/MATH_2PI * 4095);
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_out);

				break;
				case INVERTER_DOUBLE_PULSE_TEST:

					global_sm_ctrl.pwm_enable = 0;
					global_sm_ctrl.brake_enable = 0;
					inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
					inv_ctrl.inv_uab_sv_out.ua = 0;
					inv_ctrl.inv_uab_sv_out.ub = 0;
					inv_ctrl.inv_uab_sv_out.pwm_enable = global_sm_ctrl.pwm_enable;
					inv_ctrl.inv_uab_sv_out.brake_enable = global_sm_ctrl.brake_enable;

					double_pulse_test_process(&double_pulse_test);
				break;

			}
	}

	if (global_sm_ctrl.inverter_ctrl_state == INVERTER_STATE_READY)
	{
		inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
		inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
		inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
		inv_ctrl.inv_uab_sv_out.ua = 0;
		inv_ctrl.inv_uab_sv_out.ub = 0;
        sv_pwm_process(&inv_ctrl.inv_uab_sv_out);
	}

	if (global_sm_ctrl.inverter_ctrl_state == INVERTER_STATE_ERROR)
	{
		inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
		inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
		inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
		inv_ctrl.inv_uab_sv_out.ua = 0;
		inv_ctrl.inv_uab_sv_out.ub = 0;
        sv_pwm_process(&inv_ctrl.inv_uab_sv_out);
	}

	if (global_sm_ctrl.inverter_ctrl_state == INVERTER_STATE_CONNECTING)
	{

		switch (global_sm_ctrl.inverter_ctrl_mode)
			{
				case INVERTER_DOUBLE_PULSE_TEST:
					inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
					inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
					inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
					inv_ctrl.inv_uab_sv_out.ua = 0;
					inv_ctrl.inv_uab_sv_out.ub = 0;
			        sv_pwm_process(&inv_ctrl.inv_uab_sv_out);
			        break;

				default:
					inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
					inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
					inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
					inv_ctrl.inv_uab_sv_out.ua = 0;
					inv_ctrl.inv_uab_sv_out.ub = 0;
			        sv_pwm_process(&inv_ctrl.inv_uab_sv_out);
			}
	}

	if (global_sm_ctrl.inverter_ctrl_state == INVERTER_STATE_DISCONNECTING)
	{

		switch (global_sm_ctrl.inverter_ctrl_mode)
			{
				case INVERTER_DOUBLE_PULSE_TEST:
					inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
					inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
					inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
					inv_ctrl.inv_uab_sv_out.ua = 0;
					inv_ctrl.inv_uab_sv_out.ub = 0;
			        sv_pwm_process(&inv_ctrl.inv_uab_sv_out);
			        break;

				default:
					inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_START;
					inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
					inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
					inv_ctrl.inv_uab_sv_out.ua = 0;
					inv_ctrl.inv_uab_sv_out.ub = 0;
			        sv_pwm_process(&inv_ctrl.inv_uab_sv_out);
			}
	}


	if ((global_sm_ctrl.inverter_ctrl_state == INVERTER_STATE_STOP)||(global_sm_ctrl.inverter_ctrl_state == INVERTER_STATE_RESUME))
	{

		/* force all gate mosfet command to low-state in low-impedance */
		inv_ctrl.inv_uab_sv_out.pwm_enable = INVERTER_PWM_STOP;
		inv_ctrl.inv_uab_sv_out.brake_enable = BRAKE_DISABLE;
		inv_ctrl.inv_uab_sv_out.ts = param.inverter_t_pwm;
		inv_ctrl.inv_uab_sv_out.ua = 0;
		inv_ctrl.inv_uab_sv_out.ub = 0;
        sv_pwm_process(&inv_ctrl.inv_uab_sv_out);
	}
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//	/*
//	 * ADC1 ch3 -> adc1_buf[1] -> udc 			(PA3)
//	 * ADC2 ch3 -> adc2_buf[1] -> udc 			(PA3)
//	 * ADC3 ch3 -> adc3_buf[1] -> udc 			(PA3)
//	 *
//	 * ADC1 ch0 -> adc1_buf[0] -> is_u 			(PA0)
//	 * ADC2 ch0 -> adc2_buf[0] -> is_u 			(PA0)
//	 * ADC3 ch0 -> adc3_buf[0] -> is_u 			(PA0)
//	 *
//	 * ADC1 ch4 -> adc1_buf[2] -> is_v 			(PA4)
//	 * ADC2 ch4 -> adc2_buf[2] -> is_v 			(PA4)
//	 *
//	 * ADC3 ch5 -> adc3_buf[2] -> is_w 			(PF7)
//	 *
//	 * ADC1 ch9 -> adc1_buf[3] -> Tntcs_u 		(PB1)
//	 * ADC3 ch7 -> adc3_buf[3] -> Tntcs_v 		(PF9)
//	 * ADC2 ch6 -> adc2_buf[3] -> Tntcs_w 		(PA6)
//	 *
//	 * ADC3 ch10 -> adc3_buf[5] -> Tambient  	(PC0)
//	 * ADC3 ch8 -> adc3_buf[4] -> Ths  			(PF10)
//	 * */
//	if (hadc == &hadc3)
//	{
//		param.inverter_current_u = -((float)(param.inverter_current_scale * (adc1_buf[0] + adc2_buf[0] + adc3_buf[0])/3) + param.inverter_current_v_offset);
//		param.inverter_current_v = -((float)(param.inverter_current_scale * (adc1_buf[2] + adc2_buf[2])/2) + param.inverter_current_u_offset);
//		param.inverter_current_w = -((float)(param.inverter_current_scale * adc3_buf[2]) + param.inverter_current_w_offset);
//		param.dclink_voltage = (float)(param.dclink_voltage_scale * (adc1_buf[1] + adc2_buf[1] + adc3_buf[1])/3);
//
//		param.temperature_igbt_inverter_u = (float)(param.temperature_igbt_inverter_scale * adc1_buf[3]);
//		param.temperature_igbt_inverter_w = (float)(param.temperature_igbt_inverter_scale * adc2_buf[3]);
//		param.temperature_igbt_inverter_v = (float)(param.temperature_igbt_inverter_scale * adc3_buf[3]);
//
//		param.temperature_ambient = (float)(1.0/ADC_RESOLUTION * adc3_buf[5]);
//		param.temperature_heatsink = (float)(1.0/ADC_RESOLUTION * adc3_buf[4]);
//	}
//}

#ifdef EXT_OVERCURRENT_INTERRUPTS
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_11)
	{
		param.pwm_enable = INVERTER_PWM_STOP;
		param.inverter_ctrl_state = INVERTER_STATE_ERROR;
		param.inverter_fault_state_description |= INVERTER_PHU_OVERCURRENT_ERROR;
	}

	if (GPIO_Pin == GPIO_PIN_12)
	{
		param.pwm_enable = INVERTER_PWM_STOP;
		param.inverter_ctrl_state = INVERTER_STATE_ERROR;
		param.inverter_fault_state_description |= INVERTER_PHV_OVERCURRENT_ERROR;
	}

	if (GPIO_Pin == GPIO_PIN_13)
	{
		param.pwm_enable = INVERTER_PWM_STOP;
		param.inverter_ctrl_state = INVERTER_STATE_ERROR;
		param.inverter_fault_state_description |= INVERTER_PHW_OVERCURRENT_ERROR;
	}
}
#endif

