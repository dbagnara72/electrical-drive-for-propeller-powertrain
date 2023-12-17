#include "fb_application_config.h"
#include "bemf_obsv_load_est.h"
#include "bemf_obsv.h"
#include "dqvector_pi.h"
#include "sv_pwm.h"
#include "regpi.h"
#include "fb_can.h"
#include "adc.h"
#include "dac.h"
#include "gpio.h"
#include "signal_generator.h"
#include "fltlp.h"
#include "iwdg.h"

#ifndef _GLOBAL_STATE_MACHINE_
#define _GLOBAL_STATE_MACHINE_

/* Number of channels for each ADC */
#define ADC1_BUF_LEN 					4
#define ADC2_BUF_LEN 					4
#define ADC3_BUF_LEN 					6

/* ADC resolutions */
#ifdef MCI_EPL_INVERTER
#define ADC_RESOLUTION 					4095.0f /* 12 bits */
#define ADC_RESOLUTION_1				0.000244200244200244f /* inverse 12 bits scaling */
#endif

#define ADC_CALIBRATION_SAMPLES			5000

/* MAD Motor Data*/
//#ifdef MAD_MOTOR
//#define MOTOR_MAXIMUM_CURRENT 			64.00f					/* Phase peak A */
//#define MOTOR_NOMINAL_CURRENT 			64.00f					/* Phase peak A */
//#define MOTOR_MAXIMUM_VOLTAGE 			326.6f					/* Phase peak V */
//#define MOTOR_NOMINAL_VOLTAGE 			161.7f					/* Phase peak V of the back EMF at nominal speed*/
//#define MOTOR_POLE_PAIRS	 			31.0f						/* */
//#define MOTOR_NOMINAL_SPEED 			2291.0f					/* Mechanical rotor nominal speed in RPM */
//#define MOTOR_NOMINAL_PHI_M 			0.0217361189955227f 		/* Phase peak Vs - nominal value of the linkage PM flux*/
//#define MOTOR_NOMINAL_TORQUE 			64.0f			 		/* in Nm */
//#define MOTOR_NOMINAL_LA 				0.000120f			 	/* in H */
//#define MOTOR_NOMINAL_LB 				0.000020f			 	/* in H */
//#define MOTOR_NOMINAL_RS 				0.6f			 			/* in Ohm */
//#define MOTOR_NOMINAL_LD 				(MATH_3_2 * (MOTOR_NOMINAL_LA - MOTOR_NOMINAL_LB))	/* in H */
//#define MOTOR_NOMINAL_LQ 				(MATH_3_2 * (MOTOR_NOMINAL_LA + MOTOR_NOMINAL_LB))	/* in H */
//#define MOTOR_NOMINAL_LS 				MOTOR_NOMINAL_LD		/* in H */
//#define MOTOR_NOMINAL_JM				0.015f					/* in kg m^2*/
//#endif

#ifdef MAD_MOTOR
#define MOTOR_MAXIMUM_CURRENT 			64.00f					/* Phase peak A */
#define MOTOR_NOMINAL_CURRENT 			64.00f					/* Phase peak A */
#define MOTOR_MAXIMUM_VOLTAGE 			326.6f					/* Phase peak V */
#define MOTOR_NOMINAL_VOLTAGE 			220.0f					/* Phase peak V of the back EMF at nominal speed*/
#define MOTOR_POLE_PAIRS	 			31.0f					/* */
#define MOTOR_NOMINAL_SPEED 			2291.0f					/* Mechanical rotor nominal speed in RPM */
#define MOTOR_NOMINAL_PHI_M 			0.0191f			 		/* Phase peak Vs - nominal value of the linkage PM flux*/
#define MOTOR_NOMINAL_TORQUE 			64.0f			 		/* in Nm */
#define MOTOR_NOMINAL_LA 				0.000080f			 	/* in H */
#define MOTOR_NOMINAL_LB 				0.000020f			 	/* in H */
#define MOTOR_NOMINAL_RS 				0.2f		 			/* in Ohm */
#define MOTOR_NOMINAL_LD 				(MATH_3_2 * (MOTOR_NOMINAL_LA - MOTOR_NOMINAL_LB))	/* in H */
#define MOTOR_NOMINAL_LQ 				(MATH_3_2 * (MOTOR_NOMINAL_LA + MOTOR_NOMINAL_LB))	/* in H */
#define MOTOR_NOMINAL_LS 				MOTOR_NOMINAL_LD		/* in H */
#define MOTOR_NOMINAL_JM				0.015f					/* in kg m^2*/
#endif

#ifdef MAD_MOTOR
/* INVERTER Data */
#define INVERTER_MAXIMUM_CURRENT 			96.00f			/* Phase peak A - end scale of the current sensor */
#define INVERTER_NOMINAL_CURRENT 			64.00f			/* Phase peak A - is set equal to motor nominal current*/
#define INVERTER_MAXIMUM_VOLTAGE 			326.6f			/* Phase peak V */
#define INVERTER_NOMINAL_VOLTAGE 			326.6f			/* Phase peak V */
#define INVERTER_MAXIMUM_TEMPERATURE 		86.5f			/* in °C - used for alarm */
#define INVERTER_NOMINAL_TEMPERATURE 		110.0f			/* in °C - scaling NTC temperature */
#define INVERTER_OVERCURRENT_FAULT_TH		1.25f			/* Phase overcurrent fault threshold in per unit */
#define INVERTER_OVERTEMPERATURE_FAULT_TH	86.0f			/* Phase overtemperature fault threshold in per unit */
#define MOTOR_OVERSPEED_FAULT_TH			2.0f				/* motor overspeed fault threshold in per unit */
#endif

#ifdef MAD_MOTOR
/* DCLink Data */
#define DCLINK_MAXIMUM_VOLTAGE 			648.0f				/* Phase peak V - end scale of the voltage sensor*/
#define DCLINK_NOMINAL_VOLTAGE 			400.0f				/* Phase peak V */
#define DCLINK_OVERVOLTAGE_FAULT_TH		1.2f					/* DClink overvoltage fault threshold in per unit */
#define DCLINK_UNDERVOLTAGE_FAULT_TH 	0.0f					/* DClink undervoltage fault threshold in per unit */
#endif


/* Global state machine command word */
#define GLOBAL_CMD_STOP                 1
#define GLOBAL_CMD_STANDBY              2
#define GLOBAL_CMD_RUN                  3
#define GLOBAL_CMD_RESET                5

/* Global state machine status word */
#define GLOBAL_STATE_RESUME				0
#define GLOBAL_STATE_STOP				1
#define GLOBAL_STATE_READY			    2
#define GLOBAL_STATE_CONNECTING         5
#define GLOBAL_STATE_RUN		        4
#define GLOBAL_STATE_DISCONNECTING      9
#define GLOBAL_STATE_ERROR              11

/* Auto-generation of the command word state machine*/
#define GLOBAL_CMD_WORD_GENERATION_WAIT	    	7
#define GLOBAL_CMD_WORD_GENERATION_RESET       	5
#define GLOBAL_CMD_WORD_GENERATION_STOP        	1
#define GLOBAL_CMD_WORD_GENERATION_STANDBY     	2
#define GLOBAL_CMD_WORD_GENERATION_RUN         	3

/* Inverter state machine command word */
#define INVERTER_CMD_STOP               1
#define INVERTER_CMD_STANDBY            2
#define INVERTER_CMD_RUN                3
#define INVERTER_CMD_RESET              5

/* Inverter state machine status word */
#define INVERTER_STATE_RESUME           0
#define INVERTER_STATE_STOP             1
#define INVERTER_STATE_READY            2
#define INVERTER_STATE_CONNECTING       5
#define INVERTER_STATE_RUN              4
#define INVERTER_STATE_DISCONNECTING    9
#define INVERTER_STATE_ERROR            11

/* Inverter PWM enable/disable */
#define INVERTER_PWM_START              1
#define INVERTER_PWM_STOP               0

/* brake enable/disable */
#define BRAKE_ENABLE              		1
#define BRAKE_DISABLE               	0

/* Define for the GLOBAL CMD WORD generation */
#define UDC_ENABLE_CMD_STANDBY          0.70f
#define UDC_ENABLE_CMD_STOP          	0.5f
#define DUTY_ENABLE_CMD_RUN             0.04f
#define DUTY_ENABLE_CMD_STANDBY         0.02f
#define TIMEOUT_AUTO_RESET		        250000

/* Inverter control modes */
#define INVERTER_CTRL_TORQUE         	1
#define INVERTER_CTRL_U_F            	2
#define INVERTER_CTRL_I_F            	3
#define INVERTER_CTRL_SPEED          	4
#define INVERTER_DOUBLE_PULSE_TEST      21

/* Inverter faults list */
#define INVERTER_DCLINK_OVERVOLTAGE_ERROR       		0x0001		/* error code 1 */
#define INVERTER_DCLINK_UNDERVOLTAGE_ERROR      		0x0002		/* error code 2 */
#define INVERTER_PHU_OVERCURRENT_ERROR          		0x0004		/* error code 4 */
#define INVERTER_PHV_OVERCURRENT_ERROR          		0x0008		/* error code 8 */
#define INVERTER_PHW_OVERCURRENT_ERROR          		0x0010		/* error code 16 */
#define INVERTER_PHU_DRIVER_ERROR               		0x0020		/* error code 32 */
#define INVERTER_PHV_DRIVER_ERROR               		0x0040		/* error code 64 */
#define INVERTER_PHW_DRIVER_ERROR               		0x0080		/* error code 128 */
#define INVERTER_IGBT_PHU_OVERTEMPERATURE_ERROR     	0x0100		/* error code 256 */
#define INVERTER_IGBT_PHV_OVERTEMPERATURE_ERROR     	0x0200		/* error code 512 */
#define INVERTER_IGBT_PHW_OVERTEMPERATURE_ERROR     	0x0400		/* error code 1024 */
#define MOTOR_OVERSPEED_ERROR     						0x0800		/* error code 2048 */
#define INVERTER_NONE_ERROR                     		0x0000		/* error code 0 */

/* Global state machine faults list */
#define GLOBAL_STATE_DCLINK_OVERVOLTAGE_ERROR       	0x0001		/* error code 1 */
#define GLOBAL_STATE_DCLINK_UNDERVOLTAGE_ERROR      	0x0002		/* error code 2 */
#define GLOBAL_STATE_INVERTER_ERROR     				0x0004		/* error code 4 */
#define GLOBAL_STATE_MOTOR_OVERTEMPERATURE_ERROR     	0x0008		/* error code 8 */
#define GLOBAL_STATE_HEATSINK_OVERTEMPERATURE_ERROR     0x0010		/* error code 16 */
#define GLOBAL_STATE_NONE_ERROR                     	0x0000

/* List of debug modes */

/* debug mode with U/f control and state OBSERVER */
#define DEBUG_MODE_CTRL_U_F         	1

/* debug mode with #define I/f control and state OBSERVER */
#define DEBUG_MODE_CTRL_I_F         	2

/* debug mode in torque control, vector control and state OBSERVER */
#define DEBUG_MODE_CTRL_TORQUE      	3

/* debug mode with Speed control, vector control and state OBSERVER */
#define DEBUG_MODE_CTRL_SPEED      		4

/* double pulse test */
#define DEBUG_MODE_DOUBLE_PULSE_TEST	5


/* Inverter configuration */
/* SVPWM output voltage limit (default is 1 which corresponds a output voltage v_ph2ph_rms = vdc/sqrt(2)) */
#define SVPWM_OUT_LIM        1.0f
#define TORQUE_PU_LIM        0.15f /* this parameter limit the output torque reference from the speed loop */



/* Global structures definition */

typedef struct state_machine_s {
	/* global state machine variables */
	uint8_t 	global_state;							/* status of the global state machine */
    uint32_t 	global_fault_state_description;			/* error status word of the global state machine: this word collects all active errors */
    uint8_t 	global_cmd;								/* global command word */
    uint8_t 	inverter_ctrl_state;					/* status of the inverter state machine */
    uint8_t 	inverter_ctrl_mode;						/* status of the inverter control mode: speed, torque, U/f or I/f control */
    uint32_t 	inverter_fault_state_description;		/* error status word of the inverter state machine: this word collects all active errors */
    uint8_t 	inverter_cmd;							/* inverter command word: this word is generated by the global state machine */
    uint8_t 	pwm_enable;
    uint8_t 	brake_enable;
} state_machine_t;
#define STATE_MACHINE state_machine_t

typedef struct param_s {
	/* global variables */

    float iq_ref_fb;
    float id_ref_fb;

	/* ADC measure and scaling
	 * *_max: this value identifies the end-scale of the sensor
	 * *_nom: this value identifies the nominal value used in the firmware
	 * *_scale: this value identifies the scaling from the integer of the ADC and the per unit used in the firmware
	 * *_offset: this value identifies the offset from the integer of the ADC and the per unit used in the firmware
	 */
    float temperature_igbt_inverter_max;		/* this value identifies the end-scale of the sensor */
    float temperature_igbt_inverter_nom;		/* this value identifies the nominal value used in the firmware */
    float temperature_igbt_inverter_scale;		/* this value identifies the scaling from the integer of the ADC and the per unit used in the firmware */
    float temperature_igbt_inverter_offset;		/* this value identifies the offset from the integer of the ADC and the per unit used in the firmware */

    float inverter_current_max;					/* this value identifies the end-scale of the sensor */
    float inverter_current_nom;					/* this value identifies the nominal value used in the firmware */
    float inverter_current_scale;				/* this value identifies the scaling from the integer of the ADC and the per unit used in the firmware */
    float inverter_current_offset;				/* this value identifies the offset from the integer of the ADC and the per unit used in the firmware */
    float inverter_current_u_offset;			/* this value identifies the offset from the integer of the ADC and the per unit used in the firmware */
    float inverter_current_v_offset;			/* this value identifies the offset from the integer of the ADC and the per unit used in the firmware */
    float inverter_current_w_offset;			/* this value identifies the offset from the integer of the ADC and the per unit used in the firmware */

    float inverter_voltage_max;
    float inverter_voltage_nom;
    float inverter_voltage_scale;
    float inverter_voltage_offset;

    float motor_voltage_ud_pu;
    float motor_voltage_uq_pu;
    float motor_voltage_u_pu;

    float dclink_voltage_max;					/* this value identifies the end-scale of the sensor */
    float dclink_voltage_nom;					/* this value identifies the nominal value used in the firmware */
    float dclink_voltage_scale;					/* this value identifies the scaling from the integer of the ADC and the per unit used in the firmware */
    float dclink_voltage_offset;				/* this value identifies the offset from the integer of the ADC and the per unit used in the firmware */

    float motor_voltage_max;
    float motor_voltage_nom;					/* this variable is used for motor parameters normalization */
    float motor_voltage_scale;
    float motor_voltage_offset;

    float motor_current_max;
    float motor_current_nom;					/* this variable is used for motor parameters normalization */
    float motor_current_scale;
    float motor_current_offset;

    float motor_torque_sign;					/* this variable is used to change the sign of the torque reference generated by the speed loop */

    /* ADC variables */
    float inverter_current_u;
    float inverter_current_v;
    float inverter_current_w;
    float dclink_voltage;
    float temperature_igbt_inverter_u;
    float temperature_igbt_inverter_v;
    float temperature_igbt_inverter_w;
    float temperature_ambient;
    float temperature_heatsink;
    float temperature_motor;

    /* PWM period: it corresponds to the period of the main control process */
    float inverter_t_pwm;
    float base_freq_inv;

    /* variables used into the control algorithms */
    float inverter_theta_hat_sin;
    float inverter_theta_hat_cos;
    float inverter_current_d;
    float inverter_current_q;
    float inverter_current_alpha;
    float inverter_current_beta;
    float torque_lim_top;
    float torque_lim_bottom;
    float omega_ref_pu;
    float omega_hat;
    float omega_hat_pu;
    float theta_hat;
    float torque_hat;
    float torque_ref;
    float cos_theta;
    float sin_theta;
    float dclink_voltage_compensation;
    float dclink_voltage_flt;
    float u_out_lim;
    float motor_voltage_pu;

    /* speed control PI gains */
    float kp_omega;
    float ki_omega;

    /* currents control PI gains */
    float kp_inv_id;
    float ki_inv_id;
    float kp_inv_iq;
    float ki_inv_iq;

    /* state observer gains */
    float bemf_obsv_kalman_omega;
    float bemf_obsv_kalman_theta;
	float bemf_obsv_luenberger_1;
	float bemf_obsv_luenberger_2;
	float bemf_obsv_luenberger_3;
    float bemf_obsv_fb_p;
    float bemf_obsv_p;

    /* variables used as references */
    float inverter_current_reference_q;
    float inverter_current_reference_d;
	float id_ext_ref;
	float iq_ext_ref;

    /* variables used as fault thresholds */
	float inverter_overcurrent_fault_th;
	float inverter_overtemperature_fault_th;
	float dclink_overvoltage_fault_th;
	float dclink_undervoltage_fault_th;
	float omega_max_th;

	/* variables used for normalization of the PMSM motor */
    float motorc_omega_bez;
    float motorc_m_scale;
    float motorc_rs_norm;
    float motorc_ls_norm;
    float motorc_ld_norm;
    float motorc_lq_norm;
    float motorc_psi_bez;
    float motorc_psi_m_norm;
    float motorc_xbez;
    float motorc_lbez;
    float motorc_load_inertia_norm;

	/* mirrors of the global state machine variables */
    uint8_t 	global_state;
    uint32_t 	global_fault_state_description;
    uint8_t 	global_cmd;

    /* mirrors of the inverter state machine variables */
    uint8_t 	inverter_ctrl_state;
    uint8_t 	inverter_ctrl_mode;
    uint32_t 	inverter_fault_state_description;
    uint8_t 	inverter_cmd;

    /* mirrors of the global command word generation state */
    uint8_t 	global_cmd_word_generation_state;

    /* mirrors of the pwm status variables */
    uint8_t 	pwm_enable;

    /* mirrors of the brake status variables */
    uint8_t 	brake_enable;

    /* variables used for the signal generator */
    float signal_generator_freq_bez; 			/* base frequency */
    float signal_generator_u_ref_out;			/* amplitude of the output signal in Volt */
    float signal_generator_i_ref_out;			/* amplitude of the output signal in Ampere */
    float signal_generator_output;				/* signal generator output */

    /* variables used into CAN BUS */
    float duty_reference;				/* thrust input command */
    float duty_reference_abs;			/* thrust input command in abs*/
    float duty_reference_flt;			/* filtered thrust input command */
    float reference_flt_fcut;			/* filtered thrust input command: filter frequency cut */

    uint32_t debug_mode;
    uint32_t mu;
    uint32_t mv;
    uint32_t mw;

    float double_pulse_m_duty;
    float double_pulse_n_duty;
    uint8_t double_pulse_leg;
    uint32_t double_pulse_test_counter;

} param_t;

/* structure for the inverter control algorithms */
typedef struct inv_ctrl_s {
	BEMF_OBSV_LOAD_EST 		inv_obsv_load_est;  /* motor state observer with load estimator */
	BEMF_OBSV 				inv_obsv;			/* motor state observer with load estimator */
	DQVECTOR_PI 			inv_cv_ctrl;		/* motor vector current control */
	REGPI					inv_speed_ctrl;		/* motor speed control */
	SVPWM					inv_uab_sv_out;		/* space vector pwm */
} inv_ctrl_t;
#define INVERTER_CTRL inv_ctrl_t

/* structure for global state machine command word generation */
typedef struct cmd_word_generation_s {
    uint32_t 	state;
    uint32_t 	count;
    FLTLP 		duty_reference_flt;
    FLTLP 		dclink_voltage_flt;
} cmd_word_generation_t;

/* Global variables definition */
extern volatile uint16_t adc1_buf[ADC1_BUF_LEN];
extern volatile uint16_t adc2_buf[ADC2_BUF_LEN];
extern volatile uint16_t adc3_buf[ADC3_BUF_LEN];

extern volatile uint32_t adc_current_u_channel_offset;
extern volatile uint32_t adc_current_v_channel_offset;
extern volatile uint32_t adc_current_w_channel_offset;
extern volatile uint16_t adc_current_calibration_counter;

extern volatile param_t param;
extern volatile cmd_word_generation_t global_cmd_word_generation;
extern volatile double_pulse_test_pwm_t double_pulse_test;

extern volatile STATE_MACHINE global_sm_ctrl;
extern volatile INVERTER_CTRL inv_ctrl;

/* Initialization of the peripherals */
extern void peripherals_init(void);

/* Initialization of the peripherals */
extern void adc_current_calibration(void);

/* Initialization of the main functions */
extern void global_state_machine_init(void);
extern void global_cmd_word_generator_init(void);
extern void global_cmd_word_generator_reset(void);
extern void inverter_ctrl_init(void);
extern void inverter_ctrl_reset(void);

/* Process of the main functions */
extern void global_state_machine_process(void);
extern void global_cmd_word_generator_process(void);
extern void inverter_state_machine_process(void);
extern void inverter_ctrl_process(void);

/* HAL Callbacks - already defined as weak in Drivers*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif
