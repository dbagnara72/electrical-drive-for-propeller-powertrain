/* firmware version set manually */
/* (year-2000) / month / day / hour */
#define FIRMWARE_VERSION	23100314

/* In this file the application config is set. Application config means the selection of the HW driver, Motor Type and Load Mode*/

#define CLOCK_APB2 			216000000

/* begin - independent watch dog - IWD */
#define IWD_PRESCALE		4
#define IWD_WINDOW			256
#define IWD_DOWN_COUNTER	4
/* end - independent watch dog - IWD */

/* begin - Configuration application */

/* MCI-EPL Inverter */
#define MAD_MOTOR
#define MCI_EPL_INVERTER
#define PMSM_LOAD 				/* this label is used to switch from ID control, like in RL load to IQ control like with PMSM */

/* INFINEON GaN Inverter */
//#define INFINEON_GAN			/* INVERTER from infineon 1kW with GaN MOSFET */
//#define NANOTEC_DB42S02
//#define RL_LOAD

/* end - Configuration devices and application */


/* begin - Configuration switching frequency */

/* PWM Base counter - set according to APB2 clock frequency
 *  25kHz switching frequency */
#define PWM_TB 			4320		/* PWM switching frequency and main task at 25kHz */
//#define PWM_TB 			5400		/* PWM switching frequency and main task at 20kHz */
#define DEAD_TIME 		90 		/* dead time */

/* 50kHz switching frequency  - only for double pulse test */
//#define PWM_TB 			2160 	/* PWM switching frequency and main task */
//#define DEAD_TIME 		85 		/* dead time */

/* 64kHz switching frequency  - only for double pulse test */
//#define PWM_TB 			1680 	/* PWM switching frequency and main task */
//#define DEAD_TIME 		120 	/* dead time */

/* end - Configuration switching frequency */


/* begin - double pulse test configuration */

/* The signals from double pulse test are generated using the option three
 * phase combined pwm mode and using the channel 5 of the TIM1. For this application
 * we use two compare reference m_duty for channel 1,2,3 and n_duty for channel 5.
 * */
#define DPT_N_REFERENCE_DUTY 			0.15f //0.293
#define DPT_M_REFERENCE_DUTY 			1.0f //1.00

/* end - double pulse test configuration */

/* begin - enable HW over-current detection */
#define EXT_OVERCURRENT_INTERRUPTS
/* end - enable HW over-current detection */


