#ifndef _SV_PWM_
#define _SV_PWM_

#include "fb_application_config.h"
#include "math_f.h"
#include "tim.h"

#define MATH_2_SQRT3 	1.15470053837925f



/* Inverter PWM enable/disable */
#define INVERTER_PWM_START              1
#define INVERTER_PWM_STOP               0

/* brake enable/disable */
#define BRAKE_ENABLE              		1
#define BRAKE_DISABLE               	0

/* brake enable/disable */
#define PHASE_LEG_U              	1
#define PHASE_LEG_V              	2
#define PHASE_LEG_W               	3

/* PWM configuration
 * TIM1_CH1 -> PE9
 * TIM1_CH1N -> PE8
 * TIM1_CH2 -> PE11
 * TIM1_CH2N -> PB0
 * TIM1_CH3 -> PE13
 * TIM1_CH3N -> PE12
 * */


typedef struct sv_pwm_s 
{
	float		ts;
	float		ua;
	float 		ub;
	uint32_t	du;
	uint32_t	dv;
	uint32_t	dw;
	uint8_t 	pwm_enable;
	uint8_t 	brake_enable;
} sv_pwm_t;

#define SVPWM sv_pwm_t

typedef struct double_pulse_test_pwm_s
{
	float		ts;
	uint32_t	double_pulse_m_duty;
	uint32_t	double_pulse_n_duty;
	uint8_t		double_pulse_leg;
	uint8_t 	double_pulse_counter;
} double_pulse_test_pwm_t;


void sv_pwm_process(volatile SVPWM *c);

void double_pulse_test_init(volatile double_pulse_test_pwm_t *c, float pulse_m_duty,
		float pulse_n_duty, uint8_t pulse_leg, uint32_t pulse_counter);

void double_pulse_test_process(volatile double_pulse_test_pwm_t *c);

#endif
