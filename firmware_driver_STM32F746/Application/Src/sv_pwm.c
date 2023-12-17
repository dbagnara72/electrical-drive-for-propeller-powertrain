#include "sv_pwm.h"

void sv_pwm_process(volatile SVPWM *c)
{
	float ref_a, ref_b, t1, t2, t0;
	
	if ((c->pwm_enable == INVERTER_PWM_START) && (c->brake_enable == BRAKE_DISABLE))
	{
		if (c->ua >= 0)
			ref_a = c->ua * MATH_SQRT3_2;

		else
			ref_a = c->ua * -MATH_SQRT3_2;

		if (c->ub >= 0)
			ref_b = c->ub * 0.5;
		else
			ref_b = c->ub * -0.5;

		if (ref_a >= ref_b)
		{
			if (c->ua >= 0)
			{
				if (c->ub >= 0) 
				{
//					pwm_sector = 0;
					t2 = c->ub * MATH_2_SQRT3;
					t1 = c->ua - c->ub * MATH_1_SQRT3;
					t0 = 1.0 - t2 - t1;

					if (t0 > 0) 
					{
						c->du = (uint32_t)(t0 * PWM_TB * 0.5);
						c->dv = (uint32_t)(t1 * PWM_TB) + c->du;
						c->dw = (uint32_t)(t2 * PWM_TB) + c->dv;
					}
					else 
					{
						t1 = t1 / (t1 + t2);
						c->du = (uint32_t)0;
						c->dv = (uint32_t)(t1 * PWM_TB);
						c->dw = (uint32_t)PWM_TB;
					}
				}
				else 
				{
//					pwm_sector = 5;
					t1 = c->ua + c->ub * MATH_1_SQRT3;
					t2 = (c->ua - t1) * 0.5 - c->ub * MATH_SQRT3_2;
					t0 = 1.0 - t2 - t1;

					if (t0 > 0) 
					{
						c->du = (uint32_t)(t0 * PWM_TB * 0.5);
						c->dw = (uint32_t)(t1 * PWM_TB) + c->du;
						c->dv = (uint32_t)(t2 * PWM_TB) + c->dw;
					}
					else 
					{
						t1 = t1 / (t1 + t2);
						c->du = (uint32_t)0;
						c->dw = (uint32_t)(t1 * PWM_TB);
						c->dv = (uint32_t) PWM_TB;
					}
				}
			}
			else {
				if (c->ub >= 0) 
				{
//					pwm_sector = 2;
					t2 = c->ub * -MATH_1_SQRT3 - c->ua;
					t1 = c->ub * MATH_SQRT3_2 - (c->ua + t2) * 0.5;
					t0 = 1.0 - t2 - t1;

					if (t0 > 0) 
					{
						c->dv = (uint32_t)(t0 * PWM_TB * 0.5);
						c->dw = (uint32_t)(t1 * PWM_TB) + c->dv;
						c->du = (uint32_t)(t2 * PWM_TB) + c->dw;
					}
					else 
					{
						t1 = t1 / (t1 + t2);
						c->dv = (uint32_t)0;
						c->dw = (uint32_t)(t1 * PWM_TB);
						c->du = (uint32_t)PWM_TB;
					}
				}
				else 
				{
//					pwm_sector = 3;
					t1 = c->ub * -MATH_2_SQRT3;
					t2 = c->ub * MATH_1_SQRT3 - c->ua;
					t0 = 1.0 - t2 - t1;

					if (t0 > 0) 
					{
						c->dw = (uint32_t)(t0 * PWM_TB * 0.5);
						c->dv = (uint32_t)(t1 * PWM_TB) + c->dw;
						c->du = (uint32_t)(t2 * PWM_TB) + c->dv;
					}
					else 
					{
						t1 = t1 / (t1 + t2);
						c->dw = (uint32_t)0;
						c->dv = (uint32_t)(t1 * PWM_TB);
						c->du = (uint32_t)PWM_TB;
					}
				}
			}
		}
		else 
		{
			if (c->ub >= 0) 
			{
//				pwm_sector = 1;
				t1 = c->ub * MATH_1_SQRT3 - c->ua;
				t2 = (c->ua - t1) * 0.5 + c->ub * MATH_SQRT3_2;
				t0 = 1.0 - t2 - t1;

				if (t0 > 0) 
				{
					c->dv = (uint32_t)(t0 * PWM_TB * 0.5);
					c->du = (uint32_t)(t1 * PWM_TB) + c->dv;
					c->dw = (uint32_t)(t2 * PWM_TB) + c->du;
				}
				else 
				{
					t1 = t1 / (t1 + t2);
					c->dv = (uint32_t)0;
					c->du = (uint32_t)(t1 * PWM_TB);
					c->dw = (uint32_t)PWM_TB;
				}
			}
			else 
			{
//				pwm_sector = 4;
				t2 = c->ua - c->ub * MATH_1_SQRT3;
				t1 = (c->ua + t2) * -0.5 - c->ub * MATH_SQRT3_2;
				t0 = 1.0 - t2 - t1;

				if (t0 > 0) 
				{
					c->dw = (uint32_t)(t0 * PWM_TB * 0.5);
					c->du = (uint32_t)(t1 * PWM_TB) + c->dw;
					c->dv = (uint32_t)(t2 * PWM_TB) + c->du;
				}
				else 
				{
					t1 = t1 / (t1 + t2);
					c->dw = (uint32_t)0;
					c->du = (uint32_t)(t1 * PWM_TB);
					c->dv = (uint32_t)PWM_TB;
				}
			}
		}

		/* enable of the MOSFET driver*/
		GPIOG->BSRR = (uint16_t)GPIO_PIN_2;
		GPIOG->BSRR = (uint16_t)GPIO_PIN_3;
		GPIOG->BSRR = (uint16_t)GPIO_PIN_4;

		/* update PWM registers */
		htim1.Instance->CCR1 = c->du;
		htim1.Instance->CCR2 = c->dv;
		htim1.Instance->CCR3 = c->dw;

		/* enable PWM channels */
		htim1.Instance->BDTR |= TIM_BDTR_MOE;
		htim1.Instance->BDTR |= TIM_BDTR_OSSI;
		htim1.Instance->BDTR |= TIM_BDTR_OSSR;
		htim1.Instance->CCER |= TIM_CCER_CC1E;
		htim1.Instance->CCER |= TIM_CCER_CC1NE;
		htim1.Instance->CCER |= TIM_CCER_CC2E;
		htim1.Instance->CCER |= TIM_CCER_CC2NE;
		htim1.Instance->CCER |= TIM_CCER_CC3E;
		htim1.Instance->CCER |= TIM_CCER_CC3NE;

	}

	if ((c->pwm_enable == INVERTER_PWM_STOP) && (c->brake_enable == BRAKE_DISABLE))
	{

		/* disable of the MOSFET driver*/
		GPIOG->BSRR = (uint16_t)GPIO_PIN_2  << 16;
		GPIOG->BSRR = (uint16_t)GPIO_PIN_3  << 16;
		GPIOG->BSRR = (uint16_t)GPIO_PIN_4  << 16;

		/* update PWM registers */
		htim1.Instance->CCR1 = (uint32_t)0;
		htim1.Instance->CCR2 = (uint32_t)0;
		htim1.Instance->CCR3 = (uint32_t)0;

		/* enable PWM channels */
		htim1.Instance->BDTR |= TIM_BDTR_MOE;
		htim1.Instance->BDTR |= TIM_BDTR_OSSI;
		htim1.Instance->BDTR |= TIM_BDTR_OSSR;
		htim1.Instance->CCER |= TIM_CCER_CC1E;
		htim1.Instance->CCER &= ~TIM_CCER_CC1NE;
		htim1.Instance->CCER |= TIM_CCER_CC2E;
		htim1.Instance->CCER &= ~TIM_CCER_CC2NE;
		htim1.Instance->CCER |= TIM_CCER_CC3E;
		htim1.Instance->CCER &= ~TIM_CCER_CC3NE;
	}

	if ((c->pwm_enable == INVERTER_PWM_START) && (c->brake_enable == BRAKE_ENABLE))
	{
		/* enable of the MOSFET driver*/
		GPIOG->BSRR = (uint16_t)GPIO_PIN_2;
		GPIOG->BSRR = (uint16_t)GPIO_PIN_3;
		GPIOG->BSRR = (uint16_t)GPIO_PIN_4;

		/* update PWM registers */
		htim1.Instance->CCR1 = (uint32_t)(0);
		htim1.Instance->CCR2 = (uint32_t)(0);
		htim1.Instance->CCR3 = (uint32_t)(0);

		/* enable PWM channels */
		htim1.Instance->BDTR |= TIM_BDTR_MOE;
		htim1.Instance->BDTR |= TIM_BDTR_OSSI;
		htim1.Instance->BDTR |= TIM_BDTR_OSSR;
		htim1.Instance->CCER |= TIM_CCER_CC1E;
		htim1.Instance->CCER |= TIM_CCER_CC1NE;
		htim1.Instance->CCER |= TIM_CCER_CC2E;
		htim1.Instance->CCER |= TIM_CCER_CC2NE;
		htim1.Instance->CCER |= TIM_CCER_CC3E;
		htim1.Instance->CCER |= TIM_CCER_CC3NE;

	}
}

void double_pulse_test_init(volatile double_pulse_test_pwm_t *c, float pulse_m_duty,
		float pulse_n_duty, uint8_t pulse_leg, uint32_t pulse_counter)
{
	c->double_pulse_counter = pulse_counter;
	c->double_pulse_leg = pulse_leg;
	c->double_pulse_m_duty = (uint32_t)(pulse_m_duty*PWM_TB);
	c->double_pulse_n_duty = (uint32_t)(pulse_n_duty*PWM_TB);
}


void double_pulse_test_process(volatile double_pulse_test_pwm_t *c)
{

	if (c->double_pulse_counter)
	{
		switch (c->double_pulse_leg)
		{
			case PHASE_LEG_W:

				/* begin - double pulse test */
				htim1.Instance->CCR5 = (uint32_t)(c->double_pulse_n_duty);
				HAL_TIMEx_GroupChannel5(&htim1,TIM_GROUPCH5_OC3REFC);
				HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_5);
				/* end - double pulse test */

				/* enable of the MOSFET driver*/
				GPIOG->BSRR = (uint16_t)GPIO_PIN_2;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_3;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_4;

				/* update pwm registers */
				htim1.Instance->CCR1 = (uint32_t)(0);
				htim1.Instance->CCR2 = (uint32_t)(0);
				htim1.Instance->CCR3 = (uint32_t)(c->double_pulse_m_duty);

				/* enable PWM channels */
				htim1.Instance->BDTR |= TIM_BDTR_MOE;
				htim1.Instance->BDTR |= TIM_BDTR_OSSI;
				htim1.Instance->BDTR |= TIM_BDTR_OSSR;
				htim1.Instance->CCER |= TIM_CCER_CC1E;
				htim1.Instance->CCER &= ~TIM_CCER_CC1NE;
				htim1.Instance->CCER |= TIM_CCER_CC2E;
				htim1.Instance->CCER &= ~TIM_CCER_CC2NE;
				htim1.Instance->CCER |= TIM_CCER_CC3E;
				htim1.Instance->CCER |= TIM_CCER_CC3NE;

				c->double_pulse_counter--;
				break;

			case PHASE_LEG_V:

				/* begin - double pulse test */
				htim1.Instance->CCR5 = (uint32_t)(c->double_pulse_n_duty);
				HAL_TIMEx_GroupChannel5(&htim1,TIM_GROUPCH5_OC2REFC);
				HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_5);
				/* end - double pulse test */

				/* enable of the MOSFET driver*/
				GPIOG->BSRR = (uint16_t)GPIO_PIN_2;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_3;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_4;

				/* update pwm registers */
				htim1.Instance->CCR1 = (uint32_t)(0);
				htim1.Instance->CCR2 = (uint32_t)(c->double_pulse_m_duty);
				htim1.Instance->CCR3 = (uint32_t)(0);

				/* enable PWM channels */
				htim1.Instance->BDTR |= TIM_BDTR_MOE;
				htim1.Instance->BDTR |= TIM_BDTR_OSSI;
				htim1.Instance->BDTR |= TIM_BDTR_OSSR;
				htim1.Instance->CCER |= TIM_CCER_CC1E;
				htim1.Instance->CCER &= ~TIM_CCER_CC1NE;
				htim1.Instance->CCER |= TIM_CCER_CC2E;
				htim1.Instance->CCER |= TIM_CCER_CC2NE;
				htim1.Instance->CCER |= TIM_CCER_CC3E;
				htim1.Instance->CCER &= ~TIM_CCER_CC3NE;

				c->double_pulse_counter--;
				break;

			case PHASE_LEG_U:

				/* begin - double pulse test */
				htim1.Instance->CCR5 = (uint32_t)(c->double_pulse_n_duty);
				HAL_TIMEx_GroupChannel5(&htim1,TIM_GROUPCH5_OC1REFC);
				HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_5);
				/* end - double pulse test */

				/* enable of the MOSFET driver*/
				GPIOG->BSRR = (uint16_t)GPIO_PIN_2;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_3;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_4;

				/* update pwm registers */
				htim1.Instance->CCR1 = (uint32_t)(c->double_pulse_m_duty);
				htim1.Instance->CCR2 = (uint32_t)(0);
				htim1.Instance->CCR3 = (uint32_t)(0);

				/* enable PWM channels */
				htim1.Instance->BDTR |= TIM_BDTR_MOE;
				htim1.Instance->BDTR |= TIM_BDTR_OSSI;
				htim1.Instance->BDTR |= TIM_BDTR_OSSR;
				htim1.Instance->CCER |= TIM_CCER_CC1E;
				htim1.Instance->CCER |= TIM_CCER_CC1NE;
				htim1.Instance->CCER |= TIM_CCER_CC2E;
				htim1.Instance->CCER &= ~TIM_CCER_CC2NE;
				htim1.Instance->CCER |= TIM_CCER_CC3E;
				htim1.Instance->CCER &= ~TIM_CCER_CC3NE;

				c->double_pulse_counter--;
				break;
		}
	}
	else
	{
		switch (c->double_pulse_leg)
		{
			case PHASE_LEG_W:

				htim1.Instance->CCR5 = (uint32_t)(0);
				HAL_TIMEx_GroupChannel5(&htim1,TIM_GROUPCH5_NONE);
				HAL_TIM_PWM_Stop_IT(&htim1,TIM_CHANNEL_5);

				/* enable of the MOSFET driver*/
				GPIOG->BSRR = (uint16_t)GPIO_PIN_2;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_3;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_4;

				/* update PWM registers */
				htim1.Instance->CCR1 = (uint32_t)(0);
				htim1.Instance->CCR2 = (uint32_t)(0);
				htim1.Instance->CCR3 = (uint32_t)(PWM_TB);

				/* enable PWM channels */
				htim1.Instance->BDTR |= TIM_BDTR_MOE;
				htim1.Instance->BDTR |= TIM_BDTR_OSSI;
				htim1.Instance->BDTR |= TIM_BDTR_OSSR;

				htim1.Instance->CCER |= TIM_CCER_CC1E;
				htim1.Instance->CCER &= ~TIM_CCER_CC1NE;

				htim1.Instance->CCER |= TIM_CCER_CC2E;
				htim1.Instance->CCER &= ~TIM_CCER_CC2NE;

				htim1.Instance->CCER |= TIM_CCER_CC3E;
				htim1.Instance->CCER |= TIM_CCER_CC3NE;

				break;

			case PHASE_LEG_V:

				htim1.Instance->CCR5 = (uint32_t)(0);
				HAL_TIMEx_GroupChannel5(&htim1,TIM_GROUPCH5_NONE);
				HAL_TIM_PWM_Stop_IT(&htim1,TIM_CHANNEL_5);


				/* enable of the MOSFET driver*/
				GPIOG->BSRR = (uint16_t)GPIO_PIN_2;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_3;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_4;

				/* update PWM registers */
				htim1.Instance->CCR1 = (uint32_t)(0);
				htim1.Instance->CCR2 = (uint32_t)(PWM_TB);
				htim1.Instance->CCR3 = (uint32_t)(0);

				/* enable PWM channels */
				htim1.Instance->BDTR |= TIM_BDTR_MOE;
				htim1.Instance->BDTR |= TIM_BDTR_OSSI;
				htim1.Instance->BDTR |= TIM_BDTR_OSSR;

				htim1.Instance->CCER |= TIM_CCER_CC1E;
				htim1.Instance->CCER &= ~TIM_CCER_CC1NE;

				htim1.Instance->CCER |= TIM_CCER_CC2E;
				htim1.Instance->CCER |= TIM_CCER_CC2NE;

				htim1.Instance->CCER |= TIM_CCER_CC3E;
				htim1.Instance->CCER &= ~TIM_CCER_CC3NE;
				break;

			case PHASE_LEG_U:

				htim1.Instance->CCR5 = (uint32_t)(0);
				HAL_TIMEx_GroupChannel5(&htim1,TIM_GROUPCH5_NONE);
				HAL_TIM_PWM_Stop_IT(&htim1,TIM_CHANNEL_5);

				/* enable of the MOSFET driver*/
				GPIOG->BSRR = (uint16_t)GPIO_PIN_2;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_3;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_4;

				/* update PWM registers */
				htim1.Instance->CCR1 = (uint32_t)(PWM_TB);
				htim1.Instance->CCR2 = (uint32_t)(0);
				htim1.Instance->CCR3 = (uint32_t)(0);

				/* enable PWM channels */
				htim1.Instance->BDTR |= TIM_BDTR_MOE;
				htim1.Instance->BDTR |= TIM_BDTR_OSSI;
				htim1.Instance->BDTR |= TIM_BDTR_OSSR;

				htim1.Instance->CCER |= TIM_CCER_CC1E;
				htim1.Instance->CCER |= TIM_CCER_CC1NE;

				htim1.Instance->CCER |= TIM_CCER_CC2E;
				htim1.Instance->CCER &= ~TIM_CCER_CC2NE;

				htim1.Instance->CCER |= TIM_CCER_CC3E;
				htim1.Instance->CCER &= ~TIM_CCER_CC3NE;

				break;

			default:

				htim1.Instance->CCR5 = (uint32_t)(0);
				HAL_TIMEx_GroupChannel5(&htim1,TIM_GROUPCH5_NONE);
				HAL_TIM_PWM_Stop_IT(&htim1,TIM_CHANNEL_5);

				/* enable of the MOSFET driver*/
				GPIOG->BSRR = (uint16_t)GPIO_PIN_2;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_3;
				GPIOG->BSRR = (uint16_t)GPIO_PIN_4;

				/* update PWM registers */
				htim1.Instance->CCR1 = (uint32_t)0;
				htim1.Instance->CCR2 = (uint32_t)0;
				htim1.Instance->CCR3 = (uint32_t)0;

				/* enable PWM channels */
				htim1.Instance->BDTR |= TIM_BDTR_MOE;
				htim1.Instance->BDTR |= TIM_BDTR_OSSI;
				htim1.Instance->BDTR |= TIM_BDTR_OSSR;

				htim1.Instance->CCER |= TIM_CCER_CC1E;
				htim1.Instance->CCER &= ~TIM_CCER_CC1NE;

				htim1.Instance->CCER |= TIM_CCER_CC2E;
				htim1.Instance->CCER &= ~TIM_CCER_CC2NE;

				htim1.Instance->CCER |= TIM_CCER_CC3E;
				htim1.Instance->CCER &= ~TIM_CCER_CC3NE;
		}
	}


}

