#include "regpi.h"

void regpi_init(volatile REGPI *r, volatile float *ts, volatile float *kp, volatile float *ki, volatile float *l_up, volatile float *l_bottom)
{
	r->sig_out = r->ff = r->z = 0.0;
	r->ts = (float *)ts;
	r->kp = kp;
	r->ki = ki;
	r->limit_up = l_up;
	r->limit_bottom = l_bottom;

}

void regpi_reset(volatile REGPI *r)
{
	r->sig_out = r->ff = r->z = 0.0;
}

void regpi_process(volatile REGPI *r, float input, float ff, float g)
{
	float prop, output;

	prop = *r->kp * input;
	output = prop + *r->ki * *r->ts * input + r->z;

	if (output >= *r->limit_up)
		output = *r->limit_up;
	else
		if (output <= *r->limit_bottom)
			output = *r->limit_bottom;

	r->z = output - prop;

	output = (output + r->ff)*g;

	if (output >= *r->limit_up)
		output = *r->limit_up;
	else
		if (output <= *r->limit_bottom)
			output = *r->limit_bottom;

	r->sig_out = output;
}

