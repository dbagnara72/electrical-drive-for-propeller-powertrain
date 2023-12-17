#include "signal_generator.h"
#include "math_f.h"
#include "math.h"

void signal_generator_init(volatile SIGNAL_GENERATOR *s, volatile float ts, volatile float freq)
{
	s->sig_out = s->z = 0.0;
	s->ts = ts;
	s->freq = freq;
	s->a0 = s->freq * MATH_2PI * s->ts;
	s->b0 = 1;

}

void signal_generator_update(volatile SIGNAL_GENERATOR *s, volatile float ts, volatile float freq)
{
	if ((s->ts != ts) || (s->freq != freq)){
		s->ts = ts;
		s->freq = freq;
		s->a0 = s->freq * MATH_2PI * s->ts;
		s->b0 = 1;
	}
}

void signal_generator_reset(volatile SIGNAL_GENERATOR *s, float input)
{
	s->sig_out = s->z = input;
}


void signal_generator_process(volatile SIGNAL_GENERATOR *s, float input, volatile float freq)
{
	if (s->freq != freq)
	{
		s->freq = freq;
		s->a0 = s->freq * MATH_2PI * s->ts;
		s->b0 = 1;
	}

	const float signal_out = input * s->a0 + s->z * s->b0;
	const float sig_out_bonded = - MATH_PI + fmodf(signal_out + MATH_3PI, MATH_2PI);

	s->sig_out = sig_out_bonded;
	s->z = sig_out_bonded;
}
