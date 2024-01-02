/*
 * first order low pass filter
 */

#include "fltlp.h"

void fltlpInit(volatile FLTLP *s, volatile float ts, volatile float wflt)
{
	s->sig_out = s->z = 0.0;
	s->ts = ts;
	s->wflt = wflt;
	s->a0 = s->wflt * s->ts;
	s->b0 = 1 - s->a0;

}

void fltlpUpdate(volatile FLTLP *s, volatile float ts, volatile float wflt)
{
	if ((s->ts != ts) || (s->wflt != wflt)){
		s->ts = ts;
		s->wflt = wflt;
		s->a0 = s->wflt * s->ts;
		s->b0 = 1 - s->a0;
	}
}

void fltlpReset(volatile FLTLP *s, float input)
{
	s->sig_out = s->z = input;
}


void fltlpProcess(volatile FLTLP *s, float input, volatile float wflt)
{
	if (s->wflt != wflt)
	{
		s->wflt = wflt;
		s->a0 = s->wflt * s->ts;
		s->b0 = 1 - s->a0;
	}

	s->sig_out = input * s->a0 + s->z * s->b0;
	s->z = s->sig_out;
}
