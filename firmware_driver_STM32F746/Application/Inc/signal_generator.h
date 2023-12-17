#ifndef _SIGNAL_GENERATOR_H_
#define _SIGNAL_GENERATOR_H_

typedef struct signal_generator_s {
	float	sig_out;
	float	ts;
	float	freq;
	float	z;
	float	a0;
	float	b0;
}signal_generator_t;

#define SIGNAL_GENERATOR signal_generator_t

void signal_generator_init(volatile SIGNAL_GENERATOR *s, volatile float ts, volatile float freq);

void signal_generator_reset(volatile SIGNAL_GENERATOR *s, float input);

void signal_generator_update(volatile SIGNAL_GENERATOR *s, volatile float ts, volatile float freq);

void signal_generator_process(volatile SIGNAL_GENERATOR *s, float input, volatile float freq);

#endif
