#ifndef FLTLP_H_
#define FLTLP_H_

typedef struct fltlp_s {
	float	sig_out;
	float	ts;
	float	wflt;
	float	z;
	float	a0;
	float	b0;
}fltlp_t;

#define FLTLP fltlp_t

void fltlpInit(volatile FLTLP *s, volatile float ts, volatile float wflt);

void fltlpUpdate(volatile FLTLP *s, volatile float ts, volatile float wflt);

void fltlpReset(volatile FLTLP *s, float input);

void fltlpProcess(volatile FLTLP *s, float input, volatile float wflt);

#endif /* FLTLP_H_ */
