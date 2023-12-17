
#ifndef REGPI_H_
#define REGPI_H_

typedef struct regpi_s {
	float	sig_out;
	float	z;
	float	ff;
	float	*ts;
	volatile float	*kp;
	volatile float	*ki;
	volatile float	*limit_up;
	volatile float	*limit_bottom;
}regpi_t;

#define REGPI regpi_t

void regpi_init(volatile REGPI *r, volatile float *ts, volatile float *kp, volatile float *ki, volatile float *l_up, volatile float *l_bottom);

void regpi_reset(volatile REGPI *r);

void regpi_process(volatile REGPI *r, float input, float ff, float g);

#endif
