
#include "rms.h"
#include "math.h"
#include "math_f.h"

// ------------------------------------------------------------------------------
void rmsInit(volatile RMS *f, volatile float n)
{
	f->mode = 0;
	f->output = f->z = 0.0;
	f->tb = 0.0;
	f->n = n;
}

// ------------------------------------------------------------------------------
void rmsSetTimeBase(volatile RMS *f, volatile float tb)
{
	f->tb = tb;
}

// ------------------------------------------------------------------------------
void rmsReset(volatile RMS *f, float input)
{
	f->output = f->z = input;
}

// ------------------------------------------------------------------------------
void rmsProcess(volatile RMS *f, float input, float n)
{
	float y_quad;

	if (n == 0.0) {
		n = f->n;
	}

	y_quad = (fabsf(input * input) + f->z * (n - 1)) / n;

	if (y_quad <= 0) {
		y_quad = RMS_CLIP_VALUE;
	}
	f->z = y_quad;
	f->output = sqrtf(y_quad);
}
