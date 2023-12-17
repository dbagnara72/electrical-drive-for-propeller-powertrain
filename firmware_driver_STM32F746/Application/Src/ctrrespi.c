/*
 * Resonant PI (internal model control)
 */
#include "ctrrespi.h"
#include "math_f.h"

// ------------------------------------------------------------------------------
volatile float ctrrespi_one = 1.0;
volatile float ctrrespi_zero = 0.0;

// ------------------------------------------------------------------------------
void ctrrespiInit(volatile CTRRESPI *c, volatile const float *tb, volatile const float *p, volatile const float *i, volatile const float *l)
{
	c->mode = 0;
	c->x1 = 0.0;
	c->x2 = 0.0;
	c->x1_z = 0.0;
	c->x2_z = 0.0;
	c->tb = (float *)tb;
	c->clip_rel = CTRRESPI_CLIP_RELEASE;
	c->output = 0.0;

	c->p = p;
	if (c->p == 0) {
		c->p = &ctrrespi_one;
	}
	c->i = i;
	if (c->i == 0) {
		c->i = &ctrrespi_zero;
	}
	c->limit = l;
	if (c->limit == 0) {
		c->limit = &ctrrespi_one;
	}
}

// ------------------------------------------------------------------------------
void ctrrespiReset(volatile CTRRESPI *c)
{
	c->x1 = 0.0;
	c->x2 = 0.0;
	c->x1_z = 0.0;
	c->x2_z = 0.0;
	c->output = 0.0;
	c->mode &= ~CTRRESPI_CLIP;
}

// ------------------------------------------------------------------------------
void ctrrespiUpdate(volatile CTRRESPI *c, volatile const float *p, volatile const float *i)
{
	if (p) {
		c->p = p;
	}
	if (i) {
		c->i = i;
	}
}

// ------------------------------------------------------------------------------
void ctrrespiProcess(volatile CTRRESPI *c, float input, float ff)
{
	float o;


	c->x1 = input * *c->i * CTRRESPI_B1 * *c->tb + c->x1_z * CTRRESPI_A11 + c->x2_z * CTRRESPI_A12;
	c->x2 = c->x1_z * CTRRESPI_A21 + c->x2_z * CTRRESPI_A22;
	o = input * *c->p + c->x1_z * CTRRESPI_C1 + c->x2_z * CTRRESPI_C2 + ff;
	c->x1_z = c->x1;
	c->x2_z = c->x2;

	if (o > 1.0) {
		o = 1.0;
	}
	else if (o < -1.0) {
		o = -1.0;
	}
	c->output = o;

}

