/*
 * second order low pass filter - generally used to shift 90 degree at given frequency
 */
#include "fltd2q.h"
#include "math_f.h"

// ------------------------------------------------------------------------------
void fltd2qInit(volatile FLTD2Q *c, volatile const float *tb)
{
	c->mode = 0;
	c->x1 = 0.0;
	c->x2 = 0.0;
	c->y1 = 0.0;
	c->y2 = 0.0;
	c->tb = (float *)tb;
	c->output = 0.0;
}

// ------------------------------------------------------------------------------
void fltd2qReset(volatile FLTD2Q *c)
{
	c->x1 = 0.0;
	c->x2 = 0.0;
	c->y1 = 0.0;
	c->y2 = 0.0;
	c->output = 0.0;
}

// ------------------------------------------------------------------------------
void fltd2qProcess(volatile FLTD2Q *c, float input)
{
	float o;

    o = c->x1 * FLTD2Q_B1 + c->x2 * FLTD2Q_B2 - c->y1 * FLTD2Q_A1 - c->y2 * FLTD2Q_A2;
    c->x2 = c->x1;
    c->x1 = input;
    c->y2 = c->y1;
    c->y1 = o;
	c->output = o;

}

