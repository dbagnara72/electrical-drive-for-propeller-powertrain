/*
 * filtro per shiftare la componente D nella componente Q dell'inverter monofase
 *
 */
#ifndef _FLTD2Q_
#define _FLTD2Q_



#define FLTD2Q_B1			0.000966531084866133f
#define FLTD2Q_B2			0.000946498544476605f

#define FLTD2Q_A1		    -1.93814485260962f
#define FLTD2Q_A2		    0.939101367424292f

typedef struct fltd2q_s {
	unsigned int			mode;		/*!< Controller mode. */
	float					output;		/*!< Controller output. */
	float					*tb;		/*!< Pointer to the current time base. */
	float					x1;
	float					x2;
	float					y1;
	float					y2;
} fltd2q_t;
#define FLTD2Q fltd2q_t


void fltd2qInit(volatile FLTD2Q *c, volatile const float *tb);

void fltd2qReset(volatile FLTD2Q *c);

void fltd2qProcess(volatile FLTD2Q *c, float input);

#endif
