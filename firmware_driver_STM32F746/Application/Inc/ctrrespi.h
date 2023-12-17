/*
 * PI risonante (internal model control)
 *
 */
#ifndef _CTRRESPI_
#define _CTRRESPI_



#define CTRRESPI_CLIP_RELEASE		0.05f
#define CTRRESPI_CLIP_THRESHOLD	    0.01f

#define CTRRESPI_CLIP				0x0001f
#define CTRRESPI_CLIP_EXT			0x0002f
#define CTRRESPI_RELEASE			0x0004f

#define CTRRESPI_B1			1.0f
#define CTRRESPI_B2			0.0f

#define CTRRESPI_A11        1.997757892722625f
#define CTRRESPI_A12        -0.998744152176286f
#define CTRRESPI_A21        1.0f
#define CTRRESPI_A22        0.0f

#define CTRRESPI_C1         0.999207562596823f
#define CTRRESPI_C2         -0.999207562596823f

//questo va male
//#define CTRRESPI_A11		1.998385309523840
//#define CTRRESPI_A12		-0.999371878820035
//#define CTRRESPI_A21		1.0
//#define CTRRESPI_A22		0.0
//
//#define CTRRESPI_C1			0.999521472897057
//#define CTRRESPI_C2			-0.999521472897057

// questo gira
//#define CTRRESPI_A11        1.996498016262538
//#define CTRRESPI_A12        -0.9980384309875865
//#define CTRRESPI_A21        1.0
//#define CTRRESPI_A22        0.0
//
//#define CTRRESPI_C1         0.9987621455638036
//#define CTRRESPI_C2         -0.9987621455638036



typedef struct ctrrespi_s {
	unsigned int			mode;		/*!< Controller mode. */
	float					output;		/*!< Controller output. */
	float					clip_rel;	/*!< Clip release gain. */
	float					*tb;		/*!< Pointer to the current time base. */
	volatile const float	*p;			/*!< Pointer to the P gain value. */
	volatile const float	*i;			/*!< Pointer to the I gain value. */
	float					x1;
	float					x2;
	float					x1_z;
	float					x2_z;
	volatile const float	*limit;		/*!< Pointer to the limit value. */
} ctrrespi_t;
#define CTRRESPI ctrrespi_t


void ctrrespiInit(volatile CTRRESPI *c, volatile const float *tb, volatile const float *p, volatile const float *i, volatile const float *l);


void ctrrespiReset(volatile CTRRESPI *c);


void ctrrespiUpdate(volatile CTRRESPI *c, volatile const float *p, volatile const float *i);

/*
 *
 * This function is called to process the controller as RESPI controller.
 * The output is stored in the controller output. The CTRRESPI_CLIP bit
 * in the mode variable of the controller object indicates the output
 * is clipping at the moment. The feed forward component is added to
 * the output and clipped by the anti wind up mechanism.
 */
void ctrrespiProcess(volatile CTRRESPI *c, float input, float ff);

#endif
