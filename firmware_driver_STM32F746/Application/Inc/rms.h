
#ifndef RMS_CALC
#define RMS_CALC


#define RMS_CLIP_VALUE	0.01f

/* rms calc structure. */
typedef struct rms_s {
	unsigned int			mode;		/*!< Controller mode. */
	float					output;		/*!< Controller output. */
	float					tb;			/*!< Current time base. */
	float					n;			/*!< Number of samples. */
	float					z;			/*!< Delayed sample. */
} rms_t;
#define RMS rms_t

void rmsInit(volatile RMS *f, volatile float n);


void rmsSetTimeBase(volatile RMS *f, volatile float tb);


void rmsReset(volatile RMS *f, float input);


void rmsProcess(volatile RMS *f, float input, float n);

#endif
