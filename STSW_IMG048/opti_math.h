#ifndef __OPTI_MATH_H
#define __OPTI_MATH_H

//#include <stdio.h>
#include "spd_platform.h"


#define INT32_SIZE sizeof(uint32_t) * 8 /* Integer size in bits */

int uint_log2(uint32_t num);

float taylor_cos(float x);
float taylor_sin(float x);
float taylor_tan(float x);
float taylor_sqrt(float n);
float stddev(float* a, int n);
float mean(float* a, int n);
float my_div(float numerator, float denominator);
int my_rand(void);
void my_srand(unsigned int seed);

// Online mean and standard deviation (Welford's online algorithm).
// Implementation provided by https://github.com/dizcza/OnlineMean
#define UNBIASED_ESTIMATOR  1
typedef struct OnlineMean {
	float mean;
	float varsum;  // variance sum
	float stddev;  // No need to keep track of this, but easier for data analysis on PresenceEVK
	uint32_t count;
} OnlineMean;

void OnlineMean_Init(OnlineMean *oMean);
void OnlineMean_Update(OnlineMean *oMean, float newValue);
float OnlineMean_GetMean(OnlineMean *oMean);
float OnlineMean_GetStd(OnlineMean *oMean);
void OnlineMean_Reset(OnlineMean *oMean);
float my_max(float num1, float num2);
float my_min(float num1, float num2);

#endif
