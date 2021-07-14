/*
 * iir_first_order.c
 *
 *  Created on: 15 cze 2021
 *      Author: DanielKielian
 */
#include "main.h"
#include "iir_first_order.h"

/* Function which filters signal
 *  filter - handler to filter structure
 *  x_k - pointer to last point
 */
void IirFilter(IirFilter_t* filter, float* x_k)
{
	filter->y_k = filter->a*(*x_k)+(1-filter->a)*filter->y_k;
}



/* freq - sample rate [Hz]
 * T -  time constant of low-pass filter [s]
 */
void IirFilter_Init(IirFilter_t* filter, uint16_t freq, float T)
{
	filter->a = (1.0/(float)freq)/T;
	filter->y_k = 0;
}
