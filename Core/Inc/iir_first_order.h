/*
 * iir_first_order.h
 *
 *  Created on: 15 cze 2021
 *      Author: DanielKielian
 */

#ifndef INC_IIR_FIRST_ORDER_H_
#define INC_IIR_FIRST_ORDER_H_


typedef struct iir_filter_struct {
	float a;
	float y_k;
} IirFilter_t;


void IirFilter(IirFilter_t* filter, float* x_k_1);
void IirFilter_Init(IirFilter_t* filter, uint16_t freq, float T);


#endif /* INC_IIR_FIRST_ORDER_H_ */
