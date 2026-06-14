/*
 * noarm_cmsis.h
 *
 *  Created on: 23 mag 2017
 *      Author: Simone
 */

#ifndef NOARM_CMSIS_H_
#define NOARM_CMSIS_H_

#include <stdlib.h>
#include <stdio.h>
#include "arm_math.h"

void nonARM_product(  float32_t * pSrcA,float32_t * pSrcB, float32_t * pDst, int maxb);
void noARM_FIR(float32_t * Input_f32_1kHz_15kHz, float32_t * FIRCoeffs32, int TEST_LENGTH, int N_TAPS,float32_t *Output, float32_t * x_i);

#endif /* NOARM_CMSIS_H_ */
