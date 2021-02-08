/*
 * fft.h
 *
 *  Created on: 23 дек. 2017 г.
 *      Author: pavel
 */

#ifndef FFT_H_
#define FFT_H_
#include "stm32f4xx_hal.h"
#include "arm_math.h"

#define FFT_OK 0
#define FFT_ERROR 1

#define FFT_TYPE_F32 0
#define FFT_TYPE_Q15 1

uint8_t FFT_Init(uint8_t fftDataType);
uint8_t FFT_Do_F32(float32_t* in, float32_t* out);
uint8_t FFT_Do_Q15(q15_t* in, q15_t* out);
#endif /* FFT_H_ */
