#include "fft.h"
#include "stdlib.h"
#include "common.h"

static arm_rfft_fast_instance_f32 f32i;
static arm_rfft_instance_q15 q15i;
static uint8_t type;

uint8_t FFT_Init(uint8_t fftDataType)
{
	type = fftDataType;
	arm_status status;
	switch(fftDataType) {
	case FFT_TYPE_F32:
		status = arm_rfft_fast_init_f32(&f32i, FFT_SIZE);
		break;
	case FFT_TYPE_Q15:
		/*
		arm_rfft_init_q15
		[in,out]	S	points to an instance of the Q15 RFFT/RIFFT structure
		[in]	fftLenReal	length of the FFT
		[in]	ifftFlagR	flag that selects transform direction
			value = 0: forward transform
			value = 1: inverse transform

		[in]	bitReverseFlag	flag that enables / disables bit reversal of output
			value = 0: disables bit reversal of output
			value = 1: enables bit reversal of output
		 */
		status = arm_rfft_init_q15(&q15i, FFT_SIZE, 0, 1);
		break;
	}
	if (status == ARM_MATH_SUCCESS) {
		return FFT_OK;
	} else {
		return FFT_ERROR;
	}
}

/*
 *
 * Оконная функция применяется для улучшения качества выходного спектра путем
 * уменьшения амплитуды на разрыве границ окна
 */
static void Hann_Window_f32(float32_t* arr)
{
	for (int i = 0; i < FFT_SIZE; i++) {
	    arr[i] = arr[i] * (0.5 * (1.0 - cos(2.0*PI*i/FFT_SIZE)));
	}
}

static void Hann_Window_Q15(q15_t* arr)
{
	for (int i = 0; i < FFT_SIZE; i++) {
	    arr[i] = arr[i] * (0.5 * (1.0 - cos(2.0*PI*i/FFT_SIZE)));
	}
}

uint8_t FFT_Do_F32(float32_t* in, float32_t* out){
	assert_param(type == FFT_TYPE_F32);

	float32_t buf[FFT_SIZE];
	Hann_Window_f32(in);
	/*
	 * 3 параметр функции имеет размер FFT_SIZE * 2 и формат buf[0]-вещественное, buf[1]-мнимое число ...
	 */
	arm_rfft_fast_f32(&f32i, in, buf, 0);

	/*
	 * получение амплитуды из массива комплексных чисел
	 */
	arm_cmplx_mag_f32(buf, out, FFT_SIZE);

	return FFT_OK;

}

uint8_t FFT_Do_Q15(q15_t* in, q15_t* out){
	assert_param(type == FFT_TYPE_Q15);
	Hann_Window_Q15(in);
	q15_t buf[FFT_SIZE * 2];
	/*
	 * 3 параметр функции имеет размер FFT_SIZE * 2 и формат buf[0]-вещественное, buf[1]-мнимое число ...
	 */
	arm_rfft_q15(&q15i, in, buf);

	// Сдвигаем на 8 бит в соответствии с документацией
	arm_shift_q15(buf, 8, buf, FFT_SIZE);
	/*
	 * получение амплитуды из массива комплексных чисел
	 */
	arm_cmplx_mag_q15(buf, out, FFT_SIZE);

	return FFT_OK;
}
