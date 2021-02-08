/*
 * slp.c
 *
 *  Created on: 23 дек. 2017 г.
 *      Author: pavel
 */
#include "slp.h"
#include "fft.h"
#include "com.h"
#include <stdio.h>
#include "pdm2pcm_glo.h"
#include "common.h"

#define MAX_CHANNELS				8


#define AUDIO_BIT_RESOLUTION    	16 //16 BIT
#define DECIMATION_FACTOR			64
#define PDM_BUF_SIZE 				DECIMATION_FACTOR / AUDIO_BIT_RESOLUTION * FFT_SIZE // 4096

#define SIGNAL_LEN 					128
#define SPECT_LEN 					64

#define LIGHT_LEVEL 				0.5
#define FADE_PERIOD 				1000

#define DATA_UNLOCKED 				0
#define DATA_LOCKED 				1

#define __LOCK_DATA  fftDataLock = DATA_LOCKED
#define __UNLOCK_DATA fftDataLock = DATA_UNLOCKED



#define HTONS(A)  ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))

SLP_ChannelTypeDef ch[MAX_CHANNELS];
uint16_t c = 0;

static TIM_HandleTypeDef* htimPWM;
static I2S_HandleTypeDef* hi2sAudio;
static PDM_Filter_Handler_t  hpdm;
static PDM_Filter_Config_t   hpdmConf;

static uint16_t pdmBuf[PDM_BUF_SIZE * 2]; // * 2 cause dma half & full cplt callback
static int16_t pcmBuf[FFT_SIZE];

volatile uint8_t fftDataLock = DATA_UNLOCKED;

void PDMInit()
{
	/* Enable CRC peripheral to unlock the PDM library */
	__HAL_RCC_CRC_CLK_ENABLE();

	/* Init PDM filters */
	hpdm.bit_order  = PDM_FILTER_BIT_ORDER_LSB;
	hpdm.endianness = PDM_FILTER_ENDIANNESS_BE;
	hpdm.high_pass_tap = 2122358088;
	hpdm.out_ptr_channels = 1;
	hpdm.in_ptr_channels  = 1;
	PDM_Filter_Init(&hpdm);

	/* PDM lib config phase */
	hpdmConf.output_samples_number = FFT_SIZE; // need 1024 samples
	hpdmConf.mic_gain = 32;
	hpdmConf.decimation_factor = PDM_FILTER_DEC_FACTOR_64;
	PDM_Filter_setConfig(&hpdm, &hpdmConf);
}


void ProccessI2SCallback (I2S_HandleTypeDef *hi2s, uint16_t index){
	if (hi2s == hi2sAudio) {
		if (fftDataLock == DATA_UNLOCKED) {
			PDM_Filter((uint8_t*)&pdmBuf[index], (uint16_t*)&pcmBuf[0], &hpdm);
		}
	}
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	ProccessI2SCallback(hi2s, 0);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	ProccessI2SCallback(hi2s, PDM_BUF_SIZE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *ht)
{
	/*
	 if (ht->Instance == TIM6) {
		int i = 0;
		for (i = 0; i < 3; i++){
			if(ch[i].value >= 255) {
				ch[i].direction = 0;
				ch[i].value = 254;
			} else if (ch[i].value <= 0){
				ch[i].direction = 1;
				ch[i].value = 1;
			} else {
				if (ch[i].direction == 1) {
					ch[i].value++;
				} else {
					ch[i].value--;
				}
			}
		}
		__HAL_TIM_SET_COMPARE(htimPWM, TIM_CHANNEL_1, ch[0].value);
		__HAL_TIM_SET_COMPARE(htimPWM, TIM_CHANNEL_2, ch[1].value);
		__HAL_TIM_SET_COMPARE(htimPWM, TIM_CHANNEL_3, ch[2].value);
	}
	*/
}
/*
float32_t CalcWeight(uint16_t len){
	float32_t wsum = 0;
	for (int i = 0; i < len; i++) {
			wsum += 0.5 * (1.0 - cos(2.0*PI*i/(len - 1)));
	}
	return wsum;
}


float32_t Val(float32_t* dataIn, uint16_t cindex, uint16_t from, uint16_t to)
{
	//Возможно переполнение (но маловероятно... meeeh)
	float32_t sum = 0;
	uint16_t len = to - from;
	uint16_t j = 0;
	for (int i = from; i < to; i++, j++) {
		sum += dataIn[i] * (0.5 * (1.0 - cos(2.0*PI*j/(len - 1))));
	}
	return sum/ch[cindex].wsum;
}

void CalcF32(float32_t *data) {
	for (int i = 0; i < c; i++){
		float32_t res = 0;
		uint32_t index = 0;
		arm_max_f32 (&data[ch[i].beginFreq], ch[i].endFreq - ch[i].beginFreq, &res, &index);
		if (res > ch[i].noize) {
			if (res > ch[i].max) {
				ch[i].value = 1.0;
			} else {
				float32_t level = ch[i].max - ch[i].noize;
				ch[i].value = res - level / level;
			}
		} else {
			ch[i].value = 0.0;
		}
	}


	for(int i = 0; i < c; i++){
		float32_t absval = Val(data, i, ch[i].beginFreq, ch[i].endFreq);
		if(absval > ch[i].noize)
		{
			if(ch[i].max < absval){
				ch[i].max = absval;
				ch[i].value = 1.0;
			} else {
				float32_t thres = ch[i].max - ch[i].max * ch[i].threshold;
				if(thres < absval){
					ch[i].value = 1 - ((ch[i].max - absval) / (ch[i].max - thres));
				} else {
					ch[i].value = 0.0;
				}
			}
		} else {
			ch[i].value = 0.0;
		}
	}
}
*/

void UpdateLed() {
	uint32_t i = 0;
	for (i = 0; i < c; i++){
		__HAL_TIM_SET_COMPARE(htimPWM, ch[i].channel, ch[i].value);
	}
}

void Calc(int16_t *data) {
	for (int i = 0; i < c; i++){
		int16_t res = 0;
		uint32_t index = 0;
		float32_t val = 0.0;
		arm_max_q15 (&data[ch[i].beginFreq], ch[i].endFreq - ch[i].beginFreq, &res, &index);
		if (res > ch[i].noize) {
			if (res > ch[i].max) {
				val = 1.0;
			} else {
				val =  (float32_t) (res - ch[i].noize) / ch[i].max;
			}
		} else {
			val = 0.0;
		}
		ch[i].value = 255 * val;
	}
}

void ComDataPackedCallback (SLP_ComHandler* h) {
	__UNLOCK_DATA;
}

void ApproximateSpect(int16_t* src, int16_t* dst, uint16_t srcLen, uint16_t dstLen) {
	assert_param(srcLen > dstLen && srcLen % dstLen == 0);
	uint16_t range = srcLen / dstLen;
	uint16_t i = 0;
	uint16_t j = 0;
	for (i = 0; i < dstLen; i++){
		uint32_t p;
		arm_max_q15(src + j, range, &dst[i], &p);
		j+=range;
	}
}

void SLP_Process() {
	int16_t comSignal[SIGNAL_LEN];
	int16_t comSpect[SPECT_LEN];
	int16_t inBuf[FFT_SIZE];
	int16_t out[FFT_SIZE];

	__LOCK_DATA;
	memcpy(comSignal, pcmBuf, sizeof(int16_t) * SIGNAL_LEN);
	memcpy(inBuf, pcmBuf, sizeof(int16_t) * FFT_SIZE);
	__UNLOCK_DATA;

	/*
	 * Шаг частоты высчитывается, как: (частота дискретизации)/(размер преобразования)
	 * Частота бина высчитывается, как: (шаг частоты)*(индекс бина)
	 * Для нормирования спектра: s[i] := s[i] / (размер преобразования)
	 *
	 * Увеличение частоты дискретизации ведет к уменьшеню разрешающей способности преобразовния, например, увеличение частоты дискретизации
	 * в 2 раза ведет к увеличению шагу частоты бина, так же в 2 раза, но дает возможность распознать более высокие частоты,
	 * согласно теоремы Котельникова(Найквеста) до половины частоты дискретизации.
	 *
	 * Увеличение размера Fft преобразования ведет к тому, что короткоимпулсный, непостоянный, сигнал может быть "не замечен".
	 */
	FFT_Do_Q15(inBuf, out);
	Calc(out);
	UpdateLed();

	ApproximateSpect(out, comSpect, FFT_SIZE, SPECT_LEN);

	SLP_Com data;
	data.spect = comSpect;
	data.signal = comSignal;
	data.ch1val = ch[0].value;
	data.ch2val = ch[1].value;
	data.ch3val = ch[2].value;
	uint8_t tRes = ComTransmit(&data);
}

void SLP_Init(TIM_HandleTypeDef* htim, TIM_HandleTypeDef* htimb, I2S_HandleTypeDef* hi2s2, uint16_t samplerate, uint16_t size)
{
	htimPWM = htim;
	hi2sAudio = hi2s2;
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(htimb);

	HAL_I2S_Receive_DMA(hi2s2, pdmBuf, PDM_BUF_SIZE * 2);

	PDMInit();
	FFT_Init(FFT_TYPE_Q15);
	ComInit(SIGNAL_LEN, SPECT_LEN);
}


uint8_t SLP_AddChannel(SLP_ChannelTypeDef channel)
{
	if (c < MAX_CHANNELS){
		ch[c] = channel;
		c++;
		return SLP_OK;
	}
	return SLP_ERROR;
}

uint8_t SLP_RemoveChannel(uint16_t index)
{
	return SLP_OK;
}

uint8_t SLP_ReplaceChannel(uint16_t index, SLP_ChannelTypeDef channel)
{
	if(index < c){
		ch[index] = channel;
		return SLP_OK;
	}
	return SLP_ERROR;
}

float32_t SLP_GetChannelValue(uint16_t index)
{
	if (index < c){
		return ch[index].value;
	}
	return 0;
}

uint16_t SLP_GetChannelsCount(){
	return c;
}






