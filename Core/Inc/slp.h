/*
 * slp.h
 *
 *  Created on: 23 дек. 2017 г.
 *      Author: pavel
 */

#ifndef SLP_H_
#define SLP_H_

#include <stdint.h>
#include <arm_math.h>
#include <stm32f4xx_hal.h>


#define SLP_WINDOW_DEFAULT 	0

#define SLP_ERROR 			1
#define SLP_OK 				0


typedef struct {
	uint32_t channel;
	uint16_t beginFreq;
	uint16_t endFreq;
	int16_t noize;
	int16_t max;
	uint8_t value;
} SLP_ChannelTypeDef;

void SLP_Init(TIM_HandleTypeDef* htim, TIM_HandleTypeDef* htimb, I2S_HandleTypeDef* hi2s2, uint16_t samplerate, uint16_t size);
uint8_t SLP_AddChannel(SLP_ChannelTypeDef c);
uint8_t SLP_RemoveChannel(uint16_t index);
uint8_t SLP_ReplaceChannel(uint16_t index, SLP_ChannelTypeDef channel);
float32_t SLP_GetChannelValue(uint16_t index);
uint16_t SLP_GetChannelsCount();
void SLP_Process();
void SLP_Fade();

#endif /* SLP_H_ */
