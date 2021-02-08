/*
 * com.h
 *
 *  Created on: Dec 10, 2020
 *      Author: pavel
 */

#ifndef INC_COM_H_
#define INC_COM_H_

#include <stdint.h>
#include <arm_math.h>

#define COM_ERROR_OK 0
#define COM_ERROR_ALREADY_INITED 1
#define COM_ERROR_NO_MEMORY 2

#define COM_TRANSMIT_OK 0
#define COM_TRANSMIT_BUSY 1
#define COM_TRANSMIT_LOCKED 2
#define COM_TRANSMIT_ERROR 3

typedef struct {
	int16_t* signal;
	int16_t* spect;
	int8_t ch1val;
	int8_t ch2val;
	int8_t ch3val;
} SLP_Com;


typedef struct {
	uint16_t signalLen;
	uint16_t spectLen;
	uint16_t bufBytesLen;
	uint8_t* buf;
	uint32_t lockedFor;
	uint8_t state;
} SLP_ComHandler;

uint8_t ComInit(int16_t signalLen, int16_t spectLen);
void ComDeInit();
uint8_t ComTransmit (SLP_Com* data);
void ComDataPackedCallback();


#endif /* INC_COM_H_ */
