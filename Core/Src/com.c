#include "com.h"
#include "stdlib.h"
#include "usbd_cdc_if.h"
#include "assert.h"
#include "common.h"

// ComP
#define MAGIC 0x50436F6D
#define MAGICEND 0x6D6F4350

#define MAX_TX_SIZE 64

SLP_ComHandler handler;

extern USBD_HandleTypeDef hUsbDeviceFS;

void testUnpack(uint8_t* buf, SLP_Com* to);
inline void unpack(uint8_t* buf, uint16_t* padding, void* data, uint16_t len);
inline void pack(uint8_t* buf, uint16_t* padding, void* data, uint16_t len);

uint8_t ComInit(int16_t signalLen, int16_t spectLen) {
	if(handler.buf == 0) {
		uint16_t souint16= sizeof(uint16_t);
		uint16_t souint32= sizeof(uint32_t);
		handler.lockedFor = 0;
		handler.state = COM_TRANSMIT_OK;
		handler.signalLen = signalLen;
		handler.spectLen = spectLen;


		handler.bufBytesLen = spectLen * souint16;
		handler.bufBytesLen += signalLen * souint16;
		handler.bufBytesLen += souint16 * 3; // ADD LEN
		handler.bufBytesLen += 3 * sizeof(uint8_t); // channels
		handler.bufBytesLen += souint32 * 2; // ADD MAGIC


		handler.buf = malloc(handler.bufBytesLen);
		if(handler.buf != 0){
			return COM_ERROR_OK;
		} else {
			return COM_ERROR_NO_MEMORY;
		}
	} else {
		return COM_ERROR_ALREADY_INITED;
	}
}


void ComDeInit() {
	if(handler.buf != 0) {
		free(handler.buf);
	}
}


void  CDC_TransmitCpltCallback(uint8_t *Buf, uint32_t *Len, uint8_t epnum) {
	handler.state = COM_TRANSMIT_OK;
}

uint8_t ComTransmit (SLP_Com* data) {
	if (((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState == 0) {
		handler.state = COM_TRANSMIT_OK;
	}
	if (handler.state == COM_TRANSMIT_OK) {

		uint16_t padding = 0;
		uint32_t magic = MAGIC;
		uint32_t magicEnd = MAGICEND;
		uint16_t payloadLen = handler.bufBytesLen - sizeof(magic) * 2 - sizeof(uint16_t);

		pack( handler.buf, &padding, &magic, sizeof(magic)); // + 4
		pack( handler.buf, &padding, &payloadLen , sizeof(payloadLen));// + 2

		pack( handler.buf, &padding, &handler.signalLen, sizeof(handler.signalLen));// + 2
		pack( handler.buf, &padding, data->signal, sizeof(int16_t) * handler.signalLen);

		pack( handler.buf, &padding, &handler.spectLen, sizeof(handler.spectLen)); //
		pack( handler.buf, &padding, data->spect, sizeof(int16_t) * handler.spectLen);

		pack( handler.buf, &padding, &data->ch1val, sizeof(data->ch1val));
		pack( handler.buf, &padding, &data->ch2val, sizeof(data->ch2val));
		pack( handler.buf, &padding, &data->ch3val, sizeof(data->ch3val));

		pack( handler.buf, &padding, &magicEnd, sizeof(magic)); // + 4

		assert_param(padding == handler.bufBytesLen);

		ComDataPackedCallback(handler);

		uint8_t res = CDC_Transmit_FS(handler.buf, handler.bufBytesLen);
		if(res == USBD_FAIL){
			handler.state = COM_TRANSMIT_ERROR;
		}else {
			handler.state = COM_TRANSMIT_BUSY;
		}
	} else {
		return COM_TRANSMIT_LOCKED;
	}

	return COM_TRANSMIT_OK;
}


void testUnpack(uint8_t* buf, SLP_Com* to) {
	uint16_t padding = 0;
	uint32_t magic;

	unpack(buf, &padding, &magic, sizeof(magic));

	uint16_t sigLen;
	unpack(buf, &padding, &sigLen, sizeof(sigLen));
	unpack(buf, &padding, to->signal, sizeof(float32_t) * sigLen);

	uint16_t specLen;
	unpack(buf, &padding, &specLen, sizeof(specLen));
	unpack(buf, &padding, to->spect, sizeof(float32_t) * specLen);

	unpack(buf, &padding, &to->ch1val, sizeof(to->ch1val));
	unpack(buf, &padding, &to->ch2val, sizeof(to->ch2val));
	unpack(buf, &padding, &to->ch3val, sizeof(to->ch3val));

	unpack(buf, &padding, &magic, sizeof(magic));
}

inline void unpack(uint8_t* buf, uint16_t* padding, void* data, uint16_t len){
	memcpy(data, buf + *padding, len);
	*padding += len;
}

inline void pack(uint8_t* buf, uint16_t* padding, void* data, uint16_t len)
{
	memcpy(buf + *padding, data, len);
	*padding += len;
}
