/*
 * Serial.h
 *
 *  Created on: Jul 4, 2020
 *      Author: olegkaliuzhnyi
 */

#ifndef SRC_SERIAL_H_
#define SRC_SERIAL_H_


#include <stdbool.h>
#include <stm32f103xb.h>

#ifndef SERIAL_RX_BUFFER_LENGTH
#define SERIAL_RX_BUFFER_LENGTH 15
#endif

typedef struct
{
	USART_TypeDef* USARTx;
	uint8_t RxBuffer[SERIAL_RX_BUFFER_LENGTH];
	uint32_t RxBufferIndex;
	uint32_t RxBufferLength;
	uint32_t RxReadIndex;
} Serial;


void Serial_Init(Serial* serial, USART_TypeDef* USARTx);

void Serial_Deinit(Serial* serial);

bool Serial_IsInited(Serial* serial);

void Serial_SendByte(Serial* serial, uint8_t byte);

void Serial_SendBytes(Serial* serial, uint8_t* data, uint32_t size);

void Serial_SendString(Serial* serial, char* str);

void Serial_SendLine(Serial* serial, char* str);

void Serial_HandleRxInterrupt(Serial* serial);

void Serial_HandleRxDMA(Serial* serial, DMA_TypeDef* DMAx, uint32_t LL_DMA_CHANNEL_x);

uint32_t Serial_Available(Serial* serial);

bool Serial_Read(Serial* serial, uint32_t maxLength, char* outData, uint32_t* outLength);

bool Serial_ReadStringUntil(Serial* serial, char expectedChar, uint32_t maxLength, char* outData, uint32_t* outLength);


#endif /* SRC_SERIAL_H_ */
