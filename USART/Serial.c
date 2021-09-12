/*
 * serial.c
 *
 *  Created on: Jul 4, 2020
 *      Author: olegkaliuzhnyi
 */

#include "Serial.h"

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_dma.h"


bool _Serial_ReadLinear(Serial* serial, uint32_t maxLength, char* outData, uint32_t* outLength);
bool _Serial_ReadOverflow(Serial* serial, uint32_t maxLength, char* outData, uint32_t* outLength);
uint32_t _Serial_WaitForCharInBuffer(Serial* serial, char expectedChar);
bool _Serial_IsReadIndexSame(Serial* serial);


void Serial_Init(Serial* serial, USART_TypeDef* USARTx)
{
	serial->USARTx = USARTx;
	serial->RxBufferIndex = 0;
	serial->RxReadIndex = 0;
	serial->RxBufferLength = SERIAL_RX_BUFFER_LENGTH;
//	uint8_t* buffer = &serial->RxBuffer[0];
//
//	for (uint32_t index = 0; index < SERIAL_RX_BUFFER_LENGTH; ++index)
//		buffer[index] = 0;
}

void Serial_Deinit(Serial* serial)
{
	serial->RxBufferLength = 0;
}

bool Serial_IsInited(Serial* serial)
{
	return serial->RxBufferLength > 0;
}

void Serial_SendByte(Serial* serial, uint8_t byte)
{
	USART_TypeDef* usart = serial->USARTx;
	while (!LL_USART_IsActiveFlag_TXE(usart)) {} // waiting for TXE to be set to SR register
		LL_USART_TransmitData8(usart, byte);
}

void Serial_SendBytes(Serial* serial, uint8_t* data, uint32_t size)
{
	for	(uint32_t i = 0; i < size; i++)
	{
		Serial_SendByte(serial, *(uint8_t*)(data + i));
	}
}

void Serial_SendString(Serial* serial, char* str)
{
	uint32_t i = 0;
	char current = *str;
	while (current != '\0')
	{
		Serial_SendByte(serial, (uint8_t)current);

		i++;
		current = *(char*)(str + i);
	}
}

void Serial_SendLine(Serial* serial, char* str)
{
	Serial_SendString(serial, str);
	Serial_SendString(serial, "\n");
}

void Serial_HandleRxInterrupt(Serial* serial)
{
	USART_TypeDef* usart = serial->USARTx;

	if(LL_USART_IsActiveFlag_RXNE(usart) && LL_USART_IsEnabledIT_RXNE(usart))
	{
		uint8_t data = LL_USART_ReceiveData8(usart);
		serial->RxBuffer[serial->RxBufferIndex] = data;

		if (serial->RxBufferIndex == serial->RxBufferLength - 1)
			serial->RxBufferIndex = 0;
		else
			serial->RxBufferIndex++;

	}
	else if(LL_USART_IsActiveFlag_ORE(usart))
	{
	  (void) usart->DR; // read from DR to reset ORE flag (overflow error)
	}
	else if(LL_USART_IsActiveFlag_FE(usart))
	{
	  (void) usart->DR; // read from DR to reset FE flag (frame receiving error, no stop bit)
	}
	else if(LL_USART_IsActiveFlag_NE(usart))
	{
	  (void) usart->DR; // read from DR to reset NE flag (signal noise error)
	}
	else if(LL_USART_IsActiveFlag_PE(usart))
	{
	  (void) usart->DR; // read from DR to reset PE flag (parity error)
	}
}

void Serial_HandleRxDMA(Serial* serial, DMA_TypeDef* DMAx, uint32_t LL_DMA_CHANNEL_x)
{
	uint32_t bufferLength = serial->RxBufferLength;

	uint32_t currentPosition = bufferLength - LL_DMA_GetDataLength(DMAx, LL_DMA_CHANNEL_x);

	serial->RxBufferIndex = currentPosition;

	if (serial->RxBufferIndex == bufferLength)
	{
		serial->RxBufferIndex = 0;
	}
}

uint32_t Serial_Available(Serial* serial)
{
	if (_Serial_IsReadIndexSame(serial))
	{
		return 0;
	}

	if (serial->RxBufferIndex > serial->RxReadIndex)
	{
		return serial->RxBufferIndex - serial->RxReadIndex;
	}

	return serial->RxBufferLength - serial->RxReadIndex + serial->RxBufferIndex;
}

bool Serial_Read(Serial* serial, uint32_t maxLength, char* outData, uint32_t* outLength)
{
	*outLength = 0;

	if (_Serial_IsReadIndexSame(serial))
	{
		return false;
	}

	if (serial->RxBufferIndex > serial->RxReadIndex)
	{
		return _Serial_ReadLinear(serial, maxLength, outData, outLength);
	}

	return _Serial_ReadOverflow(serial, maxLength, outData, outLength);
}

bool Serial_ReadStringUntil(Serial* serial, char expectedChar, uint32_t maxLength, char* outData, uint32_t* outLength)
{
	*outLength = 0;

	if (_Serial_IsReadIndexSame(serial))
	{
		return false;
	}

	uint32_t length = _Serial_WaitForCharInBuffer(serial, expectedChar);

	if (length == 0)
	{
		// buffer has expected char but nothing else before
		serial->RxReadIndex++;
		return true; // because expected char found
	}

	if (length > maxLength)
	{
		return false;
	}

	uint32_t start = serial->RxReadIndex;
	for (uint32_t i = 0; i < length; i++)
	{
		uint32_t bufferIndex = (start + i) % serial->RxBufferLength;
		outData[i] = (char)serial->RxBuffer[bufferIndex];
	}

	outData[length] = '\0';
	*outLength = length;
	serial->RxReadIndex = (serial->RxReadIndex + length + 1) % serial->RxBufferLength;

	return true;
}


bool _Serial_ReadLinear(Serial* serial, uint32_t maxLength, char* outData, uint32_t* outLength)
{
	const uint32_t uint8Size = sizeof(uint8_t);
	const uint32_t rxBufferIndex = serial->RxBufferIndex;
	uint32_t length = rxBufferIndex - serial->RxReadIndex;
	uint32_t size = uint8Size * length;

	if (length > maxLength)
		return false;

	memcpy(outData, &serial->RxBuffer[serial->RxReadIndex], size);
	*outLength = length;
	serial->RxReadIndex = rxBufferIndex;
	return true;
}

bool _Serial_ReadOverflow(Serial* serial, uint32_t maxLength, char* outData, uint32_t* outLength)
{
	const uint32_t uint8Size = sizeof(uint8_t);
	const uint32_t rxBufferIndex = serial->RxBufferIndex;

	uint32_t length = serial->RxBufferLength - serial->RxReadIndex + rxBufferIndex;

	if (length > maxLength)
			return false;

	/* Copy end part of the buffer */
	uint32_t copySize1 = uint8Size * (serial->RxBufferLength - serial->RxReadIndex);
	memcpy(outData, &serial->RxBuffer[serial->RxReadIndex], copySize1);

	/* Copy start part of the buffer */
	uint32_t copySize2 = uint8Size * rxBufferIndex;
	memcpy(outData + copySize1 + 1, serial->RxBuffer, copySize2);

	*outLength = length;
	serial->RxReadIndex = rxBufferIndex;

	return true;
}

uint32_t _Serial_WaitForCharInBuffer(Serial* serial, char expectedChar)
{
	uint32_t index = serial->RxReadIndex;
	uint32_t length = 0;

	char current = (char)serial->RxBuffer[index];
	while(current != expectedChar)
	{
		if (index == serial->RxBufferLength - 1)
			index = 0;
		else
			index++;

		while (index == serial->RxBufferIndex); /* wait for new data */

		length++;
		if (length >= serial->RxBufferLength)
		{
			length = 0;
			break;
		}

		current = (char)serial->RxBuffer[index];
	}

	return length;
}

bool _Serial_IsReadIndexSame(Serial* serial)
{
	return serial->RxBufferIndex == serial->RxReadIndex;
}
