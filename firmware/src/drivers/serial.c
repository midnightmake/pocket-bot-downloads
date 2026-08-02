#include "drivers/serial.h"

#include <avr/io.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// - - - defines - - - //

#define MAX_TEMP_BUFFER_SIZE 128

// - - - private vars - - - //

char tempStr[MAX_TEMP_BUFFER_SIZE];

// - - - implementations - - - //

void serialInit()
{
	// enable serial TX and RX; note that this overrides those pins
	UCSR0B |= (1 << TXEN0);
	UCSR0B |= (1 << RXEN0);
}

void serialSetBaudRate(uint32_t baudRate)
{
	uint32_t ubrr = (F_CPU - (8 * baudRate)) / (16 * baudRate);
	UBRR0 = ubrr;
}

void serialTXRaw(uint8_t *data, uint16_t len)
{
	uint8_t *ptr = data;

	while(len--)
	{
		while(!(UCSR0A & (1 << UDRE0)));
		UDR0 = *ptr++;
	}
}

void serialTX(const char *format, ...)
{
	char *ptr;

	// perform formatting
	va_list args;
	va_start(args, format);
	vsnprintf(tempStr, MAX_TEMP_BUFFER_SIZE, format, args);
	va_end(args);

	// output each char
	ptr = tempStr;
	while(*ptr)
	{
		while(!(UCSR0A & (1 << UDRE0)));
		UDR0 = *ptr++;
	}
}

uint8_t serialRXAvailable()
{
	uint8_t result = UCSR0A & (1 << RXC0);
	return result;
}

uint8_t serialRX()
{
	return UDR0;
}

/*

// - - - implementations - - - //

void serialInit(uint32_t baudRate)
{
	// compute and assign value for baud rate register
	uint32_t ubrr = (F_CPU - (8 * baudRate)) / (16 * baudRate);
	UBRR0 = ubrr;

	// we only care about enabling TX for now
	UCSR0B |= (1 << TXEN0);

	// set data format; 8 data bits, 2 stop bits
	// [todo]: double-check that this is what we need
	UCSR0C |= (1 << USBS0) | (3 << UCSZ00);
}

void serialSendBlocking(uint8_t *data, uint32_t len)
{
	// loop until we're done sending data
	while(len)
	{
		// wait for empty TX buffer
		while(!(UCSR0A & (1 << UDRE0)));

		// put the data into the buffer; this sends the data
		UDR0 = *data++;

		// data was sent
		-- len;
	}
}
*/