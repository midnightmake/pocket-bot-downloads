#include "programs/irforwarding.h"

#include "drivers/serial.h"
#include "drivers/led.h"

#include "irmini/irmini.h"

#include "util/util.h"

#include <stdint.h>

#include <util/delay.h>

// - - - defines - - - //

// baud rate for incoming serial comms
#define SERIAL_BAUD_RATE 19200
#define SERIAL_BUFFER_SIZE 128

// we require NUM_HEADER_FOOTER_BYTES of SERIAL_START_BYTEs and SERIAL_END_BYTEs in between a message
// for us to retransmit it over IR
#define SERIAL_START_BYTE 0xaa
#define SERIAL_END_BYTE 0x55
#define NUM_HEADER_FOOTER_BYTES 3

// not currently used, but maybe we can TX IR in the future too if needed
#define IR_TX_BUFFER_SIZE 4

// - - - types - - - //

typedef enum READ_STATE
{
	STATE_READING_HEADER = 0,
	STATE_READING_PAYLOAD
} ReadState;

// - - - globals - - - //

ReadState readState;
uint8_t numHeaderBytes;
uint8_t numFooterBytes;
uint8_t numPayloadBytes;

// - - - implementations - - - //

void runIRForwardingProgram()
{
	uint8_t irBuffer[IR_TX_BUFFER_SIZE];
	uint8_t serialBuffer[SERIAL_BUFFER_SIZE];
	uint8_t byte;
	
	// setup serial comms
	serialInit();
	serialSetBaudRate(SERIAL_BAUD_RATE);

	// maybe allow the global vision server to receive messages in the future
	irmInit(irBuffer, IR_TX_BUFFER_SIZE);

	// prime serial reading
	readState = STATE_READING_HEADER;
	numHeaderBytes = 0;
	numFooterBytes = 0;
	numPayloadBytes = 0;

	// run forever
	while(1)
	{
		// see if we received any start-of-message bytes
		if(serialRXAvailable())
		{
			// store
			byte = serialRX();

			// header state
			if(readState == STATE_READING_HEADER)
			{
				// did we read a start byte
				if(byte == SERIAL_START_BYTE)
				{
					// enough header bytes means the start of a payload
					numHeaderBytes ++;
					if(numHeaderBytes >= NUM_HEADER_FOOTER_BYTES)
					{
						// enter payload read state
						readState = STATE_READING_PAYLOAD;
						numHeaderBytes = 0;
						numFooterBytes = 0;
						numPayloadBytes = 0;
					}
				}
				else
				{
					// not a header byte; we have to reset
					numHeaderBytes = 0;
				}
			}
			else if(readState == STATE_READING_PAYLOAD)
			{
				// store payload byte; this will overflow if the user is not careful
				serialBuffer[numPayloadBytes++] = byte;

				// did we read a header byte?
				if(byte == SERIAL_START_BYTE)
				{
					// enough header bytes resets the payload state
					numHeaderBytes ++;
					if(numHeaderBytes >= NUM_HEADER_FOOTER_BYTES)
					{
						numHeaderBytes = 0;
						numFooterBytes = 0;
						numPayloadBytes = 0;
					}
				}
				else
				{
					// non-header byte; reset header byte count
					numHeaderBytes = 0;
				}

				// did we read a footer byte?
				if(byte == SERIAL_END_BYTE)
				{
					// enough footer bytes results in a transmission
					numFooterBytes ++;
					if(numFooterBytes >= NUM_HEADER_FOOTER_BYTES)
					{
						// transmit the message over IR, minus the footer bytes
						ledSet(0, 2, 0, 0);
						ledRefresh();
						irmSendBlocking(serialBuffer, numPayloadBytes - NUM_HEADER_FOOTER_BYTES);
						ledClear();
						ledRefresh();

						// reset state and counters
						readState = STATE_READING_HEADER;
						numHeaderBytes = 0;
						numFooterBytes = 0;
						numPayloadBytes = 0;
					}
				}
				else
				{
					// non-footer byte; reset footer byte count
					numFooterBytes = 0;
				}
			}
		}

		// shut down if power gets too low
		utilHandleLowPower();
	}
}
