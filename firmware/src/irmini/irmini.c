#include "irmini/irmini.h"

#include "drivers/clock.h"
#include "drivers/led.h"
#include "drivers/serial.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>
#include <util/crc16.h>

// - - - defines - - - //

// set these to configure what RX pins you're using
#define IRM_RX_PORT PORTD
#define IRM_RX_DIR DDRD
#define IRM_RX_INPUT PIND
#define IRM_RX_PIN 2

// set these to configure what TX pins you're using
#define IRM_TX_PORT PORTB
#define IRM_TX_DIR DDRB
#define IRM_TX_PIN 1

// handy macro
#define FREQ_TO_INTERVAL(x) ((F_CPU / x - 1) / 2)

// turn on or off the hardware PWM pin
#define TX_ON  {TCNT1 = 0; TCCR1A |= (1 << COM1A0); TCCR1B |=  (1 << CS10); }
#define TX_OFF {TCCR1A &= ~(1 << COM1A0); TCCR1B &= ~(1 << CS10); IRM_TX_PORT &= ~(1 << IRM_TX_PIN);}

// turn on or off the IR RX pin change interrupts; we're using pin change interrupt group 2 for this
#define RX_ON  { PCICR |= (1 << PCIE2); PCMSK2 |= (1 << PCINT18); }
#define RX_OFF { PCICR &= ~(1 << PCIE2); PCMSK2 &= ~(1 << PCINT18); }

// note that our receiver uses an active low to denote an IR signal reception
#define HIGH_SIGNAL (!(IRM_RX_INPUT & (1 << IRM_RX_PIN)))

// how long the ON signal for the preamble lasts
#define PREAMBLE_TIME_ON_USECS 800UL

// tolerances for detecting the preamble ON signal
#define PREAMBLE_READ_MIN_USECS 500UL
#define PREAMBLE_READ_MAX_USECS 1100UL

// some TX bit timing vars
// [NOTE]: BIT_HIGH_OFF_USECS is expected to be shorter than BIT_LOW_OFF_USECS
#define BIT_HIGH_OFF_USECS 300				// length of off time when transmitting a high bit
#define BIT_LOW_OFF_USECS 600				// length of off time when transmitting a low bit
#define BIT_END_USECS 300					// length of on time when a bit is done being transmitted
#define PAYLOAD_END_USECS 2000UL			// length of off time after a complete message is sent

// some bit RX timing vars; adjust for your application as necessary, but with a resonator you should be fine
#define BIT_ON_TOLERANCE_USECS 300
#define BIT_OFF_TOLERANCE_USECS 100

// when expected data doesn't arrive in time, this is used to escape some control structures
#define RX_TIMEOUT_USECS 5000UL

// - - - types - - - //

// used to track global state of IR message reception
typedef enum IRM_STATE
{
	IRM_STATE_READING_PREAMBLE = 0,			// currently reading preamble
	IRM_STATE_READING_LENGTH,				// currently reading message length byte
	IRM_STATE_READING_CRC,					// currently reading checksum byte
	IRM_STATE_READING_PAYLOAD,				// currently reading payload bytes
} IRMState;

// used to track state of per-bit byte reading
typedef enum IRM_BIT_STATE
{
	IRM_BIT_STATE_READING_HIGH = 0,			// currently reading the high part of a bit, which has fixed length
	IRM_BIT_STATE_READING_LOW				// currently reading the low part of a bit, which has variable length
} IRMBitState;

// used for determining the status of ongoing byte-reading operations
typedef enum IRM_BYTE_READ_STATUS
{
	IRM_BYTE_READ_STATUS_IN_PROGRESS = 0,	// attempting to read current byte, nothing unusual
	IRM_BYTE_READ_STATUS_ERROR,				// failed to read current byte
	IRM_BYTE_READ_STATUS_SUCCESS			// successfully read current byte
} IRMByteReadStatus;

// - - - private vars - - - //

uint8_t *irmBuffer;							// buffer specified by irmSetBuffer()
uint8_t irmMaxPayloadLen;					// maximum payload length specified by irmSetBuffer()

volatile IRMError irmLastError;				// last error we encountered
volatile uint8_t irmRXAvailable;			// do we have a message the user can process?
volatile uint8_t irmRXCRC;					// most recently received CRC byte
volatile uint8_t irmRXLen;					// most recently received length byte

volatile uint8_t irmRXPin;					// current value of the RX pin
volatile uint32_t irmLastPinChangeTime;		// time in microseconds that we experienced the last RX pin change

IRMState irmState;							// current state of the IRM library
uint8_t irmCurrentPayloadIndex;				// index of current byte in the payload that is being received

IRMBitState irmBitState;					// current state of reading the currently expected byte
uint8_t irmBitIndex;						// index of current bit (0-7) currently being received

// - - - private prototypes - - - //

// state machine management
void irmResetRXState();
void irmResetBitState();
void irmUpdate(uint32_t usecs);
void irmUpdateStateReadingPreamble(uint32_t usecs);
void irmUpdateStateReadingLength(uint32_t usecs);
void irmUpdateStateReadingCRC(uint32_t usecs);
void irmUpdateStateReadingPayload(uint32_t usecs);

// TX/RX management
IRMByteReadStatus irmReadByte(uint32_t usecs, volatile uint8_t *result);
void irmTransmitByte(uint8_t byte);
uint8_t irmComputeCRC(uint8_t *buffer, uint8_t len);

// other helpful functions
void irmSetError(IRMError err);

// - - - implementations - - - //

void irmInit(uint8_t *buffer, uint8_t len)
{
	// save pointer to user-specified buffer memory
	irmBuffer = buffer;
	irmMaxPayloadLen = len;

	// reset internal timers and state
	irmLastPinChangeTime = 0;
	irmClearError();
	irmClearRX();
	irmResetRXState();

	// configure output/TX pin
	IRM_TX_DIR |= (1 << IRM_TX_PIN);
	IRM_TX_PORT &= ~(1 << IRM_TX_PIN);

	// configure input/RX pin
	IRM_RX_DIR &= ~(1 << IRM_RX_PIN);
	irmRXPin = 0;

	// enable pin change interrupts on our RX pin
	RX_ON;		// enable pin change interrupt group 2, and enable pin change interrupt for PCINT18 (PD2) specifically
	sei();		// enable interrupts globally
}

void irmShutdown()
{
	// disable TX timer
	TCCR1A = 0;
	TCCR1B = 0;

	// shut down TX pin
	IRM_TX_PORT &= ~(1 << IRM_TX_PIN);

	// shut down RX interrupts
	RX_OFF;
}

uint8_t irmHasRX()
{
	return irmRXAvailable;
}

uint8_t irmGetRXLength()
{
	return irmRXLen;
}

uint8_t irmGetReceivedLength()
{
	return irmRXLen;
}

void irmClearRX()
{
	// clear state, CRC, and received length
	irmRXAvailable = 0;
	irmRXCRC = 0;
	irmRXLen = 0;
}

// used to get the last error that occurred before irmClearErr() was called
IRMError irmGetError()
{
	return irmLastError;
}

// used to clear the last error; even a successful RX will not do it, so you must call this
void irmClearError()
{
	irmLastError = IRM_ERR_NONE;
}

void irmSendBlocking(uint8_t *message, uint8_t len)
{
	uint16_t i;

	// length of message must be non-zero
	if(len > 0)
	{
		// configure PWM output; turn off global clock and configure timer 1 for IR TX
		clockPause();
		OCR1A = FREQ_TO_INTERVAL(38000);		// set to 38kHz
		TCCR1A = 0;
		TCCR1B = 0;
		TCCR1B |= (1 << WGM12);					// clear-on-compare match (CTC) mode

		// send preamble
		TX_ON;
		_delay_us(PREAMBLE_TIME_ON_USECS);
		TX_OFF;

		// send length byte
		irmTransmitByte(len);

		// send CRC byte
		irmTransmitByte(irmComputeCRC(message, len));

		// send payload
		for(i = 0; i < len; ++ i)
		{
			irmTransmitByte(message[i]);
		}

		// wait in the OFF state for a bit
		_delay_us(PAYLOAD_END_USECS);

		// re-enable global clock on timer 1
		clockResume();
	}
}

// - - - private functions - - - //

// called by pin change ISR
void irmUpdate(uint32_t usecs)
{
	// do nothing if we're waiting on higher-level logic to process a buffered message or clear an error
	if(!irmRXAvailable)
	{
		// if we've waited too long for anything, reset
		if(usecs > RX_TIMEOUT_USECS)
		{
			irmResetRXState();
			irmClearRX();
		}

		// perform an update step according to current state
		if     (irmState == IRM_STATE_READING_PREAMBLE)	irmUpdateStateReadingPreamble(usecs);
		else if(irmState == IRM_STATE_READING_LENGTH)	irmUpdateStateReadingLength(usecs);
		else if(irmState == IRM_STATE_READING_CRC)		irmUpdateStateReadingCRC(usecs);
		else if(irmState == IRM_STATE_READING_PAYLOAD)	irmUpdateStateReadingPayload(usecs);
	}
}

void irmUpdateStateReadingPreamble(uint32_t usecs)
{
	// initially, we read a high value, but is the pin now reading low?
	if(!irmRXPin)
	{
		// was the preamble shorter than allowed?
		if(usecs <= PREAMBLE_READ_MIN_USECS)
		{
			// preamble was too short
			irmSetError(IRM_ERR_PREAMBLE_SHORT);
			irmResetRXState();
		}
		else if(usecs >= PREAMBLE_READ_MAX_USECS)
		{
			// preamble was too long
			irmSetError(IRM_ERR_PREAMBLE_LONG);
			irmResetRXState();
		}
		else
		{
			// preamble was just right; we now expect some off time before reading the end-of-bit 'high' signal
			irmState = IRM_STATE_READING_LENGTH;
			irmResetBitState();
		}
	}
}

void irmUpdateStateReadingLength(uint32_t usecs)
{
	IRMByteReadStatus byteReadStatus;

	// attempt to read the length byte
	byteReadStatus = irmReadByte(usecs, &irmRXLen);
	if(byteReadStatus == IRM_BYTE_READ_STATUS_ERROR)
	{
		// error; we need to reset
		irmSetError(IRM_ERR_TIMEOUT);
		irmResetRXState();
	}
	else if(byteReadStatus == IRM_BYTE_READ_STATUS_SUCCESS)
	{
		// zero length is not allowed
		if(irmRXLen == 0)
		{
			// the message cannot have zero length; failure
			irmSetError(IRM_ERR_INVALID_LENGTH);
			irmResetRXState();
		}
		// make sure length is within our buffer's limits
		else if(irmRXLen <= irmMaxPayloadLen)
		{
			// we can successfully store a message of this length; proceed to reading CRC
			irmState = IRM_STATE_READING_CRC;
			irmResetBitState();
		}
		else
		{
			// the message is too long; failure
			irmSetError(IRM_ERR_INVALID_LENGTH);
			irmResetRXState();
		}
	}
}

void irmUpdateStateReadingCRC(uint32_t usecs)
{
	IRMByteReadStatus byteReadStatus;

	// attempt to read the length byte
	byteReadStatus = irmReadByte(usecs, &irmRXCRC);
	if(byteReadStatus == IRM_BYTE_READ_STATUS_ERROR)
	{
		// error; we need to reset
		irmSetError(IRM_ERR_TIMEOUT);
		irmResetRXState();
	}
	else if(byteReadStatus == IRM_BYTE_READ_STATUS_SUCCESS)
	{
		// proceed to read the payload
		irmState = IRM_STATE_READING_PAYLOAD;
		irmResetBitState();
		irmCurrentPayloadIndex = 0;
	}
}

void irmUpdateStateReadingPayload(uint32_t usecs)
{
	IRMByteReadStatus byteReadStatus;

	// do we have any bytes left to read in the message?
	if(irmCurrentPayloadIndex < irmRXLen)
	{
		// attempt to read the next byte
		byteReadStatus = irmReadByte(usecs, &irmBuffer[irmCurrentPayloadIndex]);
		if(byteReadStatus == IRM_BYTE_READ_STATUS_ERROR)
		{
			irmSetError(IRM_ERR_TIMEOUT);
			irmResetRXState();
		}
		else if(byteReadStatus == IRM_BYTE_READ_STATUS_SUCCESS)
		{
			// advance to next byte in the payload and prepare to read it
			irmCurrentPayloadIndex ++;
			irmResetBitState();

			// if we are done reading bytes, validate the CRC
			if(irmCurrentPayloadIndex >= irmRXLen)
			{
				// check the CRC
				if(irmRXCRC == irmComputeCRC(irmBuffer, irmRXLen))
				{
					// we successfully read an incoming message and it appears to not be corrupted
					irmRXAvailable = 1;
					irmResetRXState();
				}
				else
				{
					// message received, but checksum failed
					irmSetError(IRM_ERR_INVALID_CRC);
					irmResetRXState();
				}
			}
			// if we waited too long for the rest of the payload, we time out
			else if(usecs > RX_TIMEOUT_USECS)
			{
				// time out error because we never got the data we expected
				irmSetError(IRM_ERR_TIMEOUT);
				irmResetRXState();
			}
		}
	}
}

void irmResetRXState()
{
	irmState = IRM_STATE_READING_PREAMBLE;
	irmCurrentPayloadIndex = 0;
}

void irmResetBitState()
{
	// record start time and prepare to wait out the low period
	irmBitState = IRM_BIT_STATE_READING_LOW;
	irmBitIndex = 0;
}

IRMByteReadStatus irmReadByte(uint32_t usecs, volatile uint8_t *byte)
{
	// default is to say we're currently reading, with nothing to report yet
	IRMByteReadStatus result = IRM_BYTE_READ_STATUS_IN_PROGRESS;

	// bitness is determined by how long the low state lasts
	if(irmBitState == IRM_BIT_STATE_READING_LOW)
	{
		// has the signal become high?
		if(irmRXPin)
		{
			// make sure the signal hasn't been low for too long
			if(usecs < (BIT_LOW_OFF_USECS + BIT_HIGH_OFF_USECS))
			{
				// determine if the off time was less than the halfway time between the off and on time; and if it was...
				if(usecs < ((BIT_HIGH_OFF_USECS + BIT_LOW_OFF_USECS) / 2))
				{
					// ...it's a 1!
					*byte = ((*byte >> 1) | 0x80);
				}
				else
				{
					// ...it's a 0!
					*byte = ((*byte >> 1) & 0x7f);
				}
			
				// now we are reading the high portion
				irmBitState = IRM_BIT_STATE_READING_HIGH;
			}
			else
			{
				// signal was low for too long; this is a timeout error
				result = IRM_BYTE_READ_STATUS_ERROR;
				irmResetRXState();
			}
		}
	}
	else if(irmBitState == IRM_BIT_STATE_READING_HIGH)
	{
		// has the signal become low?
		if(!irmRXPin)
		{
			// make sure the signal wasn't high for too long
			if(usecs < BIT_END_USECS + BIT_ON_TOLERANCE_USECS)
			{
				// have we read all the bits for this byte?
				irmBitIndex ++;
				if(irmBitIndex == 8)
				{
					// indicate to higher-level code that we're done reading the byte
					result = IRM_BYTE_READ_STATUS_SUCCESS;
				}
				else
				{
					// wait for the signal to go up again to read the next bit
					irmBitState = IRM_BIT_STATE_READING_LOW;
				}
			}
			else
			{
				// signal was high for too long; this is a timeout error
				result = IRM_BYTE_READ_STATUS_ERROR;
				irmResetRXState();
			}
		}
	}

	// did we successfully read the byte?
	return result;
}

void irmTransmitByte(uint8_t byte)
{
	uint8_t i;

	for(i = 0; i < 8; ++ i)
	{
		// for a `1` bit, we stay off for BIT_HIGH_OFF_USECS, but for a `0` we stay off for BIT_LOW_OFF_USECS
		if(byte & 0x01) _delay_us(BIT_HIGH_OFF_USECS);
		else			_delay_us(BIT_LOW_OFF_USECS);

		// transmit end-of-bit pulse
		TX_ON;
		_delay_us(BIT_END_USECS);
		TX_OFF;

		// advance
		byte >>= 1;
	}
}

uint8_t irmComputeCRC(uint8_t *buffer, uint8_t len)
{
	uint8_t result = 0;
	uint8_t i;

	// apparently avr-libc has its own 8-bit CRC implementation so we don't have to write our own...nice
	for(i = 0; i < len; ++ i)
	{
		result = _crc8_ccitt_update(result, buffer[i]);
	}

	return result;
}

void irmSetError(IRMError err)
{
	// set a new error if the old one has been cleared
	if(irmLastError == IRM_ERR_NONE)
	{
		irmLastError = err;
	}
}

// used to detect pin logic level changes on our RX pin
ISR(PCINT2_vect)
{
	// compute how long it's been since the last pin state change
	uint32_t us = clockMicros();
	uint32_t dt = us - irmLastPinChangeTime;
	irmLastPinChangeTime = us;
	
	// grab pin state and update IR receiving state machine based on time elapsed since last pin change
	irmRXPin = HIGH_SIGNAL;
	irmUpdate(dt);
}
