#pragma once

#include <stdint.h>

// used to determine what, if anything, went wrong; call irmGetError() to get the most recent uncleared error
typedef enum IRM_ERROR
{
	IRM_ERR_NONE = 0,			// no error
	IRM_ERR_PREAMBLE_SHORT,		// a preamble was detected, but it was too short
	IRM_ERR_PREAMBLE_LONG,		// a preamble was detected, but it was too long
	IRM_ERR_TIMEOUT,			// a preamble was detected, and message reading started taking place, but we gave up waiting for the message content
	IRM_ERR_INVALID_LENGTH,		// a message was received, but it was ignored because it was determined from the header to be too long to store given the user's provided buffer length
	IRM_ERR_INVALID_CRC			// a message of a valid length was received, but the checksum failed
} IRMError;

// call this to initialize the IRM pins and begin RX interrupts; this tells IRM to use `buffer` to store the most recently-
// received IR message, and specifies the maximum length `len` of messages that can be stored or received there; messages
// containing more than `len` bytes will be ignored and will not be available for reading by the user
void irmInit(uint8_t *buffer, uint8_t len);

// disables IR RX interrupts
void irmShutdown();

// returns 1 if there is a message waiting to be processed; if so, you can call irmGetRXLength() to get the length of this
// message, process it, and then call irmClearRX() to put the system in a state where it can receive the next message
uint8_t irmHasRX();

// used to retrieve the length of the latest message available as indicated by irHasRX(); the result of this function,
// if called outside of irmHasRX() and irmClearX() calls, is undefined
uint8_t irmGetRXLength();

// signals to IRM that you are done with the most recently-received message and that it can begin listening to IR signals again;
// this MUST be called after irmRX() returns 1 and you are done processing that message in your program logic, otherwise no
// further messages will be received; be sure to only call this method once you are done reading the `buffer` provided in
// your earlier call to irmInit()
void irmClearRX();

// used to get the last error that occurred before irmClearError() was called; newer errors will not change this value until you
// call irmClearError() first
IRMError irmGetError();

// used to clear the last error; even a successful RX will not do it, so you must call this
void irmClearError();

// this sends an IR message in a blocking fashion; you can send messages of any (8-bit) length you want, regardless of the
// `len` value you specified for irmInit(), but keep in mind any receivers with buffer lengths shorter than your message
// length will not be able to receive the message
void irmSendBlocking(uint8_t *message, uint8_t len);
