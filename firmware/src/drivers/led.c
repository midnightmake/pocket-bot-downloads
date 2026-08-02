#include "drivers/led.h"

#include <stdint.h>

#include <avr/io.h>

#include <util/delay.h>

// - - - defines - - - //

// if LED clock strobe delays are not required (according to the BB-2020BGR-TRB datasheet, we're fine at 8MHz), we can leave this disabled;
// if we're using slower LEDs or a faster clock in the future then we may need these
//#define USE_CLOCK_STROBE_DELAY

// useful macro for handling timing requirements of our LED communications; enable USE_CLOCK_STROBE_DELAY if delays are required
#define STROBE_US 10
#ifdef USE_CLOCK_STROBE_DELAY
	#define CLOCK_STROBE_DELAY {_delay_us(STROBE_US);}
#else
	#define CLOCK_STROBE_DELAY {}
#endif

// data output
#define DATA_PORT PORTD
#define DATA_DIR DDRD
#define DATA_PIN 4

// clock output
#define CLOCK_PORT PORTD
#define CLOCK_DIR DDRD
#define CLOCK_PIN 7

// - - - static members - - - //

static uint8_t _red[NUM_LEDS];
static uint8_t _green[NUM_LEDS];
static uint8_t _blue[NUM_LEDS];
static uint8_t _needsRefresh;

// - - - static functions - - - //

static void _ledSendByte(uint8_t value);

// - - - implementations - - - //

void ledInit()
{
	uint8_t i;

	// set data pin as output, and off
	DATA_DIR |= (1 << DATA_PIN);
	DATA_PORT &= ~(1 << DATA_PIN);

	// set clock pin as output, and off
	CLOCK_DIR |= (1 << CLOCK_PIN);
	CLOCK_PORT &= ~(1 << CLOCK_PIN);

	// set all LEDs to 0
	_needsRefresh = 0;
	for(i = 0; i < NUM_LEDS; ++ i)
	{
		_red[i] = 0;
		_green[i] = 0;
		_blue[i] = 0;
	}
}

void ledSet(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
	// do a check to make sure index is valid
	if(index < NUM_LEDS)
	{
		// check if changes are needed
		if(_red[index] != red || _green[index] != green || _blue[index] != blue)
		{
			_red[index] = red;
			_green[index] = green;
			_blue[index] = blue;
			_needsRefresh = 1;
		}
	}
}

void ledGet(uint8_t index, uint8_t *red, uint8_t *green, uint8_t *blue)
{
	// do a check to make sure index is valid
	if(index < NUM_LEDS)
	{
		*red = _red[index];
		*green = _green[index];
		*blue = _blue[index];
	}
}

void ledClear()
{
	uint8_t i;

	for(i = 0; i < NUM_LEDS; ++ i)
	{
		ledSet(i, 0, 0, 0);
	}
}

void ledRefresh()
{
	uint8_t i, j;

	// see if a refresh is needed
	if(_needsRefresh)
	{
		// send start frame, which is 32 bits of zeros
		DATA_PORT &= ~(1 << DATA_PIN);
		for(i = 0; i < 32; i ++)
		{
			//CLOCK_STROBE_DELAY;
			CLOCK_PORT |= (1 << CLOCK_PIN);
			//CLOCK_STROBE_DELAY;
			CLOCK_PORT &= ~(1 << CLOCK_PIN);
		}

		// send data frame(s)
		for(i = 0; i < NUM_LEDS; ++ i)
		{
			// send LED frame, which is 3 bits of ones...
			DATA_PORT |= (1 << DATA_PIN);
			for(j = 0; j < 3; ++ j)
			{
				//CLOCK_STROBE_DELAY;
				CLOCK_PORT |= (1 << CLOCK_PIN);
				//CLOCK_STROBE_DELAY;
				CLOCK_PORT &= ~(1 << CLOCK_PIN);
			}

			// 5 bits of brightness...
			// [note]: we don't expose this to the user; presumably they can set their own brightness values implicitly in the RGB data
			DATA_PORT |= (1 << DATA_PIN);
			for(j = 0; j < 5; ++ j)
			{
				//CLOCK_STROBE_DELAY;
				CLOCK_PORT |= (1 << CLOCK_PIN);
				//CLOCK_STROBE_DELAY;
				CLOCK_PORT &= ~(1 << CLOCK_PIN);
			}
		
			// and 8 bits for each BGR channel (32 bits total)
			_ledSendByte(_blue[i]);
			_ledSendByte(_green[i]);
			_ledSendByte(_red[i]);
		}

		// send end frame, which is 32 bits of ones
		DATA_PORT |= (1 << DATA_PIN);
		for(i = 0; i < 32; ++ i)
		{
			//CLOCK_STROBE_DELAY;
			CLOCK_PORT |= (1 << CLOCK_PIN);
			//CLOCK_STROBE_DELAY;
			CLOCK_PORT &= ~(1 << CLOCK_PIN);
		}

		// no need to refresh values now
		_needsRefresh = 0;
	}
}

void _ledSendByte(uint8_t value)
{
	uint8_t i;

	// send one bit at a time
	for(i = 0; i < 8; ++ i)
	{
		// set or clear data pin
		if(value & 0x80) DATA_PORT |= (1 << DATA_PIN);
		else			 DATA_PORT &= ~(1 << DATA_PIN);

		// strobe clock signal
		//CLOCK_STROBE_DELAY;
		CLOCK_PORT |= (1 << CLOCK_PIN);
		//CLOCK_STROBE_DELAY;
		CLOCK_PORT &= ~(1 << CLOCK_PIN);
	
		// next bit, if any
		value <<= 1;
	}
}
