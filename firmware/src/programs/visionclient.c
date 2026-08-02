#include "programs/visionclient.h"

#include "drivers/clock.h"
#include "drivers/led.h"
#include "drivers/motor.h"
#include "drivers/button.h"

#include "util/util.h"
#include "util/store.h"

#include "irmini/irmini.h"

#include <stdlib.h>

#include <avr/eeprom.h>

#include <util/delay.h>

// - - - defines - - - //

// buffer size in bytes of the commands expected by the IR remote control device
#define IR_BUFFER_LEN 32

// code indicating a motion command for multiple robots
#define IR_COMMAND_COMPOUND_MOVE 0x02

// how long a move command lasts in milliseconds
#define MOVE_COMMAND_MS 300

// - - - prototypes - - - //

float distanceToLEDFactor(uint8_t led, int16_t pos);

// - - - implementations - - - //

void runVisionClientProgram()
{
	// IR and state control
	uint8_t buffer[IR_BUFFER_LEN];	// buffer in which commands are read from IR
	uint8_t len;					// length of received IR message in bytes
	int8_t left;					// last commanded left motor speed
	int8_t right;					// last commanded right motor speed
	uint8_t done;					// set to 1 iff it's time to quit; triggered by pushbutton
	uint8_t id;						// unique ID of robot
	uint32_t timer;					// used to time motor commands
	uint8_t i;

	// rainbow effect control
	uint8_t dir;					// direction of scroll
	int16_t pos;					// simulated LED position
	uint8_t count;					// used to slowly increment hue value
	uint8_t hue;					// continuous hue value we increment for rainbow effect
	uint8_t r, g, b;				// resulting RGB values based on current hue

	// initialize the IRM library
	irmInit(buffer, IR_BUFFER_LEN);

	// hard brake for precise control
	motorUseHardBrake(1);

	// load our ID from EEPROM store and prime motor command timing
	id = eeprom_read_byte((const uint8_t*)STORE_ID_ADDR);
	timer = 0;

	// loop until user is ready to exit
	hue = 0;
	dir = 0;
	pos = 0;
	count = 0;
	done = 0;
	while(!done)
	{
		// update IR functions; did we get an IR command?
		if(irmHasRX())
		{
			// make sure we've received a compound movement command
			if(buffer[0] == IR_COMMAND_COMPOUND_MOVE)
			{
				// extract length and parse; we skip every other byte to search for an ID matching ours
				len = irmGetRXLength();
				for(i = 1; i < len; i += 2)
				{
					// can we find our ID in the IR data?
					if(buffer[i] == id)
					{
						// extract the motor speeds from the byte that comes immediately after; our
						// speeds range from -100 to 100, so scale and translate them accordingly
						left =  (int16_t)((buffer[i + 1] >>    4) - 8) * 12;
						right = (int16_t)((buffer[i + 1] &  0x0f) - 8) * 12;

						// assign motor speeds
						motorLeft(left);
						motorRight(right);

						// reset the movement timer
						timer = clockMillis() + MOVE_COMMAND_MS;
					}
				}
			}

			// ready for next message
			irmClearRX();
		}

		// shut down motors if motor commands are stale
		if(clockMillis() > timer)
		{
			// shut down motors
			motorLeft(0);
			motorRight(0);

			// LED heartbeat
			ledClear();
			ledSet(0, clockMicros() % 1000000UL < 25000 ? 50 : 0, 0, 0);
			ledRefresh();
		}
		else
		{
			// fancy rainbow pattern; scroll based on direction, and change direction if we reach either end
			if(dir)
			{
				pos += 14;
				if(pos > 4000)
				{
					pos = 4000;
					dir = !dir;
				}
			}
			else
			{
				pos -= 14;
				if(pos < 0)
				{
					pos = 0;
					dir = !dir;
				}
			}

			// each LED is activated based on some distance function to the current position, and we
			// also apply a scrolling hue also influenced by LED position
			count ++;
			if(count > 5)
			{
				count = 0;
				hue += 1;
			}
			ledClear();
			for(i = 0; i < NUM_LEDS; ++ i)
			{
				utilHue2RGB(hue + (i * 30), &r, &g, &b);
				ledSet(i,
					   distanceToLEDFactor(i, pos) * (r / 10),
					   distanceToLEDFactor(i, pos) * (g / 10),
					   distanceToLEDFactor(i, pos) * (b / 10));
			}
			ledRefresh();
		}

		// shut down if power gets too low
		utilHandleLowPower();
	}

	// shut down IR library
	irmShutdown();
}

float distanceToLEDFactor(uint8_t led, int16_t pos)
{
	return (float)clamp(255L - (abs((led * 1000) - pos) / 4), 0, 255) / 255.0f;
}
