#include "programs/cylon.h"

#include "drivers/clock.h"
#include "drivers/led.h"
#include "drivers/motor.h"
#include "drivers/button.h"

#include "irmini/irmini.h"

#include "util/util.h"

#include <stdlib.h>

#include <util/delay.h>

// - - - defines - - - //

// buffer size in bytes of the commands expected by the IR remote control device
#define IR_BUFFER_LEN 3

// code indicating a motion command
#define IR_COMMAND_MOVE 0x01

// - - - private functions - - - //

uint8_t distanceToLED(uint8_t led, int16_t pos)
{
	return clamp(255L - (abs((led * 1000) - pos) / 4), 0, 255);
}

// - - - implementations - - - //

// This is a very simple program that just scrolls the RGB LEDs in the classic red "Cylon" animation style.
void runCylonProgram()
{
	// used to control LED flashing
	uint8_t dir;
	int16_t pos;
	uint8_t done;
	uint8_t i;

	// IR control
	uint8_t irBuffer[IR_BUFFER_LEN + 1];

	// clear LEDs
	ledClear();
	ledRefresh();

	// initialize the IRM library
	irmInit(irBuffer, IR_BUFFER_LEN);

	// loop until done
	dir = 0;
	pos = 0;
	done = 0;
	while(!done)
	{
		// update IR functions; did we get an IR command?
		if(irmHasRX())
		{
			// move command
			if(irBuffer[0] == IR_COMMAND_MOVE)
			{
				// endlessly do the cylon thing
				while(!done)
				{
					// scroll based on direction, and change direction if we reach either end
					if(dir)
					{
						pos += 2;
						if(pos > 4000)
						{
							pos = 4000;
							dir = !dir;
						}
					}
					else
					{
						pos -= 2;
						if(pos < 0)
						{
							pos = 0;
							dir = !dir;
						}
					}

					// each red LED is activated based on some distance function to the current position
					for(i = 0; i < NUM_LEDS; ++ i)
					{
						ledSet(i, distanceToLED(i, pos), 0, 0);
					}
					ledRefresh();

					// shut down if button is pressed
					if(buttonDown())
					{
						_delay_ms(50);
						while(buttonDown());
						done = 1;
					}
				}
			}
		}

		// shut down if button is pressed
		if(buttonDown())
		{
			_delay_ms(50);
			while(buttonDown());
			done = 1;
		}
	}
}
