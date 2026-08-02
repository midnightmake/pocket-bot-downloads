#include "programs/rave.h"

#include "drivers/led.h"
#include "drivers/motor.h"
#include "drivers/button.h"

#include "util/util.h"

#include <stdlib.h>

#include <util/delay.h>

void runRaveProgram()
{
	uint8_t hue;
	uint8_t r, g, b;
	uint8_t i;
	uint8_t motorsOn;
	uint8_t done;

	// loop until done
	hue = 0;
	motorsOn = 0;
	done = 0;
	while(!done)
	{
		// scroll through hue
		hue += 3;

		// set LEDs
		ledClear();
		for(i = 0; i < NUM_LEDS; ++ i)
		{
			utilHue2RGB(hue + (i * 30), &r, &g, &b);
			ledSet(i, r, g, b);
		}
		ledRefresh();

		// slight delay so the user can see all this at a func pace
		_delay_ms(10);

		// respond to button press
		if(buttonDown())
		{
			// wait for release
			_delay_ms(50);
			while(buttonDown());

			// begin rotating, or quit
			if(motorsOn)
			{
				done = 1;
			}
			else
			{
				// activate motors
				motorsOn = 1;
				motorLeft(-20);
				motorRight(20);
			}
		}
	}
}
