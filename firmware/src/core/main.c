#include "drivers/clock.h"
#include "drivers/adc.h"
#include "drivers/motor.h"
#include "drivers/linesensor.h"
#include "drivers/led.h"
#include "drivers/button.h"

#include "programs/linefollowing.h"
#include "programs/fence.h"
#include "programs/barcode.h"
#include "programs/rave.h"
#include "programs/linedebug.h"
#include "programs/irforwarding.h"
#include "programs/visionclient.h"

#include "irmini/irmini.h"

#include "util/store.h"
#include "util/xabc.h"
#include "util/util.h"

#include <avr/io.h>
#include <avr/eeprom.h>

#include <util/delay.h>

#include <stdlib.h>

// - - - defines - - - //

// number of programs we can choose from; conveniently, this is equal to the number of LEDs we have available
#define NUM_PROGRAMS NUM_LEDS

// determines what length of time in milliseconds is considered a button tap vs. a hold
#define SHORT_PRESS_MS 1000

// - - - implementations - - - //

// entry point
int main(void)
{
	// program selection control
	uint8_t program;
	uint32_t buttonDownTime;
	uint8_t longPress;
	uint32_t factor;
	uint8_t red, green, blue;
	uint8_t i, j;

	// initialize core libraries
	clockInit();		// clock and timing
	xabcInit();			// pseudo-random number generator
	adcInit();			// analog-to-digital converter

	// initialize our hardware drivers
	motorInit();
	lineSensorInit();
	ledInit();
	buttonInit();

	// initialize starting LED sequence indicating setup was successful
	for(i = 0; i < 255; ++ i)
	{
		// all LEDs cycle through colours
		for(j = 0; j < NUM_LEDS; ++ j)
		{
			// compute hue for this LED at this part of the animation
			utilHue2RGB(((i + 190) * 2) + (j * 40), &red, &green, &blue);

			// compute fade in and then a fade out
			if(i < 128)
			{
				factor = (((uint32_t)i * 1000) / 128);
			}
			else
			{
				factor = (((255 - (uint32_t)i) * 1000) / 128);
			}

			// compute colour components
			red = (((uint32_t)red * factor) / 2000);
			green = (((uint32_t)green * factor) / 2000);
			blue = (((uint32_t)blue  * factor) / 2000);
			ledSet(j, red, green, blue);
			ledRefresh();
		}
	}

	// [OPTIONAL]: bypass user selection mode and start a program automatically here
	//runVisionClientProgram();

	// [OPTIONAL]: assign a robot ID on start-up if needed
	//eeprom_write_byte((uint8_t*)STORE_ID_ADDR, 17);

	// fade in first blue LED since this will always be highlighted by default
	ledClear();
	for(i = 0; i < 64; i += 2)
	{
		ledSet(0, 0, 0, i);
		ledRefresh();
		_delay_ms(10);
	}

	// set program 0 as the default
	program = 0;
	ledClear();
	ledSet(program, 0, 0, 64);
	ledRefresh();

	// [OPTIONAL]: adjust motor speeds slightly if they're imbalanced; this is very common even for expensive motors
	//motorScaleLeft(0.85f);
	//motorScaleRight(1.0f);

	// main loop; we should never exit out of this
	while(1)
	{
		// was the button pressed?
		if(buttonDown())
		{
			// start a timer
			buttonDownTime = clockMillis();
			longPress = 0;

			// debounce and wait for button to come up again
			_delay_ms(50);
			while(buttonDown())
			{
				// is it a long hold?
				if(clockMillis() - buttonDownTime > SHORT_PRESS_MS)
				{
					// show all LEDs, indicating long press is active
					for(i = 0; i < NUM_LEDS; ++ i)
					{
						ledSet(i, 0, 0, 64);
					}
					ledRefresh();

					// register as a long hold on the button
					longPress = 1;
				}
			}

			// turn off leds if they need to be
			ledClear();
			ledRefresh();

			// what kind of press was it?
			if(longPress)
			{
				// wait for user to start
				utilWaitForButtonPress(program);

				// clear LED status in preparation for upcoming program
				ledClear();
				ledRefresh();

				// start the program; we block here until the program is complete
				if     (program == 0) runLineFollowingProgram();
				else if(program == 1) runFenceProgram();
				else if(program == 2) runBarcodeProgram();
				else if(program == 3) runRaveProgram();
				else if(program == 4) runLineDebugProgram();

				// [ at this point, whatever program was running is done ]

				// stop motors
				motorLeft(0);
				motorRight(0);
			}
			else
			{
				// next program
				program = (program + 1) % NUM_PROGRAMS;
			}

			// indicate program with LEDs
			ledClear();
			ledSet(program, 0, 0, 64);
			ledRefresh();
		}

		// handle low battery indicator
		utilHandleLowPower();
	}

	// good practice to prevent execution of random code by accident
	while(1);
}
