#include "util/util.h"

#include "drivers/clock.h"
#include "drivers/battery.h"
#include "drivers/button.h"
#include "drivers/led.h"
#include "drivers/motor.h"

#include <stdint.h>
#include <stdlib.h>

#include <avr/sleep.h>

#include <util/delay.h>

// - - - defines - - - //

// battery millivolt value below which we shut down the robot
#define SHUTDOWN_MV 3350

// - - - prototypes - - - //

static uint8_t colorDistFunc(uint8_t hue, uint8_t center);

// - - - implementations - - - //

// flash a "ready" LED and block until user presses and releases the button
void utilWaitForButtonPress(uint8_t index)
{
	uint8_t done = 0;

	// loop until user presses and releases button
	while(!done)
	{
		// flash green "ready" LED
		ledSet(index, 0, (clockMillis() % 500 < 50) * 64, 0);
		ledRefresh();

		// wait for button press
		if(buttonDown())
		{
			// debounce and wait for button release
			_delay_ms(50);
			while(buttonDown());

			// exit loop
			done = 1;
		}
	}

	// clear LEDs
	ledClear();
	ledRefresh();
}

// show a "warning" LED if the battery voltage is getting low, and shut down the bot if it goes below mininum safe levels
void utilHandleLowPower()
{
	uint16_t mv;
	uint8_t i, j;

	// perform a read
	batteryRead();
	mv = batteryGetMillivolts();
	
	// if we fall below safe levels, shut down the robot as best we can
	if(mv < SHUTDOWN_MV)
	{
		// turn off all motors
		motorLeft(0);
		motorRight(0);

		// brief pause with LEDs off; this prevents us seeing a brief red LED flash during normal power-off
		ledClear();
		ledRefresh();
		_delay_ms(500);

		// show LEDs all flash red
		for(i = 0; i < 20; i ++)
		{
			// turn on briefly
			for(j = 0; j < NUM_LEDS; ++ j)
			{
				ledSet(j, 64, 0, 0);
			}
			ledRefresh();
			_delay_ms(20);

			// turn off briefly
			ledClear();
			ledRefresh();
			_delay_ms(100);
		}

		// and then shut down; in this state, we will use <= 1mA of power
		ADCSRA &= ~(1 << ADEN);				// shut down ADC
		SMCR |= (1 << SM1);					// set power-down bit
		SMCR |= (1 << SE);					// enable sleep bit
		sleep_cpu();

		// [ you must power cycle the bot to get out of this state ]
	}
}

void utilHue2RGB(uint8_t hue, uint8_t *red, uint8_t *green, uint8_t *blue)
{
	*red = max(colorDistFunc(hue, 0), colorDistFunc(hue, 255));
	*green = colorDistFunc(hue, 85);
	*blue = colorDistFunc(hue, 170);
}

// - - - static methods - - - //

static uint8_t colorDistFunc(uint8_t hue, uint8_t center)
{
	return clamp(255L - (abs(center - hue) * 3), 0, 255);
}
