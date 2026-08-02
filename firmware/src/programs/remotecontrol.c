#include "programs/remotecontrol.h"

#include "drivers/clock.h"
#include "drivers/led.h"
#include "drivers/motor.h"
#include "drivers/button.h"

#include "util/util.h"

#include "irmini/irmini.h"

#include <util/delay.h>

// - - - defines - - - //

// buffer size in bytes of the commands expected by the IR remote control device
#define IR_BUFFER_LEN 3

// code indicating a motion command
#define IR_COMMAND_MOVE 0x01

// - - - implementations - - - //

void runRemoteControlProgram()
{
	// IR control
	uint8_t irBuffer[IR_BUFFER_LEN + 1];
	int8_t left;
	int8_t right;
	uint8_t done;

	// initialize the IRM library
	irmInit(irBuffer, IR_BUFFER_LEN);

	// smoother and easier on the robot
	motorUseHardBrake(0);

	// loop until user is ready to exit
	done = 0;
	while(!done)
	{
		// update IR functions; did we get an IR command?
		if(irmHasRX())
		{
			// move command
			if(irBuffer[0] == IR_COMMAND_MOVE)
			{
				// read speeds
				left = (int8_t)(int16_t)irBuffer[1] - 100;
				right = (int8_t)(int16_t)irBuffer[2] - 100;

				// ready for next message
				irmClearRX();

				// start moving
				motorLeft(left);
				motorRight(right);

				// if we're moving forward
				if(left > 0 || right > 0)
				{
					// hide reverse/brake lights
					ledSet(0, 0, 0, 0);
					ledSet(4, 0, 0, 0);
				}
				else
				{
					// otherwise, show reverse lights
					ledSet(0, 16, 16, 16);
					ledSet(4, 16, 16, 16);
				}

				// update LEDs
				ledRefresh();

				// run motors briefly and then shut them down
				_delay_ms(180);
			}
		}
		else
		{
			// shut down motors
			motorLeft(0);
			motorRight(0);

			// show brake lights LED indicating stop
			ledSet(0, 64, 0, 0);
			ledSet(4, 64, 0, 0);
			ledRefresh();
		}

		// shut down if button is pressed
		if(buttonDown())
		{
			_delay_ms(50);
			while(buttonDown());
			done = 1;
		}

		// shut down if power gets too low
		utilHandleLowPower();
	}

	// shut down IR library
	irmShutdown();
}
