#include "programs/linedebug.h"

#include "drivers/linesensor.h"
#include "drivers/motor.h"
#include "drivers/led.h"
#include "drivers/button.h"
#include "drivers/serial.h"

#include "util/store.h"
#include "util/util.h"

#include <avr/eeprom.h>

#include <util/delay.h>

#include <stdlib.h>

// - - - defines - - - //

// [anything?]

// - - - prototypes - - - //

static void readCalibratedSensors(uint16_t *sensors);

// - - - globals - - - //

// sensor readings and calibrations are always available
uint16_t sensors[NUM_LINE_SENSORS];
uint16_t maxReadings[NUM_LINE_SENSORS];
uint16_t minReadings[NUM_LINE_SENSORS];

// - - - implementations - - - //

// runs the line following program until the user presses the button
void runLineDebugProgram()
{
	uint8_t done;

	// clear LEDs
	ledClear();
	ledRefresh();

	// stop motors
	motorLeft(0);
	motorRight(0);
	motorUseHardBrake(1);

	// read in line calibration values
	eeprom_read_block(minReadings, (const void*)STORE_MIN_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);
	eeprom_read_block(maxReadings, (const void*)STORE_MAX_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);

	// run main loop
	done = 0;
	while(!done)
	{
		// take a reading from the line following sensors
		readCalibratedSensors(sensors);

		// set the LEDs to indicate read line strength; note that we don't use the full brightness of the LEDs here
		// since calibrated sensor values fall in the 0 - 100 range
		ledClear();
		ledSet(0, 0, sensors[0], 0);
		ledSet(2, 0, sensors[1], 0);
		ledSet(4, 0, sensors[2], 0);
		ledRefresh();

		// shut down if button is pressed
		if(buttonDown())
		{
			_delay_ms(50);
			while(buttonDown());
			done = 1;
		}

		// also spit out serial output showing calibrated sensor values
		serialTX("%4d  %4d  %4d\r\n", (int)sensors[0], (int)sensors[1], (int)sensors[2]);

		// show low-power warnings or shut down if needed
		utilHandleLowPower();
	}
}

static void readCalibratedSensors(uint16_t *sensors)
{
	uint8_t i;

	// take a reading from the line following sensors
	lineSensorRead(sensors);

	// scale into usable range; 0 to 100
	for(i = 0; i < NUM_LINE_SENSORS; ++ i)
	{
		sensors[i] = clamp(sensors[i], minReadings[i], maxReadings[i]);
		sensors[i] = ((sensors[i] - minReadings[i]) * 100) / (maxReadings[i] - minReadings[i] + 1);
	}
}
