#include "programs/linecalibration.h"

#include "drivers/led.h"
#include "drivers/linesensor.h"
#include "drivers/button.h"

#include "util/store.h"
#include "util/util.h"

#include <avr/eeprom.h>

#include <stdint.h>

// - - - implementations - - - //

void runLineCalibrationProgram()
{
	uint16_t minReadings[NUM_LINE_SENSORS];
	uint16_t maxReadings[NUM_LINE_SENSORS];
	uint16_t sensors[NUM_LINE_SENSORS];

	uint32_t i, j, k;

	// take some bogus readings
	for(i = 0; i < 10; ++ i)
	{
		lineSensorRead(sensors);
	}

	// reset readings values
	for(i = 0; i < NUM_LINE_SENSORS; ++ i)
	{
		maxReadings[i] = 0;
		minReadings[i] = UINT16_MAX;
	}

	// loop that takes a bunch of readings
	for(i = 0; i < NUM_LEDS; ++ i)
	{
		// set LEDs to indicate count down/time remaining
		for(j = 0; j < NUM_LEDS; ++ j)
		{
			ledSet(j, j >= i ? 20 : 0, 0, 0);
		}
		ledRefresh();

		// take a bunch of readings
		for(j = 0; j < 50; ++ j)
		{
			lineSensorRead(sensors);
			for(k = 0; k < NUM_LINE_SENSORS; ++ k)
			{
				maxReadings[k] = max(sensors[k], maxReadings[k]);
				minReadings[k] = min(sensors[k], minReadings[k]);
			}
		}
	}

	// store the values in EEPROM
	eeprom_write_block(minReadings, (void*)STORE_MIN_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);
	eeprom_write_block(maxReadings, (void*)STORE_MAX_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);
}
