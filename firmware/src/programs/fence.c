#include "programs/fence.h"

#include "drivers/linesensor.h"
#include "drivers/motor.h"
#include "drivers/led.h"
#include "drivers/button.h"

#include "util/util.h"
#include "util/store.h"
#include "util/xabc.h"

#include <avr/eeprom.h>

#include <util/delay.h>

#include <string.h>
#include <stdio.h>

// - - - defines - - - //

// max power used by the motors when running fence program
#define BASE_SPEED 30

// reflection value above which we associate a black line or a platform edge
#define EDGE_THRESHOLD 75

// calibration timing values
#define NUM_CALIBRATION_STEPS 5

// - - - prototypes - - - //

// calibration functions
static void calibrate();
static void readCalibratedSensors(uint16_t *sensors);
static void runCalibrationCycle(uint8_t steps, uint16_t *minReadings, uint16_t *maxReadings);

// - - - globals - - - //

// sensor readings and calibrations are always available
uint16_t sensors[NUM_LINE_SENSORS];
uint16_t maxReadings[NUM_LINE_SENSORS];
uint16_t minReadings[NUM_LINE_SENSORS];

// - - - implementations - - - //

// runs the line following program until the user presses the button
void runFenceProgram()
{
	uint8_t edgeDetected;

	uint8_t i;
	uint8_t done = 0;

	// clear LEDs
	ledClear();
	ledRefresh();

	// stop motors
	motorLeft(0);
	motorRight(0);
	motorUseHardBrake(1);

	// brief pause so user can remove their fingers, and then perform auto-calibration routine: the robot rotates while reading the line sensors
	_delay_ms(1000);
	calibrate();

	// read in line calibration values
	eeprom_read_block(minReadings, (const void*)STORE_MIN_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);
	eeprom_read_block(maxReadings, (const void*)STORE_MAX_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);

	// run main loop
	while(!done)
	{
		// move forward
		motorLeft(100);
		motorRight(100);
		
		// take line sensor readings
		readCalibratedSensors(sensors);
		
		// if any readings got too high, then we stepped onto a line or over the edge of something
		edgeDetected = 0;
		i = 0;
		while(i < NUM_LINE_SENSORS && !edgeDetected)
		{
			if(sensors[i++] > EDGE_THRESHOLD)
			{
				edgeDetected = 1;
			}
		}

		// if we detected an edge, do a back-up-and-rotate sequence
		if(edgeDetected)
		{
			// turn on a red LED
			ledSet(2, 64, 0, 0);
			ledRefresh();

			// back up for a bit
			motorLeft(-BASE_SPEED);
			motorRight(-BASE_SPEED);
			_delay_ms(400);

			// rotate to one side; note that if we're using a platform then a fixed direction and rotation time works much better
			motorLeft(BASE_SPEED);
			motorRight(-BASE_SPEED);
			_delay_ms(300);
			
			// reset
			edgeDetected = 0;

			// turn off LEDs
			ledClear();
			ledRefresh();
		}

		// shut down if button is pressed
		if(buttonDown())
		{
			_delay_ms(50);
			while(buttonDown());
			done = 1;
		}

		// show low-power warnings or shut down if needed
		utilHandleLowPower();
	}
}

static void calibrate()
{
	uint8_t i;

	// reset readings values
	for(i = 0; i < NUM_LINE_SENSORS; ++ i)
	{
		maxReadings[i] = 0;
		minReadings[i] = UINT16_MAX;
	}

	// rotate right
	motorLeft(20);
	motorRight(-20);
	runCalibrationCycle(NUM_CALIBRATION_STEPS, minReadings, maxReadings);

	// rotate left
	motorLeft(0);
	motorRight(0);
	_delay_ms(50);
	motorLeft(-20);
	motorRight(20);
	runCalibrationCycle(NUM_CALIBRATION_STEPS * 2, minReadings, maxReadings);

	// rotate right again to recenter
	motorLeft(0);
	motorRight(0);
	_delay_ms(50);
	motorLeft(20);
	motorRight(-20);
	runCalibrationCycle(NUM_CALIBRATION_STEPS, minReadings, maxReadings);

	// store the values in EEPROM
	eeprom_write_block(minReadings, (void*)STORE_MIN_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);
	eeprom_write_block(maxReadings, (void*)STORE_MAX_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);

	// turn off LEDs and motors
	motorLeft(0);
	motorRight(0);
	ledClear();
	ledRefresh();
}

static void runCalibrationCycle(uint8_t steps, uint16_t *minReadings, uint16_t *maxReadings)
{
	uint16_t sensors[NUM_LINE_SENSORS];
	uint8_t i, j, k;

	// loop that takes a bunch of readings
	for(i = 0; i < NUM_LEDS; ++ i)
	{
		// set LEDs to indicate count down/time remaining
		for(j = 0; j < NUM_LEDS; ++ j)
		{
			ledSet(j, j >= i ? 10 : 0, j >= i ? 10 : 0, 0);
		}
		ledRefresh();

		// take a bunch of readings
		for(j = 0; j < steps; ++ j)
		{
			lineSensorRead(sensors);
			for(k = 0; k < NUM_LINE_SENSORS; ++ k)
			{
				maxReadings[k] = max(sensors[k], maxReadings[k]);
				minReadings[k] = min(sensors[k], minReadings[k]);
			}
		}
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
