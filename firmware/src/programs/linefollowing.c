#include "programs/linefollowing.h"

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

// max motor speed
#define MAX_SPEED 100

// calibrated sensor value used to determine if we think we've detected a line
#define MIN_LINE_THRESHOLD 45

// calibration timing values
#define NUM_CALIBRATION_STEPS 5

// PID controller gains
#define P_GAIN 0.8f
#define D_GAIN 1.1f

// - - - prototypes - - - //

static void calibrate();
static void readCalibratedSensors(uint16_t *sensors);
static void runCalibrationCycle(uint8_t steps, uint16_t *minReadings, uint16_t *maxReadings);
static void showDirection(int8_t dir);

// - - - globals - - - //

// sensor readings and calibrations are always available
uint16_t sensors[NUM_LINE_SENSORS];
uint16_t maxReadings[NUM_LINE_SENSORS];
uint16_t minReadings[NUM_LINE_SENSORS];

// - - - implementations - - - //

// runs the line following program until the user presses the button
void runLineFollowingProgram()
{
	uint64_t lineAvg;
	uint32_t lineSum;
	uint32_t linePos;
	uint32_t lastLinePos;
	uint8_t lineOn;

	int32_t error;
	int32_t derivative;
	int32_t lastError = 0;
	int32_t motorPowerDiff;
	int32_t left, right;
	int32_t baseSpeed;

	uint8_t i;
	uint8_t done = 0;

	// clear LEDs
	ledClear();
	ledRefresh();

	// stop motors
	motorLeft(0);
	motorRight(0);
	motorUseHardBrake(1);

	// initialize serial functionality; this is useful for debugging
	serialInit();
	serialSetBaudRate(9600);

	// brief pause so user can remove their fingers, and then perform auto-calibration routine: the robot rotates while reading the line sensors
	_delay_ms(1000);
	calibrate();

	// read in line calibration values
	eeprom_read_block(minReadings, (const void*)STORE_MIN_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);
	eeprom_read_block(maxReadings, (const void*)STORE_MAX_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);

	// run main loop
	lastLinePos = 100;
	while(!done)
	{
		// take a reading from the line following sensors
		readCalibratedSensors(sensors);

		// reset line-tracking values
		lineAvg = 0;
		lineSum = 0;
		linePos = 0;

		// take sensor readings
		lineOn = 0;
		for(i = 0; i < NUM_LINE_SENSORS; ++ i)
		{
			// accumulate position average
			lineAvg += (uint64_t)(sensors[i]) * (i * 100);
			lineSum += sensors[i];

			// are we on the line at all?
			if(sensors[i] > MIN_LINE_THRESHOLD)
			{
				lineOn = 1;
			}
		}

		// apply averaging if we have a position on the line
		if(lineSum > 0 && lineOn)
		{
			// linePos ranges from 0 to 100, where 0 is on the left, 100 is centered, and 200 is on the right
			linePos = lineAvg / lineSum;
			lastLinePos = linePos;
		}
		else
		{
			linePos = lastLinePos;
		}

		// compute error term; 100 represents a line at the center of the bot
		error = (int32_t)linePos - 100;

		// compute derivative term
		derivative = error - lastError;

		// update last error term
		lastError = error;

		// update the PD controller
		motorPowerDiff = (int)(((float)error        * P_GAIN) + \
							   ((float)derivative   * D_GAIN));

		// our base speed is a function of our corrective power
		baseSpeed = MAX_SPEED - abs(motorPowerDiff);

		// assign left and right motor speeds; clamp first to avoid overload when calling motor speed functions
		left = clamp(baseSpeed + motorPowerDiff, -MAX_SPEED, MAX_SPEED);
		right = clamp(baseSpeed - motorPowerDiff, -MAX_SPEED, MAX_SPEED);
		motorLeft(left);
		motorRight(right);

		// shut down if button is pressed
		if(buttonDown())
		{
			_delay_ms(50);
			while(buttonDown());
			done = 1;
		}

		// show what direction we're turning in
		showDirection(error);

		// show low-power warnings or shut down if needed
		utilHandleLowPower();

		// debugging output useful for testing
		//serialTX("%4d  %4d  %4d\r\n", (int)sensors[0], (int)sensors[1], (int)sensors[2]);
		//serialTX("on: %1d  |  pos: %4d\r\n", (int)lineOn, (int)linePos);
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

static void showDirection(int8_t dir)
{
	// clamp to useful range
	dir = clamp(dir, -100, 100);

	if(dir < -50)
	{
		ledSet(0, 0, abs(dir) - 50, 0);
		ledSet(1, 0, 50, 0);
		ledSet(2, 0, 50, 0);
		ledSet(3, 0, 0, 0);
		ledSet(4, 0, 0, 0);
	}
	else if(dir < 0)
	{
		ledSet(0, 0, 0, 0);
		ledSet(1, 0, abs(dir), 0);
		ledSet(2, 0, 50, 0);
		ledSet(3, 0, 0, 0);
		ledSet(4, 0, 0, 0);
	}
	else if(dir <= 50)
	{
		ledSet(0, 0, 0, 0);
		ledSet(1, 0, 0, 0);
		ledSet(2, 0, 50, 0);
		ledSet(3, 0, abs(dir), 0);
		ledSet(4, 0, 0, 0);
	}
	else if(dir <= 100)
	{
		ledSet(0, 0, 0, 0);
		ledSet(1, 0, 0, 0);
		ledSet(2, 0, 50, 0);
		ledSet(3, 0, 50, 0);
		ledSet(4, 0, abs(dir) - 50, 0);
	}

	// apply changes
	ledRefresh();
}
