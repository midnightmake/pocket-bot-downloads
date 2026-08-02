#include "programs/barcode.h"

#include "drivers/linesensor.h"
#include "drivers/motor.h"
#include "drivers/led.h"
#include "drivers/button.h"
#include "drivers/serial.h"
#include "drivers/clock.h"

#include "irmini/irmini.h"

#include "util/store.h"
#include "util/util.h"

#include <avr/eeprom.h>

#include <util/delay.h>

#include <stdlib.h>

// - - - defines - - - //

// max motor speeds
#define MAX_SPEED 20

// calibrated sensor value used to determine if we think we've detected a line
#define BIT_LOW 25
#define BIT_HIGH 65

// calibration timing values
#define NUM_CALIBRATION_STEPS 5

// line-following PID controller gains
#define P_GAIN 0.12f//0.25f
#define D_GAIN 0.05f//0.1f

// command reading settings
#define MAX_SUPPORTED_INSTRUCTIONS 32

// buffer size in bytes that we support for IR receiving
#define IR_BUFFER_LEN 8

// some interesting commands we can execute; we could potentially do something
// much lower-level like variable setting, loops, conditionals, etc.
#define COMMAND_NOP 0				// does nothing
#define COMMAND_RED_LEDS 10			// flash red LEDs
#define COMMAND_GREEN_LEDS 11		// flash green LEDs
#define COMMAND_BLUE_LEDS 12		// flash blue LEDs
#define COMMAND_WIGGLE 20			// rotate back and forth rapidly
#define COMMAND_SHAKE 21			// move back and forth rapidly
#define COMMAND_SPIN 42				// spin around a little
#define COMMAND_EOF 63				// end of instructions, so execute stored commands

// - - - types - - - //

// control state of barcode scanning
typedef enum SCAN_STATE
{
	SCAN_STATE_NORMAL = 0,			// scanning a line, nothing to read yet
	SCAN_STATE_BYTE_BEGIN,			// scanning a blank start-of-byte portion
	SCAN_STATE_BIT_BEGIN,			// scanning bit markers on the side
	SCAN_STATE_BIT_END,				// done scanning bit markers on the side, ready for next bit markers
	SCAN_STATE_COMPLETE				// scan done; complete byte is ready
} ScanState;

// - - - prototypes - - - //

// scan state control functions
static void updateScanStateNormal(uint32_t dt);
static void updateScanStateByteBegin(uint32_t dt);
static void updateScanStateBitBegin(uint32_t dt);
static void updateScanStateBitEnd(uint32_t dt);
static void updateScanStateComplete(uint32_t dt);

// runs an array of commands, either read from the course or received via IR; expected to end with COMMAND_EOF
static void runCommands(uint8_t *instructions);
static void runCommand(uint8_t instruction);

// command functions that are embedded in the course, transmitted over IR, or received over IR
static void commandRedLEDs();
static void commandGreenLEDs();
static void commandBlueLEDs();
static void flashLEDs(uint8_t red, uint8_t green, uint8_t blue);
static void commandWiggle();
static void commandShake();
static void commandSpin();

// lower-level functions
static void calibrate();
static void runCalibrationCycle(uint8_t steps, uint16_t *minReadings, uint16_t *maxReadings);
static void readCalibratedSensors(uint16_t *sensors);
static void followLine();

// - - - globals - - - //

// global state
ScanState scanState;
uint32_t stateTimer;
uint8_t done;

// scan results
uint32_t blankMarkerTime;
uint8_t numBitsScanned;
uint8_t scannedByte;
uint32_t bitReadTimes[8];
uint32_t lowestReadTime;
uint32_t highestReadTime;

// program instructions that we scan from the course or get via IR
uint8_t instructions[MAX_SUPPORTED_INSTRUCTIONS];
uint8_t numInstructions;

// sensor readings and calibrations are always available
uint16_t sensors[NUM_LINE_SENSORS];
uint16_t maxReadings[NUM_LINE_SENSORS];
uint16_t minReadings[NUM_LINE_SENSORS];

// - - - implementations - - - //

// This is a line-following program with additional logic implemented on top of it to control the reading of gaps and their spacing inside
// of a regular line course to read a 6-bit value encoded in it. Using the course-encoder program provided with the Pocket Bot, you can
// encode any 6-bit value into one page of a line course, print it out, and even chain multiples together. If a decoded number corresponds
// to a command (see the #define's above), it will be stored in an array that will be executed when the COMMAND_EOF is read. You can also
// implement your own commands, including loops or conditionals, if you want, to execute more complex behaviours.
void runBarcodeProgram()
{
	// low-level timing
	uint32_t currentTime;
	uint32_t oldTime;
	uint32_t dt;

	// bot behaviour
	uint8_t irBuffer[IR_BUFFER_LEN + 1];
	uint8_t isScanBot;

	// clear LEDs
	ledClear();
	ledRefresh();

	// stop motors
	motorLeft(0);
	motorRight(0);
	motorUseHardBrake(1);

	// initialize serial functionality; this is useful for debugging
	serialInit();
	serialSetBaudRate(19200);

	// we need IR for TXing and RXing
	irmInit(irBuffer, IR_BUFFER_LEN);

	// brief pause so user can remove their fingers
	_delay_ms(1000);
	calibrate();

	// wait for another button press; this is where the user will place the bot where it should be (on the line course, or off of it)
	utilWaitForButtonPress(2);
	_delay_ms(1000);

	// do a quick read; scanning bots are expected to be on a line course, while listening bots are not
	readCalibratedSensors(sensors);
	isScanBot = sensors[1] > BIT_HIGH;

	// prime nav if we need it
	if(isScanBot)
	{
		// the scanning bot needs navigation/scanning control
		scanState = SCAN_STATE_NORMAL;
		numInstructions = 0;
		scannedByte = 0;

		// read in line calibration values
		eeprom_read_block(minReadings, (const void*)STORE_MIN_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);
		eeprom_read_block(maxReadings, (const void*)STORE_MAX_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);
	}

	// run main loop
	oldTime = clockMillis();
	done = 0;
	while(!done)
	{
		// compute delta time in milliseconds
		currentTime = clockMillis();
		dt = (currentTime - oldTime);
		oldTime = currentTime;

		// scanning bots run the line course, read the instructions, and transmit commands
		if(isScanBot)
		{
			// function dispatch according to state
			if     (scanState == SCAN_STATE_NORMAL)			updateScanStateNormal(dt);
			else if(scanState == SCAN_STATE_BYTE_BEGIN)		updateScanStateByteBegin(dt);
			else if(scanState == SCAN_STATE_BIT_BEGIN)		updateScanStateBitBegin(dt);
			else if(scanState == SCAN_STATE_BIT_END)		updateScanStateBitEnd(dt);
			else if(scanState == SCAN_STATE_COMPLETE)		updateScanStateComplete(dt);

			// if we read a terminal instruction or have run out of space, transmit the instructions and run them locally
			if(scannedByte == COMMAND_EOF || numInstructions >= MAX_SUPPORTED_INSTRUCTIONS)
			{
				// stop moving
				motorLeft(0);
				motorRight(0);

				// transmit the instructions over IR
				irmSendBlocking(instructions, numInstructions);

				// run them ourselves and quit
				runCommands(instructions);
				done = 1;
			}
		}
		// and non-scanning bots just listen for the commands over IR
		else
		{
			// flash red LED to indicate we're waiting for commands
			ledSet(2, (clockMillis() % 2000 < 50) * 64, 0, 0);
			ledRefresh();

			// see if we have received a set of commands
			if(irmHasRX())
			{
				// execute them, but only process as many as we can support over IR; if there's an EOF before this then that's okay too
				irBuffer[irmGetRXLength() - 1] = COMMAND_EOF;

				// execute the commands up until any EOF in the buffer
				runCommands(irBuffer);

				// ready for next commands if we want to do another run
				irmClearRX();

				// quit
				done = 1;
			}
			// clear IR errors if we get any, but keep this conditional so we have the option of debugging if we want
			else if(irmGetError() != IRM_ERR_NONE)
			{
				// ignore IR errors
				irmClearError();
			}
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

static void updateScanStateNormal(uint32_t dt)
{
	// always follow the yellow brick road
	readCalibratedSensors(sensors);
	followLine();

	// if we encounter a blank spot, then we've hit the beginning of a byte
	if(sensors[1] < BIT_LOW)
	{
		// start a timer; we need to time how long this calibration blank spot lasts
		stateTimer = clockMillis();

		// ready to start reading a byte
		scanState = SCAN_STATE_BYTE_BEGIN;
		numBitsScanned = 0;
	}
}

static void updateScanStateByteBegin(uint32_t dt)
{
	// always follow the yellow brick road
	readCalibratedSensors(sensors);
	followLine();
	
	// has the middle sensor hit the line?
	if(sensors[1] > BIT_HIGH)
	{
		// if we're on our first bit, record the time the calibration blank took
		blankMarkerTime = clockMillis() - stateTimer;

		// reset bounds on bit times we've seen; use the calibration marker as a low starting point
		lowestReadTime = blankMarkerTime;
		highestReadTime = blankMarkerTime;

		// now reset the timer so we can see how long this line segment lasts
		stateTimer = clockMillis();

		// begin reading the bit
		scanState = SCAN_STATE_BIT_BEGIN;
	}
}

static void updateScanStateBitBegin(uint32_t dt)
{
	uint32_t bitTime;

	// always follow the yellow brick road
	readCalibratedSensors(sensors);
	followLine();

	// if we encounter the end of a bit marker, we're done
	if(sensors[1] < BIT_LOW)
	{
		// record the time that we travelled over this bit
		bitTime = clockMillis() - stateTimer;

		// briefly indicate with an LED that we've read something
		ledClear(); ledSet(2, 20, 20, 20); ledRefresh();
		_delay_ms(10);
		ledClear(); ledRefresh();

		// store bit scan time
		lowestReadTime = min(lowestReadTime, bitTime);
		highestReadTime = max(highestReadTime, bitTime);
		bitReadTimes[numBitsScanned] = bitTime;

		// next bit
		numBitsScanned ++;
	
		// are we done?
		if(numBitsScanned >= 8)
		{
			// yes; stop
			scanState = SCAN_STATE_COMPLETE;
		}
		else
		{
			// no; prepare to read another bit
			scanState = SCAN_STATE_BIT_END;
		}
	}
}

static void updateScanStateBitEnd(uint32_t dt)
{
	// always follow the yellow brick road
	readCalibratedSensors(sensors);
	followLine();
	
	// has the middle sensor hit the line?
	if(sensors[1] > BIT_HIGH)
	{
		// now reset the timer so we can see how long this line segment lasts
		stateTimer = clockMillis();

		// begin reading the bit
		scanState = SCAN_STATE_BIT_BEGIN;
	}
}

static void updateScanStateComplete(uint32_t dt)
{
	uint32_t halfway;
	uint8_t i;

	// always follow the yellow brick road
	readCalibratedSensors(sensors);
	followLine();

	// compute halfway point between our longest and shortest bit times
	halfway = lowestReadTime + (highestReadTime - lowestReadTime) / 2;

	// based on where the bit times sit relative to this halfway point, build our scanned value
	scannedByte = 0;
	for(i = 0; i < 8; ++ i)
	{
		// accumulate bits until we get a complete byte; we do this using a parameterless approach
		// in which bits whose length measures greater than the halfway point between the shortest
		// and longest read bit is a 1, and a 0 otherwise
		scannedByte <<= 1;
		if(bitReadTimes[7 - i] > halfway)
		{
			scannedByte |= 0x01;
		}
	}

	// shift right by two bits to exclude our calibration bits
	scannedByte >>= 2;

	// store the instruction
	instructions[numInstructions++] = scannedByte;

	// reset our scanning state machine to potentially prepare to read another byte
	scanState = SCAN_STATE_NORMAL;
}

static void runCommands(uint8_t *instructions)
{
	uint8_t *ptr;

	// start at the first instruction and run until we reach the terminal command
	ptr = instructions;
	while(*ptr != COMMAND_EOF)
	{
		runCommand(*ptr++);
	}

	// shut down LEDs and motors just to be safe
	ledClear();
	ledRefresh();
	motorLeft(0);
	motorRight(0);
}

static void runCommand(uint8_t instruction)
{
	// respond to whatever command we get
	if     (instruction == COMMAND_RED_LEDS)	commandRedLEDs();
	else if(instruction == COMMAND_GREEN_LEDS)	commandGreenLEDs();
	else if(instruction == COMMAND_BLUE_LEDS)	commandBlueLEDs();
	else if(instruction == COMMAND_WIGGLE)		commandWiggle();
	else if(instruction == COMMAND_SHAKE)		commandShake();
	else if(instruction == COMMAND_SPIN)		commandSpin();
}

static void commandRedLEDs()
{
	flashLEDs(50, 0, 0);
}

static void commandGreenLEDs()
{
	flashLEDs(0, 50, 0);
}

static void commandBlueLEDs()
{
	flashLEDs(0, 0, 50);
}

static void flashLEDs(uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t i, j;

	// flash multiple times
	for(i = 0; i < 5; i ++)
	{
		// turn on briefly
		for(j = 0; j < NUM_LEDS; ++ j)
		{
			ledSet(j, red, green, blue);
		}
		ledRefresh();
		_delay_ms(30);

		// turn off briefly
		ledClear();
		ledRefresh();
		_delay_ms(120);
	}
}

static void commandWiggle()
{
	uint8_t i;

	// rotate back and forth
	for(i = 0; i < 10; ++ i)
	{
		// turn left and pause
		motorLeft(-40);
		motorRight(40);
		_delay_ms(100);

		// turn right and pause
		motorLeft(40);
		motorRight(-40);
		_delay_ms(100);
	}

	// stop all motors
	motorLeft(0);
	motorRight(0);
}

static void commandShake()
{
	uint8_t i;

	// move back and forth
	for(i = 0; i < 10; ++ i)
	{
		// move forward and pause
		motorLeft(40);
		motorRight(40);
		_delay_ms(100);

		// move backward and pause
		motorLeft(-40);
		motorRight(-40);
		_delay_ms(100);
	}

	// stop all motors
	motorLeft(0);
	motorRight(0);
}

static void commandSpin()
{
	uint8_t hue;
	uint8_t r, g, b;
	uint8_t cycles;
	uint8_t i;

	// spin around
	motorLeft(-20);
	motorRight(20);

	// this is roughly a 360 spin
	hue = 0;
	cycles = 0;
	while(cycles < 211)
	{
		// scroll through hue
		hue += 3;
		cycles ++;

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
	}

	// stop motors
	motorLeft(0);
	motorRight(0);

	// shut down LEDs
	ledClear();
	ledRefresh();
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

	// stop moving and turn off LEDs
	motorLeft(0);
	motorRight(0);
	ledClear();
	ledRefresh();

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

static void followLine()
{
	// line position tracking
	uint64_t lineAvg = 0;
	uint32_t lineSum = 0;
	uint32_t linePos = 0;
	uint8_t lineOn = 0;

	// driving controls
	int32_t err;
	int32_t motorPowerDiff;
	int32_t left, right;
	int32_t baseSpeed;
	uint8_t i;

	// line following PID state must persist across calls
	static uint32_t lineFollowLastPos = 0;
	static int32_t lineFollowDerivative = 0;
	static int32_t lineFollowLastError = 0;
	
	// take sensor readings
	for(i = 0; i < NUM_LINE_SENSORS; ++ i)
	{
		// accumulate position average
		lineAvg += (uint64_t)(sensors[i]) * (i * 100);
		lineSum += sensors[i];

		// are we on the line at all?
		if(sensors[i] > BIT_LOW)
		{
			lineOn = 1;
		}
	}

	// apply averaging if we have a position on the line
	if(lineSum > 0 && lineOn)
	{
		// linePos ranges from 0 to 100, where 0 is on the left, 100 is centered, and 200 is on the right
		linePos = lineAvg / lineSum;
		lineFollowLastPos = linePos;
	}
	else
	{
		// use last known line position
		linePos = lineFollowLastPos;
	}

	// PID update
	err = (int32_t)linePos - 100;							// compute error term; 100 represents a line at the center of the bot
	lineFollowDerivative = err - lineFollowLastError;		// compute derivative term
	lineFollowLastError = err;								// update last error term

	// update the PD controller
	motorPowerDiff = (int)(((float)err						* P_GAIN) + \
						   ((float)lineFollowDerivative		* D_GAIN));

	// our base speed is a function of our corrective power
	baseSpeed = MAX_SPEED - abs(motorPowerDiff);

	// assign left and right motor speeds; clamp first to avoid overload when calling motor speed functions
	left = clamp(baseSpeed + motorPowerDiff, -MAX_SPEED, MAX_SPEED);
	right = clamp(baseSpeed - motorPowerDiff, -MAX_SPEED, MAX_SPEED);
	motorLeft(left);
	motorRight(right);
}
