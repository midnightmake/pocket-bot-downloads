#include "programs/gridnav.h"

#include "drivers/linesensor.h"
#include "drivers/motor.h"
#include "drivers/led.h"
#include "drivers/button.h"
#include "drivers/serial.h"
#include "drivers/clock.h"

#include "util/store.h"
#include "util/util.h"

#include <avr/eeprom.h>

#include <util/delay.h>

#include <stdlib.h>

// - - - defines - - - //

// max motor speed
#define TURN_SPEED 15
#define MAX_SPEED 20

// calibrated sensor value used to determine if we think we've detected a line
#define MIN_LINE_THRESHOLD 25
#define INTERSECTION_THRESHOLD 75

// calibration timing values
#define NUM_CALIBRATION_STEPS 5

// PID controller gains
#define P_GAIN 0.25f//0.4f//0.25f//0.25f//0.4f//0.35f//0.8f
#define D_GAIN 0.1f//0.55f//0.1f//0.1f//275f//0.55f//0.8f//1.1f

#define PATH_LEFT (1 << 0)
#define PATH_STRAIGHT (1 << 1)
#define PATH_RIGHT (1 << 2)

// - - - types - - - //

// controls high-level robot navigation

// controls low-level robot motion against the grid
typedef enum NAV_STATE
{
	NAV_STATE_ASSESS = 0,
	NAV_STATE_FORWARD_START,
	NAV_STATE_FORWARD_END,
	NAV_STATE_ROTATING
} NavState;

typedef enum DIR
{
	DIR_NORTH = 0,
	DIR_EAST,
	DIR_SOUTH,
	DIR_WEST
} Dir;

// - - - prototypes - - - //

static void updateNavStateAssess(uint32_t dt);
static void updateNavStateForwardStart(uint32_t dt);
static void updateNavStateForwardEnd(uint32_t dt);
static void updateNavStateRotating(uint32_t dt);

//static void calibrate();
static void readCalibratedSensors(uint16_t *sensors);
static void followLine();
static void setMotorRotation(Dir targetDir);
//static void showDirection(int8_t dir);

// - - - globals - - - //

// global state
NavState navState;
uint32_t stateTimer;
uint8_t navDone;
uint8_t navIsCenteredOnIntersection;

// robot pose
uint8_t posX;
uint8_t posY;
Dir heading;		// orientation in global frame
Dir travelDir;		// target direction of travel in global frame
uint8_t rotatingLeft;

// destination coordinates
uint8_t targetX;
uint8_t targetY;

// target corner that gets further and further away
uint8_t cornerX;
uint8_t cornerY;

// sensor readings and calibrations are always available
uint16_t sensors[NUM_LINE_SENSORS];
uint16_t maxReadings[NUM_LINE_SENSORS];
uint16_t minReadings[NUM_LINE_SENSORS];

// - - - implementations - - - //

// runs the line following program until the user presses the button
void runGridNavProgram()
{
	// low-level timing
	uint32_t currentTime;
	uint32_t oldTime;
	uint32_t dt;

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

	// brief pause so user can remove their fingers, and then perform auto-calibration routine: the robot rotates while reading the line sensors
	_delay_ms(500);
	//calibrate();

	// read in line calibration values
	eeprom_read_block(minReadings, (const void*)STORE_MIN_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);
	eeprom_read_block(maxReadings, (const void*)STORE_MAX_READING_SENSOR_1_ADDR, sizeof(uint16_t) * NUM_LINE_SENSORS);

	// prime nav
	oldTime = clockMillis();
	navState = NAV_STATE_ASSESS;
	navIsCenteredOnIntersection = 1;
	stateTimer = 0;

	// set robot pose
	posX = 0;
	posY = 0;
	heading = DIR_NORTH;

	// set initial corner pos
	cornerX = 1;
	cornerY = 1;

	// set target position
	targetX = cornerX;
	targetY = cornerY;

	// run main loop
	navDone = 0;
	while(!navDone)
	{
		// compute delta time in milliseconds
		currentTime = clockMillis();
		dt = (currentTime - oldTime);
		oldTime = currentTime;
		
		// function dispatch according to state
		if     (navState == NAV_STATE_ASSESS)			updateNavStateAssess(dt);
		else if(navState == NAV_STATE_FORWARD_START)	updateNavStateForwardStart(dt);
		else if(navState == NAV_STATE_FORWARD_END)		updateNavStateForwardEnd(dt);
		else if(navState == NAV_STATE_ROTATING)			updateNavStateRotating(dt);
		
		// update LEDs to indicate state
		ledClear();
		ledSet((uint8_t)navState, 0, 50, 0);
		ledRefresh();

		// shut down if button is pressed
		if(buttonDown())
		{
			_delay_ms(50);
			while(buttonDown());
			navDone = 1;
		}

		// show low-power warnings or shut down if needed
		utilHandleLowPower();

		// debugging output useful for testing
		//readCalibratedSensors(sensors);
		//serialTX("%4d  %4d  %4d\r\n", (int)sensors[0], (int)sensors[1], (int)sensors[2]);
	}
}

static void updateNavStateAssess(uint32_t dt)
{
	// have we arrived?
	if(targetX == posX && targetY == posY)
	{
		// we're done; stop
		motorLeft(0);
		motorRight(0);

		// new-position
		if(targetX == cornerX && targetY == cornerY)
		{
			targetX = 0;
			targetY = 0;
		}
		else
		{
			cornerX ++;
			cornerY ++;
			targetX = cornerX;
			targetY = cornerY;
		}
	}
	else
	{
		// determine which global heading we need to move in
		if     (targetX < posX) travelDir = DIR_WEST;
		else if(targetX > posX) travelDir = DIR_EAST;
		else if(targetY < posY) travelDir = DIR_SOUTH;
		else if(targetY > posY) travelDir = DIR_NORTH;

		// are we facing the correct way?
		if(heading == travelDir)
		{
			// start moving forward
			navState = NAV_STATE_FORWARD_START;
		}
		else
		{
			// nope; start turning in the direction we need to go; time it to be just about the amount we need
			navState = NAV_STATE_ROTATING;
			setMotorRotation(travelDir);

			ledClear();
			if((heading + 1) % 4 == travelDir || (travelDir + 1) % 4 == heading)
			{
				ledSet(0, 50, 0, 0);
				ledRefresh();
				_delay_ms(550);
			}
			else
			{
				ledSet(1, 0, 0, 50);
				ledRefresh();
				_delay_ms(1400);
			}
		}
	}
}

static void updateNavStateForwardStart(uint32_t dt)
{
	// simple; just follow the line!
	readCalibratedSensors(sensors);
	followLine();

	// wait to exit a grid line we might be stepping on
	if(sensors[0] < 40 && sensors[2] < 40)
	{
		// we've exited the line, now wait for the next one
		navState = NAV_STATE_FORWARD_END;
	}
}

static void updateNavStateForwardEnd(uint32_t dt)
{
	// simple; just follow the line!
	readCalibratedSensors(sensors);
	followLine();

	// wait to firmly step on a new grid line
	if(sensors[0] > 65 && sensors[1] > 65 && sensors[2] > 65)
	{
		// update our position
		if     (heading == DIR_NORTH) posY ++;
		else if(heading == DIR_EAST) posX ++;
		else if(heading == DIR_SOUTH) posY --;
		else if(heading == DIR_WEST) posX --;

		// move forward just slightly
		_delay_ms(100);

		// figure out what to do next
		navState = NAV_STATE_ASSESS;
	}
}

static void updateNavStateRotating(uint32_t dt)
{
	// read grid
	readCalibratedSensors(sensors);

	// are we aligned with the new direction we want to travel in?
	if(sensors[1] > 75)
	{
		// update heading based on rotation direction
		heading = travelDir;

		// figure out what to do next
		motorLeft(0);
		motorRight(0);
		navState = NAV_STATE_ASSESS;
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

static void setMotorRotation(Dir targetDir)
{
	if(heading != targetDir)
	{
		if(heading == DIR_NORTH)
		{
			if(targetDir == DIR_WEST)
			{
				motorLeft(-TURN_SPEED);
				motorRight(TURN_SPEED);
				rotatingLeft = 1;
			}
			else
			{
				motorLeft(TURN_SPEED);
				motorRight(-TURN_SPEED);
				rotatingLeft = 0;
			}
		}
		else if(heading == DIR_EAST)
		{
			if(targetDir == DIR_NORTH)
			{
				motorLeft(-TURN_SPEED);
				motorRight(TURN_SPEED);
				rotatingLeft = 1;
			}
			else
			{
				motorLeft(TURN_SPEED);
				motorRight(-TURN_SPEED);
				rotatingLeft = 0;
			}
		}
		else if(heading == DIR_SOUTH)
		{
			if(targetDir == DIR_EAST)
			{
				motorLeft(-TURN_SPEED);
				motorRight(TURN_SPEED);
				rotatingLeft = 1;
			}
			else
			{
				motorLeft(TURN_SPEED);
				motorRight(-TURN_SPEED);
				rotatingLeft = 0;
			}
		}
		else if(heading == DIR_WEST)
		{
			if(targetDir == DIR_SOUTH)
			{
				motorLeft(-TURN_SPEED);
				motorRight(TURN_SPEED);
				rotatingLeft = 1;
			}
			else
			{
				motorLeft(TURN_SPEED);
				motorRight(-TURN_SPEED);
				rotatingLeft = 0;
			}
		}
	}
}

/*
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
*/