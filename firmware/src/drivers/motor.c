#include "drivers/motor.h"

#include <avr/io.h>

#include <util/delay.h>

#include <stdlib.h>
#include <stdint.h>

// - - - defines - - - //

#define MOTOR_1A_PORT PORTD
#define MOTOR_1A_DIR DDRD
#define MOTOR_1A_PIN 6

#define MOTOR_1B_PORT PORTD
#define MOTOR_1B_DIR DDRD
#define MOTOR_1B_PIN 5

#define MOTOR_2A_PORT PORTB
#define MOTOR_2A_DIR DDRB
#define MOTOR_2A_PIN 3

#define MOTOR_2B_PORT PORTD
#define MOTOR_2B_DIR DDRD
#define MOTOR_2B_PIN 3

#define SPEED_LIMIT 100

// - - - prototypes - - - //

void _disableLeftMotorOutputs();
void _disableRightMotorOutputs();
int8_t _computeValidSpeed(int8_t speed);
uint8_t _isDirDifferent(int8_t old, int8_t newDir);

// - - - static members - - - //

// set by the motorUseHardBrake() function
static uint8_t _useHardBrake = 1;

// we track motor speeds and direction so we can make intelligent decisions about turning
// on or off our output compare pins
static int8_t _leftSpeed = 0;
static int8_t _rightSpeed = 0;

// used to scale motor speeds if they aren't perfectly matched
static float _leftScale = 1.0f;
static float _rightScale = 1.0f;

// - - - implementations - - - //

void motorInit()
{
	// [note]: during initial testing, I've observed some cases in which motors trying to run at full speed will fail to run; I have some
	// theories about this, but we'll just avoid running the motors at a full 100% duty cycle for now; for simplicity, we just double the
	// incoming motor speeds (-100 to 100) when they're received, giving us a maximum duty cycle of 200 / 255 = 78%; we should revisit this
	// in the future if higher speeds are desired, but for now this doesn't seem to be a real issue for a small tabletop robot

	// immediately keep motor pins low
	MOTOR_1A_PORT &= ~(1 << MOTOR_1A_PIN);
	MOTOR_1B_PORT &= ~(1 << MOTOR_1B_PIN);
	MOTOR_2A_PORT &= ~(1 << MOTOR_2A_PIN);
	MOTOR_2B_PORT &= ~(1 << MOTOR_2B_PIN);

	// set motor 1 pins as outputs
	MOTOR_1A_DIR |= (1 << MOTOR_1A_PIN);
	MOTOR_1B_DIR |= (1 << MOTOR_1B_PIN);

	// set motor 2 pins as outputs
	MOTOR_2A_DIR |= (1 << MOTOR_2A_PIN);
	MOTOR_2B_DIR |= (1 << MOTOR_2B_PIN);

	// disable motor outputs
	_disableLeftMotorOutputs();
	_disableRightMotorOutputs();

	// setup timer 0 for motor 1
	TCCR0A |= (1 << WGM00) | (1 << WGM01);					// fast PWM mode with TOP defined as 0xff
	TCCR0B |= (1 << CS00) | (1 << CS02);					// initial tests show that a /1024 prescalar, while a little choppy at low duty cycles, works best
															// out of all the prescalars available to both timer 0 and timer 2

	// setup timer 2 for motor 2
	TCCR2A |= (1 << WGM20) | (1 << WGM21);					// fast PWM mode with TOP defined as 0xff
	TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22);		// /1024 prescalar; [note]: different registers are used for this prescalar than timer 0

	// initialize our speeds
	_useHardBrake = 1;
	_leftSpeed = 0;
	_rightSpeed = 0;
	_leftScale = 1.0f;
	_rightScale = 1.0f;
}

void motorUseHardBrake(uint8_t useHardBrake)
{
	// enable hard brake
	_useHardBrake = useHardBrake;

	// re-input motor speeds for any-zero speed motors
	if(_leftSpeed == 0) motorLeft(0);
	if(_rightSpeed == 0) motorRight(0);
}

void motorLeft(int8_t speed)
{
	// keep in valid range after applying a scaling
	speed = _computeValidSpeed((int8_t)((float)speed * _leftScale));

	// first, disable left output entirely if we've changed direction
	if(_isDirDifferent(_leftSpeed, speed))
	{
		_disableLeftMotorOutputs();
		_delay_ms(5);
	}

	// are we going forwards?
	if(speed > 0)
	{
		// set compare value and enable output compare pin for OC0A
		OCR0A = abs(speed) * 2;
		TCCR0A |= (1 << COM0A1);
	}
	// are we going backwards?
	else if(speed < 0)
	{
		// set compare value and enable output compare pin for OC0B
		OCR0B = abs(speed) * 2;
		TCCR0A |= (1 << COM0B1);
	}
	// we've stopped
	else
	{
		// brake according to braking settings
		if(_useHardBrake)
		{
			// set left motor pins high to enable hard brake
			MOTOR_1A_PORT |= (1 << MOTOR_1A_PIN);
			MOTOR_1B_PORT |= (1 << MOTOR_1B_PIN);
		}
	}

	// record current speed
	_leftSpeed = speed;
}

void motorRight(int8_t speed)
{
	// keep in valid range after applying a scaling
	speed = _computeValidSpeed((int8_t)((float)-speed * _rightScale));

	// first, disable right output entirely if we've changed direction
	if(_isDirDifferent(_rightSpeed, speed))
	{
		_disableRightMotorOutputs();
		_delay_ms(5);
	}

	// are we going forwards?
	if(speed > 0)
	{
		// set compare value and enable output compare pin for OC2A
		OCR2A = abs(speed) * 2;
		TCCR2A |= (1 << COM2A1);
	}
	// are we going backwards?
	else if(speed < 0)
	{
		// set compare value and enable output compare pin for OC2B
		OCR2B = abs(speed) * 2;
		TCCR2A |= (1 << COM2B1);
	}
	// we've stopped
	else
	{
		// brake according to braking settings
		if(_useHardBrake)
		{
			// set right motor pins high to enable hard brake
			MOTOR_2A_PORT |= (1 << MOTOR_2A_PIN);
			MOTOR_2B_PORT |= (1 << MOTOR_2B_PIN);
		}
	}

	// record current speed
	_rightSpeed = speed;
}

void motorScaleLeft(float scale)
{
	_leftScale = scale;
}

void motorScaleRight(float scale)
{
	_rightScale = scale;
}

void _disableLeftMotorOutputs()
{
	// disconnect left motor output compare pins
	TCCR0A &= ~((1 << COM0A0) | (1 << COM0A1));
	TCCR0A &= ~((1 << COM0B0) | (1 << COM0B1));

	// set left motor pins low; this puts us in coast mode
	MOTOR_1A_PORT &= ~(1 << MOTOR_1A_PIN);
	MOTOR_1B_PORT &= ~(1 << MOTOR_1B_PIN);
}

void _disableRightMotorOutputs()
{
	// disconnect right motor output compare pins
	TCCR2A &= ~((1 << COM2A0) | (1 << COM2A1));
	TCCR2A &= ~((1 << COM2B0) | (1 << COM2B1));

	// set right motor pins low; this puts us in coast mode
	MOTOR_2A_PORT &= ~(1 << MOTOR_2A_PIN);
	MOTOR_2B_PORT &= ~(1 << MOTOR_2B_PIN);
}

int8_t _computeValidSpeed(int8_t speed)
{
	if     (speed < -SPEED_LIMIT) speed = -SPEED_LIMIT;
	else if(speed > SPEED_LIMIT)  speed = SPEED_LIMIT;
	return speed;
}

uint8_t _isDirDifferent(int8_t oldVal, int8_t newVal)
{
	return (oldVal > 0 && newVal <= 0) ||
		   (oldVal < 0 && newVal >= 0) ||
		   (oldVal == 0 && newVal != 0);
}
