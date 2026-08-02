#include "drivers/linesensor.h"
#include "drivers/adc.h"

#include <avr/io.h>

#include <util/delay.h>

#include <stdint.h>

// - - - defines - - - //

// the pin that controls enabling/disabling of the active IR used by the sensors
#define SENSOR_ENABLE_PORT PORTB
#define SENSOR_ENABLE_DIR DDRB
#define SENSOR_ENABLE_PIN 2

// pin for line sensor 1 (left)
#define SENSOR_1_PORT PORTC
#define SENSOR_1_DIR DDRC
#define SENSOR_1_PIN 1
#define SENSOR_1_CHANNEL 1

// pin for line sensor 2 (middle)
#define SENSOR_2_PORT PORTC
#define SENSOR_2_DIR DDRC
#define SENSOR_2_PIN 2
#define SENSOR_2_CHANNEL 2

// pin for line sensor 3 (right)
#define SENSOR_3_PORT PORTC
#define SENSOR_3_DIR DDRC
#define SENSOR_3_PIN 3
#define SENSOR_3_CHANNEL 3

// maybe the design will change in the future to allow more line sensors; who knows?
#define NUM_LINE_SENSORS 3

// number of samples we take; this is high because raw readings are noisy; increasing this
// gives smoother results, but at a slower rate
#define NUM_SAMPLES 20

// - - - implementations - - - //

void lineSensorInit()
{
	// set sensor enable pin as output, but disable it by default
	SENSOR_ENABLE_DIR |= (1 << SENSOR_ENABLE_PIN);
	SENSOR_ENABLE_PORT &= ~(1 << SENSOR_ENABLE_PIN);

	// setup sensor pins
	SENSOR_1_DIR &= ~(1 << SENSOR_1_PIN);
	SENSOR_1_PORT &= ~(1 << SENSOR_1_PIN);
	SENSOR_2_DIR &= ~(1 << SENSOR_2_PIN);
	SENSOR_2_PORT &= ~(1 << SENSOR_2_PIN);
	SENSOR_3_DIR &= ~(1 << SENSOR_3_PIN);
	SENSOR_3_PORT &= ~(1 << SENSOR_3_PIN);
}

void lineSensorRead(uint16_t *sensors)
{
	int8_t i;
	uint8_t j;

	// wipe out old values
	for(i = 0; i < NUM_LINE_SENSORS; ++ i)
	{
		sensors[i] = 0;
	}

	// do fast dummy reads to clear things out and clear our initial sensor values
	for(i = 0; i < NUM_LINE_SENSORS; ++ i)
	{
		adcSetChannel(SENSOR_1_CHANNEL + i);
		adcRead();
	}

	// accumulate samples so we can average later
	for(j = 0; j < NUM_SAMPLES; ++ j)
	{
		// turn on emitters
		SENSOR_ENABLE_PORT |= (1 << SENSOR_ENABLE_PIN);
		_delay_us(100);

		// alternate between starting at left or right sensors
		for(i = (j % 2 == 1 ? 0                        : NUM_LINE_SENSORS - 1);
			    (j % 2 == 1 ? i < NUM_LINE_SENSORS     : i >= 0);
				(j % 2 == 1 ? ++ i                     : -- i))
		{
			// switch channels, take a dummy reading, turn on the IR emitter, wait, turn off the emitter, and take the actual reading
			adcSetChannel(SENSOR_1_CHANNEL + i);
			adcRead();
			sensors[i] += adcRead();
		}

		// turn off and cool down
		SENSOR_ENABLE_PORT &= ~(1 << SENSOR_ENABLE_PIN);
		_delay_us(10);
	}
	
	// average everything out
	for(i = 0; i < NUM_LINE_SENSORS; ++ i)
	{
		sensors[i] /= NUM_SAMPLES;
	}
}
