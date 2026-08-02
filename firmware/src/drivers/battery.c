#include "drivers/battery.h"
#include "drivers/adc.h"

#include <avr/io.h>

#include <stdint.h>
#include <math.h>

// - - - defines - - - //

// battery input port and pin
#define BATTERY_PORT PORTC
#define BATTERY_DIR DDRC
#define BATTERY_PIN 0
#define BATTERY_CHANNEL 0

// these are the resistor values of our voltage divider
#define DIVIDER_R1 510UL
#define DIVIDER_R2 1870UL

// reference voltage we use, in millivolts
#define REFERENCE_VOLTAGE_MV 3300UL

// - - - globals - - - //

uint16_t _rawBatteryValue;
uint16_t _millivolts;

// - - - implementations - - - //

void batteryInit()
{
	// set battery pin as input and low
	BATTERY_DIR &= ~(1 << BATTERY_PIN);
	BATTERY_PORT &= ~(1 << BATTERY_PIN);

	// set default reading
	_rawBatteryValue = 0;
}

void batteryRead()
{
	uint32_t divided;

	// perform a read from the channel connected to our voltage divider
	adcSetChannel(BATTERY_CHANNEL);
	_rawBatteryValue = adcRead();

	// convert from ADC to divided voltage
	divided = (_rawBatteryValue * REFERENCE_VOLTAGE_MV) / 1024;
	
	// convert from divided to actual voltage
	_millivolts = (divided * (DIVIDER_R1 + DIVIDER_R2)) / DIVIDER_R2;
}

uint16_t batteryGetMillivolts()
{
	return _millivolts;
}
